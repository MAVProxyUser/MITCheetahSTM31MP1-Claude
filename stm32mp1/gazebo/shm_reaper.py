#!/usr/bin/env python3
"""SHM ring-buffer reaper for the per-tick control-loop trace.

Companion to stm32mp1/gazebo/ShmTrace.h (the writer side - read that
file's own header comment for the full design rationale: root-causing a
fleet-only fast-fall that clean control-loop timing already rules out as
a scheduling stall, which means the cause lives in the CONTROL DATA at a
resolution the existing printf logging - gated behind env vars because a
full-rate printf would itself perturb the timing this project has
repeatedly found to matter - has never had.

This is a REAPER, not the writer: it never runs inside the control loop
it is watching. Per direct instruction, it uses raw ctypes bindings to
POSIX shm_open()/mmap() rather than Python's multiprocessing.shared_memory
wrapper - guarantees byte-for-byte interop with the C++ side (same OS
primitive, same name string) without depending on that module's own
internal naming conventions, which are not documented as stable across
platforms/versions.

Two ways to use it - "even the launcher itself can reap":
  - as a library: `from shm_reaper import dump_snapshot; dump_snapshot(0, "crash_reason")`
    lets mission_runner.py or server.py pull a dog's trace on demand, the
    instant they see a FALL/ESTOP line in the orchestration log - no
    separate process needed for that path.
  - standalone: `python3 shm_reaper.py --watch 0,1,2 --archive-dir DIR`
    runs a continuous watch over N dogs' segments, auto-archiving the
    moment a "FALL" or "recover_giveup" tagged record shows up in the
    stream - the crash "oracle" this was built for, accumulating
    evidence across runs instead of each one vanishing when the
    controller process exits.

Byte layout - MUST track ShmTrace.h exactly, verified by direct
measurement (not hand-derived - a hand sum of the Record fields was
wrong twice before landing on the right number):
  Header  (24 bytes): uint64 write_seq, uint32 capacity, uint32
           record_size, uint32 magic, uint32 writer_pid - the last field
           fills what used to be 4 bytes of trailing pad (the struct's own
           8-byte alignment from the leading atomic<uint64_t>), so this
           did not grow the struct. writer_pid exists so watch() (below)
           can tell "a new process reused this instance number" apart from
           "the same process kept ticking" - write_seq alone cannot: a
           fresh run's write_seq restarts at 0 and can land ABOVE or below
           the previous run's count by coincidence, which was measured
           live to silently swallow a whole run's crash tag before this
           field existed (see watch()'s own comment).
  Record  (98 bytes, #pragma pack(1) on the C++ side so this is exact):
           double t; uint64 seq; char tag[20]; float roll,pitch,yaw;
           float wx,wy,wz; float vx,vy,vz; float z; float period_ms;
           float contact[4]; uint8 op_mode; uint8 finite.
  Ring = Header immediately followed by capacity*Record, no gap (a
  pack(1) struct imposes no alignment requirement on where it sits,
  confirmed by measuring offsetof(Ring, records) == sizeof(Header)).
"""
import argparse
import ctypes
import json
import mmap
import os
import struct
import sys
import time

HEADER_FMT = "<QIIII"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 24, f"Header format drifted from ShmTrace.h (got {HEADER_SIZE}, want 24)"

RECORD_FMT = "<dQ20s" + "f" * 15 + "BB"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
assert RECORD_SIZE == 98, f"Record format drifted from ShmTrace.h's Record (got {RECORD_SIZE}, want 98)"

RING_CAPACITY = 65536
MAGIC = 0x43484554  # "CHET", little-endian bytes happen to spell nothing readable - just a sentinel

# Field names, in the SAME order as RECORD_FMT's unpacked tuple - used to
# turn each decoded record into a dict for JSON archiving / easy grepping.
_FIELDS = ["t", "seq", "tag", "roll", "pitch", "yaw", "wx", "wy", "wz",
           "vx", "vy", "vz", "z", "period_ms",
           "c0", "c1", "c2", "c3", "op_mode", "finite"]

# CRASH-WORTHY tags: any record with one of these causes a standalone
# --watch run to archive the whole ring immediately. "tick" (the per-tick
# trace) never triggers this on its own - it is the CONTEXT a crash tag
# gets archived alongside, not itself an event.
_CRASH_TAGS = {"FALL", "recover_giveup"}

_libc = ctypes.CDLL(None, use_errno=True)
_libc.shm_open.restype = ctypes.c_int
_libc.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
_libc.shm_unlink.restype = ctypes.c_int
_libc.shm_unlink.argtypes = [ctypes.c_char_p]

O_RDONLY = 0


def _shm_name(instance):
    return f"/cheetah_trace_{instance}".encode()


def attach(instance):
    """Open the named shm segment read-only and mmap it.
    Returns (fd, mmap_obj, ring_size) or (None, None, None) if the
    segment does not exist yet (the dog process never logged, or has
    already been reaped and unlinked)."""
    fd = _libc.shm_open(_shm_name(instance), O_RDONLY, 0o666)
    if fd < 0:
        return None, None, None
    ring_size = HEADER_SIZE + RING_CAPACITY * RECORD_SIZE
    try:
        m = mmap.mmap(fd, ring_size, prot=mmap.PROT_READ)
    except Exception:
        os.close(fd)
        return None, None, None
    header = struct.unpack_from(HEADER_FMT, m, 0)
    write_seq, capacity, record_size, magic, writer_pid = header
    if magic != MAGIC:
        # Either a stale segment from an unrelated process, or the writer
        # is mid-ensure_open() (a few instructions between shm_open and
        # the header being stamped) - either way, do not trust the data.
        m.close()
        os.close(fd)
        return None, None, None
    return fd, m, ring_size


def read_all(instance):
    """Every currently-valid record in the ring, oldest first, as a list
    of dicts. "Currently valid" = the ring has not wrapped past it since
    it was written - if write_seq > capacity, the oldest (write_seq -
    capacity) records are already overwritten and are skipped rather
    than guessed at."""
    fd, m, _ = attach(instance)
    if m is None:
        return []
    try:
        write_seq = struct.unpack_from("<Q", m, 0)[0]
        first_seq = max(0, write_seq - RING_CAPACITY)
        out = []
        for seq in range(first_seq, write_seq):
            off = HEADER_SIZE + (seq % RING_CAPACITY) * RECORD_SIZE
            vals = struct.unpack_from(RECORD_FMT, m, off)
            d = dict(zip(_FIELDS, vals))
            d["tag"] = d["tag"].split(b"\x00", 1)[0].decode("ascii", "replace")
            out.append(d)
        return out
    finally:
        m.close()
        os.close(fd)


def dump_snapshot(instance, reason, archive_dir="/tmp/cheetah_conductor/archive/shm_trace",
                   run_id=None):
    """Write the CURRENT full contents of dog `instance`'s ring to a
    timestamped JSON file under archive_dir - the "oracle" entry point.
    Safe to call from ANY process (the launcher, server.py, a standalone
    watch loop) at any time, including after the writer process has
    already exited (shm segments outlive the process that created them
    until explicitly unlinked - see ShmTrace.h's Writer destructor
    comment for why that is deliberate). Returns the archive path, or
    None if there was nothing to read (segment never existed).
    """
    records = read_all(instance)
    if not records:
        return None
    os.makedirs(archive_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    run_tag = f"run{run_id}_" if run_id is not None else ""
    path = os.path.join(archive_dir, f"{ts}_{run_tag}dog{instance}_{reason}.json")
    with open(path, "w") as f:
        json.dump({
            "instance": instance,
            "reason": reason,
            "run_id": run_id,
            "captured_at": ts,
            "n_records": len(records),
            "span_s": (records[-1]["t"] - records[0]["t"]) if len(records) > 1 else 0.0,
            "records": records,
        }, f)
    print(f"[shm_reaper] archived {len(records)} records ({records[-1]['t']-records[0]['t']:.1f}s span) "
          f"for dog{instance} ({reason}) -> {path}", flush=True)
    return path


def unlink(instance):
    """Remove the segment once its archive (if any) has been captured -
    otherwise POSIX shm segments on macOS/Linux persist across process
    exits indefinitely and accumulate across every run in a session."""
    _libc.shm_unlink(_shm_name(instance))


def watch(instances, archive_dir, poll_s=0.5):
    """Standalone continuous watch: poll each instance's ring for new
    records, and the moment any _CRASH_TAGS tag appears, archive that
    dog's full current ring and print a note. Runs until interrupted.
    This is the "separate process reaps it" mode from the design brief -
    intended to run for the life of a conductor session (started once,
    watches every dog that ever appears), not per-mission.
    """
    last_seq = {i: 0 for i in instances}
    archived_this_run = {i: False for i in instances}
    last_pid = {i: None for i in instances}
    print(f"[shm_reaper] watching instances {instances}, archive_dir={archive_dir}", flush=True)
    while True:
        for i in instances:
            fd, m, _ = attach(i)
            if m is None:
                continue
            try:
                # Identify the SEGMENT's WRITER, not just its write_seq. A
                # new run reusing this instance number calls shm_unlink()+
                # shm_open(O_CREAT) in ShmTrace.h's ensure_open(), and its
                # write_seq restarts at 0 and climbs independently of the
                # previous run's count - it can land ABOVE *or* below the
                # old high-water mark by coincidence, so write_seq alone
                # cannot tell "the same run kept going" apart from "a new
                # run happens to have a similar tick count" (measured live:
                # a 131-record run following a 61-record run on the same
                # instance was silently never scanned, because 131 is not
                # less than 61, and the leftover archived_this_run=True from
                # the FIRST run's crash then permanently suppressed the
                # second one's). st_ino cannot help either - macOS reports
                # st_ino=0 for every POSIX shm fd. writer_pid (stamped in
                # ensure_open(), read fresh here every poll) is what
                # actually changes between two different processes.
                write_seq, writer_pid = struct.unpack_from("<Q12xI", m, 0)
            finally:
                m.close(); os.close(fd)
            if writer_pid != last_pid[i]:
                # Fresh writer: reset the high-water mark so `start` below
                # is computed against THIS run's own numbering, and re-arm
                # archived_this_run - a stale True left over from the
                # previous run on this instance would otherwise silently
                # suppress archiving for every run after the first crash
                # this instance number ever saw.
                last_pid[i] = writer_pid
                last_seq[i] = 0
                archived_this_run[i] = False
            if write_seq == last_seq[i]:
                continue
            # Scan only the NEW records since the last poll for a crash tag -
            # cheap even at 500 Hz writer rate against a 0.5s poll (a few
            # hundred records, not the whole ring).
            fd, m, _ = attach(i)
            if m is None:
                continue
            try:
                start = max(last_seq[i], write_seq - RING_CAPACITY)
                for seq in range(start, write_seq):
                    off = HEADER_SIZE + (seq % RING_CAPACITY) * RECORD_SIZE
                    tag_bytes = struct.unpack_from("20s", m, off + 16)[0]
                    tag = tag_bytes.split(b"\x00", 1)[0].decode("ascii", "replace")
                    if tag in _CRASH_TAGS and not archived_this_run[i]:
                        archived_this_run[i] = True
                        m.close(); os.close(fd)
                        dump_snapshot(i, tag, archive_dir)
                        fd, m, _ = attach(i)
                        if m is None:
                            break
            finally:
                if m is not None:
                    m.close(); os.close(fd)
            last_seq[i] = write_seq
        time.sleep(poll_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", help="comma-separated dog instance indices to watch continuously, e.g. 0,1,2")
    ap.add_argument("--dump", type=int, help="dump a single instance's current ring once and exit")
    ap.add_argument("--reason", default="manual", help="tag for --dump's archive filename")
    ap.add_argument("--archive-dir", default="/tmp/cheetah_conductor/archive/shm_trace")
    ap.add_argument("--poll", type=float, default=0.5, help="watch poll interval, seconds")
    args = ap.parse_args()

    if args.dump is not None:
        path = dump_snapshot(args.dump, args.reason, args.archive_dir)
        sys.exit(0 if path else 1)
    if args.watch:
        instances = [int(x) for x in args.watch.split(",")]
        watch(instances, args.archive_dir, args.poll)
        return
    ap.error("need --watch or --dump")


if __name__ == "__main__":
    main()
