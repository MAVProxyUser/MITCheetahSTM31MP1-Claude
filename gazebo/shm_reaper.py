#!/usr/bin/env python3
"""SHM ring-buffer reaper for the per-tick control-loop trace.

Companion to gazebo/ShmTrace.h (the writer side - read that
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
  Header  (32 bytes): uint64 write_seq, uint32 capacity, uint32
           record_size, uint32 magic, uint32 writer_pid, uint32 run_id,
           uint32 _pad. run_id is the conductor's $SIM_RUN_ID - the SAME
           number the panel, the orchestration log, every archived
           filename and each ctrl log's [RUNID] stamp use, so run identity
           is checkable in the ring itself. writer_pid - the field
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

# MUST mirror ShmTrace.h::Header exactly. run_id was appended (with an
# explicit _pad to keep write_seq's 8-byte alignment) so the conductor's
# run number is carried in shared memory itself, not only in a log line -
# see that struct's own comment for why identity in the ring matters.
HEADER_FMT = "<QIIIIII"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 32, f"Header format drifted from ShmTrace.h (got {HEADER_SIZE}, want 32)"

RECORD_FMT = "<dQ20s" + "f" * 24 + "BB"
RECORD_SIZE = struct.calcsize(RECORD_FMT)
assert RECORD_SIZE == 134, f"Record format drifted from ShmTrace.h's Record (got {RECORD_SIZE}, want 134)"

RING_CAPACITY = 65536
MAGIC = 0x43484554  # "CHET", little-endian bytes happen to spell nothing readable - just a sentinel

# ---- text ring: the printf replacement -----------------------------------
# Companion to ShmTrace.h's TextRecord/TextRing/logf() - see that file's own
# header for why this is a SEPARATE ring rather than a generic redesign of
# Record above. Same Header layout as the numeric ring (capacity/record_size
# differ, which is how a generic reader would tell the two apart, though in
# practice each is read by name - "/cheetah_trace_<instance>" vs
# "/cheetah_trace_text_<instance>").
TEXT_RECORD_FMT = "<dQ200s"
TEXT_RECORD_SIZE = struct.calcsize(TEXT_RECORD_FMT)
assert TEXT_RECORD_SIZE == 216, f"TextRecord format drifted from ShmTrace.h (got {TEXT_RECORD_SIZE}, want 216)"

TEXT_RING_CAPACITY = 8192

# Field names, in the SAME order as RECORD_FMT's unpacked tuple - used to
# turn each decoded record into a dict for JSON archiving / easy grepping.
_FIELDS = ["t", "seq", "tag", "roll", "pitch", "yaw", "wx", "wy", "wz",
           "vx", "vy", "vz", "z", "period_ms",
           "c0", "c1", "c2", "c3",
           # added 2026-09-03 with the matching ShmTrace.h Record change -
           # kin_z is the height the FALL DETECTOR actually uses (it prefers
           # this over the estimate); foot_z* are the per-leg terms it is the
           # min of. See OPEN-26.
           "kin_z", "foot_z0", "foot_z1", "foot_z2", "foot_z3",
           # 2026-09-04: per-leg WORLD FOOT SPEED (m/s). Began as a force
           # field; there is no measured force in this build, and the
           # operator's EDU dog has no contact sensors either, so the
           # sensorless test - a planted foot does not move - is the one
           # that transfers. Names kept to avoid a third layout change.
           "foot_fz0", "foot_fz1", "foot_fz2", "foot_fz3",
           "op_mode", "finite"]

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


def _text_shm_name(instance):
    return f"/cheetah_trace_text_{instance}".encode()


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
    write_seq, capacity, record_size, magic, writer_pid, run_id, _pad = header
    if magic != MAGIC:
        # Either a stale segment from an unrelated process, or the writer
        # is mid-ensure_open() (a few instructions between shm_open and
        # the header being stamped) - either way, do not trust the data.
        m.close()
        os.close(fd)
        return None, None, None
    # LAYOUT MISMATCH - SAY SO, LOUDLY.
    #
    # The writer stamps the record size it is actually using. Until
    # 2026-09-03 nothing compared it to the size this module is about to
    # unpack at, and the cost was immediate: a Record layout change (98 ->
    # 118 bytes for kin_z/foot_z) was edited into this file WHILE campaign
    # c26 was running against a deployed binary still writing 98-byte
    # records. Every dump from that moment on returned None, the campaign
    # wrote "NONE" in its snapshot column without complaint, and 14 of 16
    # runs lost their trace - discovered only when the analysis came back
    # with 2 rows.
    #
    # Returning None was the LUCKY outcome. Had the sizes been closer, this
    # would have unpacked at the wrong stride and produced plausible
    # garbage - numbers that go into the record and are believed. A reader
    # whose format does not match the writer must fail loudly, never
    # quietly return nothing.
    if record_size != RECORD_SIZE:
        sys.stderr.write(
            "shm_reaper: RECORD LAYOUT MISMATCH on instance %s - the writer "
            "is using %d-byte records, this reader unpacks %d. The deployed "
            "binary and this file are from different commits; rebuild and "
            "redeploy (gazebo/deploy_host.sh) before trusting any trace.\n"
            % (instance, record_size, RECORD_SIZE))
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


def attach_text(instance):
    """Same as attach() above, but for the TEXT ring
    ("/cheetah_trace_text_<instance>") - the printf replacement's segment,
    written by ShmTrace.h's logf(). Returns (fd, mmap_obj, ring_size) or
    (None, None, None) if it does not exist yet."""
    fd = _libc.shm_open(_text_shm_name(instance), O_RDONLY, 0o666)
    if fd < 0:
        return None, None, None
    ring_size = HEADER_SIZE + TEXT_RING_CAPACITY * TEXT_RECORD_SIZE
    try:
        m = mmap.mmap(fd, ring_size, prot=mmap.PROT_READ)
    except Exception:
        os.close(fd)
        return None, None, None
    magic = struct.unpack_from(HEADER_FMT, m, 0)[3]
    if magic != MAGIC:
        m.close()
        os.close(fd)
        return None, None, None
    return fd, m, ring_size


def read_all_text(instance):
    """Every currently-valid text record in the ring, oldest first, as a
    list of {"t", "seq", "msg"} dicts - the human-readable event trail
    ([nav]/[mission]/[gait]/[FALL]/[recover]/... lines) alongside
    read_all()'s numeric physical-state trail."""
    fd, m, _ = attach_text(instance)
    if m is None:
        return []
    try:
        write_seq = struct.unpack_from("<Q", m, 0)[0]
        first_seq = max(0, write_seq - TEXT_RING_CAPACITY)
        out = []
        for seq in range(first_seq, write_seq):
            off = HEADER_SIZE + (seq % TEXT_RING_CAPACITY) * TEXT_RECORD_SIZE
            t, s, msg = struct.unpack_from(TEXT_RECORD_FMT, m, off)
            out.append({"t": t, "seq": s,
                        "msg": msg.split(b"\x00", 1)[0].decode("utf-8", "replace")})
        return out
    finally:
        m.close()
        os.close(fd)


def _pid_alive(pid):
    """Is this pid a live process? Used to tell a running controller's SHM
    ring from a dead one's leftovers - see the call site for the false-PASS
    that motivated it. signal 0 performs the existence/permission check
    without delivering anything. A pid of 0 is never a real writer here."""
    if not pid:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True          # exists, just not ours to signal
    except OSError:
        return False
    return True


def tail_text_to_log(instance, log_path, poll_s=0.2, expect_run_id=None):
    """Continuously append new text-ring records for `instance` to
    `log_path`, in the order they were written - this IS the printf
    replacement's other half: RobotRunner.cpp/mit_sim_main.cpp no longer
    write these lines to stdout at all (see ShmTrace.h's file header), so
    without this running, server.py's/mission_runner.py's existing regex
    parsing of ctrl_%d.log (waypoint progress, mission result, FALL,
    gait changes, ...) would see nothing from those two files ever again.
    Opens `log_path` in APPEND mode and never truncates it - server.py
    still opens the SAME path itself in "w" mode to capture the
    controller process's own raw stdout/stderr (whatever printf survives
    in files this pass did not convert, plus any crash output), so the
    two writers interleave onto one growing file exactly like two
    processes tailing -f into the same target; each write here is one
    line, one write() syscall, which POSIX guarantees is atomic against
    the other writer's own line-buffered appends.
    Runs until interrupted - meant to be launched once per dog alongside
    that dog's controller process and torn down with it (see server.py's
    launch/stop, which now spawns and tracks this next to the controller
    Popen). Same cross-run writer_pid reset logic as watch() below (see
    that function's own comment for why write_seq alone cannot detect a
    new process reusing this instance number).
    """
    last_seq = 0
    last_pid = None
    with open(log_path, "a", buffering=1) as f:   # line-buffered
        while True:
            fd, m, _ = attach_text(instance)
            if m is None:
                time.sleep(poll_s)
                continue
            try:
                write_seq, writer_pid, run_id = struct.unpack_from("<Q12xII", m, 0)
                if writer_pid != last_pid:
                    last_pid = writer_pid
                    # NEVER REPLAY A DEAD WRITER'S RING. The segment
                    # (/cheetah_trace_text_N) outlives the process that made
                    # it - ShmTrace only shm_unlink()s at STARTUP, not at
                    # exit - so between runs the previous controller's ring
                    # is still sitting there, full, with its final records
                    # intact. server.py spawns this reaper microseconds after
                    # the controller, and the controller needs ~1 s to boot
                    # before it unlinks and recreates the segment, so the
                    # first attach lands on the OLD run's ring essentially
                    # every time. Replaying it from seq 0 dumped the previous
                    # mission's entire tail - including its "MISSION COMPLETE"
                    # and "[mission] RESULT: PASS" lines - into the new run's
                    # freshly truncated ctrl_%d.log.
                    #
                    # That is a FALSE PASS generator, and it produced one:
                    # run430 (oval, trotRunning @3.5) was declared
                    # "COMPLETE t=212.8s PASS" NINE SECONDS after launch,
                    # reporting run429's parallel-course time, because
                    # _start_poller scans the whole ctrl log for exactly
                    # those markers. Caught only because the [RUNID] stamp
                    # added earlier the same evening said run=429 in a file
                    # that claimed to be run430.
                    #
                    # A live writer's ring is real data and is replayed in
                    # full (that is the whole point of the ring - the reaper
                    # must not miss lines written before it attached). A DEAD
                    # writer's ring is history that already has an archived
                    # log file of its own, so skip to its end rather than
                    # re-emitting it. When the real controller then creates
                    # its segment the pid changes again and this branch
                    # re-fires with a live pid, replaying that ring properly
                    # from 0 - so nothing the new run writes is ever lost.
                    # STALENESS, decided on the RUN ID first and the pid
                    # only as a fallback. $SIM_RUN_ID is the same token the
                    # conductor prints, every archived filename carries, and
                    # each ctrl log's [RUNID] stamp shows - so "is this ring
                    # mine?" is now answerable against the number a human
                    # reads, not just an OS pid that gets reused and means
                    # nothing in a log. A ring whose run_id is older than the
                    # one we were told to follow is history with an archived
                    # file of its own; skip to its end instead of replaying
                    # it into a fresh log (that replay produced a false PASS
                    # - see the long note below).
                    stale = (expect_run_id is not None and run_id
                             and run_id != expect_run_id)
                    if not stale and not _pid_alive(writer_pid):
                        stale = True
                    last_seq = write_seq if stale else 0
                if write_seq != last_seq:
                    start = max(last_seq, write_seq - TEXT_RING_CAPACITY)
                    for seq in range(start, write_seq):
                        off = HEADER_SIZE + (seq % TEXT_RING_CAPACITY) * TEXT_RECORD_SIZE
                        msg = struct.unpack_from("200s", m, off + 16)[0]
                        msg = msg.split(b"\x00", 1)[0].decode("utf-8", "replace")
                        f.write(msg + "\n")
                    last_seq = write_seq
            finally:
                # fd/m are only ever set right above (never carried over
                # from a previous iteration), so this always closes exactly
                # what THIS iteration opened - no double-close, no leak,
                # regardless of which branch above ran.
                m.close(); os.close(fd)
            time.sleep(poll_s)


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
    # The human-readable event trail alongside the numeric one - "here's
    # everything around the crash" in one bundle, rather than needing a
    # second manual pull of the text ring right after this archives the
    # numeric one. Best-effort: a dog that fell before ever calling logf()
    # (impossible in practice - the FIRST logf() call site is the boot
    # banner - but the text ring not existing must never block archiving
    # the numeric trace, which is the one this function's caller actually
    # cares about) just gets an empty list here instead of failing.
    text_records = read_all_text(instance)
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
            "n_text_records": len(text_records),
            "text_log": text_records,
        }, f)
    print(f"[shm_reaper] archived {len(records)} records ({records[-1]['t']-records[0]['t']:.1f}s span) "
          f"+ {len(text_records)} text lines for dog{instance} ({reason}) -> {path}", flush=True)
    return path


def unlink(instance):
    """Remove BOTH the numeric and text segments once their archive (if
    any) has been captured - otherwise POSIX shm segments on macOS/Linux
    persist across process exits indefinitely and accumulate across every
    run in a session."""
    _libc.shm_unlink(_shm_name(instance))
    _libc.shm_unlink(_text_shm_name(instance))


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
                write_seq, writer_pid, run_id = struct.unpack_from("<Q12xII", m, 0)
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
    ap.add_argument("--poll", type=float, default=0.5, help="watch/tail-text poll interval, seconds")
    ap.add_argument("--tail-text", type=int, metavar="INSTANCE",
                     help="continuously append INSTANCE's text ring to --append-to "
                          "(the printf replacement's bridge - see tail_text_to_log())")
    ap.add_argument("--expect-run-id", type=int, default=None,
                     help="only follow a ring stamped with THIS $SIM_RUN_ID; a "
                          "ring from any other run is treated as history and "
                          "skipped to its end rather than replayed into the new "
                          "log. One run number across SHM, conductor and logs.")
    ap.add_argument("--append-to", metavar="PATH",
                     help="log file --tail-text appends to, e.g. ctrl_<i>.log")
    args = ap.parse_args()

    if args.dump is not None:
        path = dump_snapshot(args.dump, args.reason, args.archive_dir)
        sys.exit(0 if path else 1)
    if args.tail_text is not None:
        if not args.append_to:
            ap.error("--tail-text needs --append-to")
        tail_text_to_log(args.tail_text, args.append_to, args.poll,
                         args.expect_run_id)
        return
    if args.watch:
        instances = [int(x) for x in args.watch.split(",")]
        watch(instances, args.archive_dir, args.poll)
        return
    ap.error("need --watch, --dump, or --tail-text")


if __name__ == "__main__":
    main()
