#!/usr/bin/env python3
"""Condense the whole shm_trace archive to per-second statistics, so the raw
per-tick traces can be deleted.

Operator, 2026-09-21, on the 63 GB of `.json.zst` in conductor/archive/shm_trace:
"remove this. keep what ever you need, or process it to be smaller usable stats".

WHAT THIS KEEPS, AND WHY THAT SHAPE. The single most decisive comparison this
project has run on a trace is "peak |pitch| per second beside mean body speed",
three runs side by side - it is what showed the weave's pitch event is a
REPRODUCIBLE feature of the reversal re-acceleration (18-23 deg in passing runs)
that occasionally runs away, rather than a planning error. That needs one row
per SECOND, not one per tick: a 120 s run becomes ~120 rows instead of ~40,000,
which is the ~1000x reduction that makes 63 GB into tens of MB.

It also keeps, per run, the quantities the open issues actually cite: peak
attitude, max/mean body speed, min estimated and KINEMATIC height, and - per
`feedback-classify-at-the-event` - the time of the first E-stop AND the attitude
AT it, because reading the final pose instead invented a phantom "level collapse"
mode that cost two days and four dead hypotheses.

WHAT IT THROWS AWAY: per-tick resolution. A future question that genuinely needs
individual ticks (the 8-consecutive-identical-samples freeze that motivated
CTRL_LEG_HELD_GATE, say) cannot be answered from this output. That is the trade
the operator asked for, and it is stated here rather than discovered later.

TWO CORRECTNESS POINTS, both learned the hard way:
  * `tag` is tick | ESTOP | FALL | recover_*. Peaks are taken over `tick` ONLY -
    the post-E-stop flop otherwise contaminates every one of them.
  * roll/pitch/yaw in the trace are RADIANS (gazebo/ShmTrace.h). Read as degrees
    they give 0.4-0.6 and look plausible. Emitted here as degrees.
The record regex and the radian conversion are IMPORTED from trace_slim rather
than retyped, because that file's field order was verified against ShmTrace.h's
writer and a second copy is a second thing to drift.

This script NEVER deletes. It writes the condensed output and reports, per file,
which parse path was used and which files failed; pruning is a separate step
that must read this output back first.

usage: trace_condense.py [--src DIR] [--out DIR] [--jobs N] [--limit N]
"""
import os, sys, re, csv, math, json, glob, argparse, subprocess
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trace_slim import REC, D            # verified field order + rad->deg
import snapio

# Filenames look like
#   20260908_125146_dog0_a150r4_open28_corner_PASS.json.zst
# i.e. <stamp>_<dog>_<arm>_<campaign>_<VERDICT>. The verdict and arm are only
# in the name, so they are parsed out here - losing them with the raw file
# would make the condensed rows much harder to group.
NAME = re.compile(r"^(\d{8}_\d{6})_(dog\d+)_(.*)_([A-Z]+)$")

SEC_HDR = ["stamp", "dog", "arm_campaign", "verdict", "sec", "n", "post_stand",
           "pitch_max", "roll_max", "spd_mean", "spd_max",
           "z_min", "kinz_min", "contacts_mean"]

SUM_HDR = ["file", "stamp", "dog", "arm_campaign", "verdict", "parse",
           "n_ticks", "span_s", "t_stood", "pitch_max", "roll_max",
           "spd_max", "spd_mean",
           "z_min", "z_max", "kinz_min", "contacts_mean",
           "t_estop", "pitch_at_estop", "roll_at_estop", "t_fall", "other_tags"]

# THE BOOT WINDOW MUST BE EXCLUDED FROM EVERY PER-RUN PEAK, and this is not a
# tuning choice - it is a real artifact with a known cause. The robot spawns
# belly-down by design, and tick seq=0 is written BEFORE the first sensor packet
# arrives, so it carries `roll: 3.14159265` (pi rad = exactly 180 deg) at
# `z: 2e-10`. That is OPEN-6's uninitialised `VectorNavData` showing through.
# Left in, it makes `roll_max` read 180.0 for every run in the archive, PASSES
# INCLUDED - measured on the first three files condensed, which is why this
# exists. By seq=1 the same run reads roll -1.6e-06 at z 0.214.
# 0.25 m is NOT a guess: it is `RobotRunner.cpp:551,588`'s own `stood` latch
# ("clearly standing, not mid-transient"), which exists for this exact reason.
STOOD_Z = 0.25


def recs_fast(text):
    """(t, tag, pitch_deg, roll_deg, speed, z, contacts, kin_z) as floats."""
    for m in REC.finditer(text):
        t, tag, roll, pitch, wy, vx, vy, z, c0, c1, c2, c3, kz = m.groups()
        nc = sum(1 for c in (c0, c1, c2, c3) if float(c) > 0.5)
        yield (float(t), tag, abs(float(pitch)) * D, abs(float(roll)) * D,
               math.hypot(float(vx), float(vy)), float(z), nc, float(kz))


def recs_slow(text):
    for r in json.loads(text).get("records", []):
        if "vx" not in r or r.get("z") is None:
            continue
        yield (float(r["t"]), r.get("tag", ""),
               abs(float(r["pitch"])) * D, abs(float(r["roll"])) * D,
               math.hypot(float(r["vx"]), float(r["vy"])), float(r["z"]),
               sum(1 for k in ("c0", "c1", "c2", "c3") if r.get(k, 0) > 0.5),
               float(r.get("kin_z") or 0.0))


def condense_one(path):
    base = os.path.basename(path)
    for ext in (".json.zst", ".json.gz", ".json"):
        if base.endswith(ext):
            base = base[: -len(ext)]
            break
    m = NAME.match(base)
    stamp, dog, arm, verdict = m.groups() if m else (base, "", "", "")

    try:
        text = snapio.read_bytes(path).decode("utf-8", "replace")
    except Exception as e:
        return None, None, dict(file=base, err="read: %s" % e)

    parse = "regex"
    recs = list(recs_fast(text))
    if not recs:                       # never guess - fall back and SAY so
        parse = "json"
        try:
            recs = list(recs_slow(text))
        except Exception as e:
            return None, None, dict(file=base, err="parse: %s" % e)
    if not recs:
        return None, None, dict(file=base, err="no records")
    del text

    ticks = [r for r in recs if r[1] == "tick"]
    if not ticks:
        return None, None, dict(file=base, err="no tick records")

    # The stand latch, mirroring RobotRunner: first tick clearly above the
    # belly-down spawn. Peaks come from here onward; the per-second rows keep
    # the boot seconds but flag them, so nobody later mistakes a trace for
    # having started at cruise.
    t_stood = None
    for t, _tag, _pi, _ro, _sp, z, _nc, _kz in ticks:
        if z > STOOD_Z:
            t_stood = t
            break
    scored = [r for r in ticks if t_stood is not None and r[0] >= t_stood]
    if not scored:                     # never stood: score everything, say so
        scored = ticks

    # per-second buckets, ticks only
    buckets = {}
    for t, _tag, pi, ro, sp, z, nc, kz in ticks:
        buckets.setdefault(int(t), []).append((pi, ro, sp, z, nc, kz, t))
    sec_rows = []
    for sec in sorted(buckets):
        b = buckets[sec]
        post = 1 if (t_stood is not None and b[0][6] >= t_stood) else 0
        sec_rows.append([stamp, dog, arm, verdict, sec, len(b), post,
                         round(max(x[0] for x in b), 2),
                         round(max(x[1] for x in b), 2),
                         round(sum(x[2] for x in b) / len(b), 3),
                         round(max(x[2] for x in b), 3),
                         round(min(x[3] for x in b), 4),
                         round(min(x[5] for x in b), 4),
                         round(sum(x[4] for x in b) / len(b), 2)])

    # the EVENT, not the end: first E-stop and the attitude at it
    t_estop = pi_estop = ro_estop = t_fall = ""
    others = set()
    for t, tag, pi, ro, _sp, _z, _nc, _kz in recs:
        if tag == "tick":
            continue
        others.add(tag)
        if tag == "ESTOP" and t_estop == "":
            t_estop, pi_estop, ro_estop = round(t, 3), round(pi, 2), round(ro, 2)
        if tag == "FALL" and t_fall == "":
            t_fall = round(t, 3)

    summary = dict(
        file=base, stamp=stamp, dog=dog, arm_campaign=arm, verdict=verdict,
        parse=parse, n_ticks=len(ticks),
        span_s=round(ticks[-1][0] - ticks[0][0], 2),
        t_stood=("" if t_stood is None else round(t_stood, 3)),
        pitch_max=round(max(r[2] for r in scored), 2),
        roll_max=round(max(r[3] for r in scored), 2),
        spd_max=round(max(r[4] for r in scored), 3),
        spd_mean=round(sum(r[4] for r in scored) / len(scored), 3),
        z_min=round(min(r[5] for r in scored), 4),
        z_max=round(max(r[5] for r in scored), 4),
        kinz_min=round(min(r[7] for r in scored), 4),
        contacts_mean=round(sum(r[6] for r in scored) / len(scored), 2),
        t_estop=t_estop, pitch_at_estop=pi_estop, roll_at_estop=ro_estop,
        t_fall=t_fall, other_tags="|".join(sorted(others)))
    return sec_rows, summary, None


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.expanduser(
        "~/Desktop/Cheetah/rundata/conductor/archive/shm_trace"))
    ap.add_argument("--out", default=os.path.expanduser(
        "~/Desktop/Cheetah/rundata/distilled"))
    ap.add_argument("--jobs", type=int, default=4,
                    help="keep modest: this analysis is itself a host tenant")
    ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args(argv[1:])

    files = sorted(glob.glob(os.path.join(a.src, "*.json*")))
    if a.limit:
        files = files[: a.limit]
    if not files:
        print("no traces under %s" % a.src)
        return 1
    os.makedirs(a.out, exist_ok=True)
    sec_path = os.path.join(a.out, "trace_seconds.csv")
    sum_path = os.path.join(a.out, "trace_summary.csv")
    err_path = os.path.join(a.out, "trace_condense_errors.txt")
    print("condensing %d traces -> %s" % (len(files), a.out))

    n_ok = n_err = n_sec = 0
    paths = {"regex": 0, "json": 0}
    with open(sec_path, "w", newline="") as sf, \
         open(sum_path, "w", newline="") as mf, \
         open(err_path, "w") as ef:
        sw = csv.writer(sf); sw.writerow(SEC_HDR)
        mw = csv.DictWriter(mf, fieldnames=SUM_HDR); mw.writeheader()
        with Pool(a.jobs) as pool:
            for i, (sec_rows, summary, err) in enumerate(
                    pool.imap_unordered(condense_one, files, chunksize=4), 1):
                if err:
                    n_err += 1
                    ef.write("%s\t%s\n" % (err["file"], err["err"]))
                    ef.flush()
                else:
                    n_ok += 1
                    n_sec += len(sec_rows)
                    paths[summary["parse"]] += 1
                    sw.writerows(sec_rows)
                    mw.writerow(summary)
                if i % 250 == 0:
                    print("  %5d/%d  ok=%d err=%d  second-rows=%d"
                          % (i, len(files), n_ok, n_err, n_sec), flush=True)

    print("\ndone: %d ok, %d failed  (parse: %d regex, %d json fallback)"
          % (n_ok, n_err, paths["regex"], paths["json"]))
    print("  %-22s %8d rows  %7.1f MB" % ("trace_seconds.csv", n_sec,
                                          os.path.getsize(sec_path) / 1e6))
    print("  %-22s %8d rows  %7.1f MB" % ("trace_summary.csv", n_ok,
                                          os.path.getsize(sum_path) / 1e6))
    if n_err:
        print("  %d FAILURES listed in %s - do NOT prune while this is nonzero"
              % (n_err, err_path))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
