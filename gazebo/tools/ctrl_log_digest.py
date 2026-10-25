#!/usr/bin/env python3
"""Digest the archived ctrl logs to one small table, so the logs can be deleted.

Operator, 2026-10-24: delete the bridge dumps, the conductor logs + reports, and
the yawtruth jsonl. This runs first, because the ctrl log is the ONLY place a
few irreplaceable quantities live:

  * the orientation E-stop AND the attitude at it. `feedback-classify-at-the-event`
    exists because reading the final pose instead invented a phantom "level
    collapse" mode that cost two days and four dead hypotheses. It is NOT in a
    shm_trace - traces carry no ESTOP tag at all (measured over all 5,907).
  * `[Recovery Balance] body height is X`, printed on every RecoveryStand entry.
    The whole OPEN-40 arithmetic - ~2.29 mm of bleed per cycle, the
    cycles_to_fold budget, the bleed-vs-headroom race - is derived from this
    series and from nowhere else.
  * the per-leg trip lines, whose `, N ticks` suffix DATES THE BINARY: the
    debounced build prints a tick count and the pre-debounce one cannot, which
    is how era A/B/C were separated without trusting the calendar.
  * the 1:1 trips-to-RecoveryStand-entries signature that IS the limit cycle.

Per log it emits ONE row: counts of each marker plus the first/last of the ones
whose VALUE matters. Counts, not lines, because a log line exists only when its
condition fired - `feedback-a-log-line-is-a-selected-sample`. A count per run is
a population; a pile of matched lines is a selection.

usage: ctrl_log_digest.py [--src DIR] [--out CSV] [--jobs N]
This script never deletes.
"""
import os, re, csv, glob, argparse
from multiprocessing import Pool

# Marker -> column. Kept deliberately short; anything not here is recoverable
# from the campaign CSVs, which are only 4 MB and are NOT being deleted.
COUNTED = [
    ("trip_legy",    "y-position is bad"),
    ("trip_legv",    "is moving too quickly"),
    ("trip_attitude", "Unsafe locomotion: roll"),
    ("unsafe_any",   "Unsafe locomotion"),
    ("recov_entry",  "[Recovery Balance] body height"),
    ("fold",         "Folding legs"),
    ("locoadv",      "[locoadv]"),
    ("lococap",      "[lococap]"),
    ("legv_absorb",  "[legv]"),
    ("legkin_absorb", "[legkin]"),
    ("leghold",      "[leghold]"),
    ("stall",        "[STALL]"),
    ("estop",        "Orientation safety check failed"),
]

RE_HEIGHT = re.compile(r"\[Recovery Balance\] body height is ([-\d.]+)")
RE_ESTOP = re.compile(r"roll=([-\d.]+) pitch=([-\d.]+) deg, peak ([-\d.]+), held (\d+) ms")
RE_FALL = re.compile(r"\[FALL\][^\n]*?roll=([-\d.]+) deg pitch=([-\d.]+) deg z=([-\d.]+)")
RE_RESULT = re.compile(r"\[mission\] RESULT: (\w+)")
RE_LEGY = re.compile(r"y-position is bad \(([-\d.]+) m")
# filename: <stamp>_run<N>_ctrl_<i>.log
RE_NAME = re.compile(r"^(\d{8}_\d{6})_run(\d+)_(ctrl|bridge)_(\d+)$")

HDR = (["file", "stamp", "run_id", "kind", "dog"]
       + [c for c, _ in COUNTED]
       + ["recov_h_first", "recov_h_last", "recov_h_min", "bled_mm",
          "legy_first_mm", "legy_max_mm",
          "estop_roll", "estop_pitch", "estop_peak", "estop_held_ms",
          "fall_roll", "fall_pitch", "fall_z", "result"])


def digest_one(path):
    base = os.path.basename(path)
    if base.endswith(".log"):
        base = base[:-4]
    m = RE_NAME.match(base)
    stamp, run_id, kind, dog = m.groups() if m else (base, "", "", "")
    try:
        with open(path, errors="replace") as fh:
            text = fh.read()
    except Exception:
        return None

    row = dict(file=base, stamp=stamp, run_id=run_id, kind=kind, dog=dog)
    for col, marker in COUNTED:
        row[col] = text.count(marker)

    hs = [float(x) for x in RE_HEIGHT.findall(text)]
    row["recov_h_first"] = round(hs[0], 4) if hs else ""
    row["recov_h_last"] = round(hs[-1], 4) if hs else ""
    row["recov_h_min"] = round(min(hs), 4) if hs else ""
    # the bleed: entry height minus the lowest the cycle reached, in mm
    row["bled_mm"] = round((hs[0] - min(hs)) * 1000.0, 1) if hs else ""

    ly = [float(x) for x in RE_LEGY.findall(text)]
    row["legy_first_mm"] = round(abs(ly[0]) * 1000.0, 1) if ly else ""
    row["legy_max_mm"] = round(max(abs(v) for v in ly) * 1000.0, 1) if ly else ""

    e = RE_ESTOP.search(text)
    row["estop_roll"], row["estop_pitch"], row["estop_peak"], row["estop_held_ms"] = (
        e.groups() if e else ("", "", "", ""))
    f = RE_FALL.search(text)
    row["fall_roll"], row["fall_pitch"], row["fall_z"] = f.groups() if f else ("", "", "")
    r = RE_RESULT.search(text)
    row["result"] = r.group(1) if r else ""
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=os.path.expanduser(
        "~/Desktop/Cheetah/rundata/conductor/archive"))
    ap.add_argument("--out", default=os.path.expanduser(
        "~/Desktop/Cheetah/rundata/distilled/ctrl_digest.csv"))
    ap.add_argument("--jobs", type=int, default=6)
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(a.src, "*ctrl_*.log")))
    print("digesting %d ctrl logs" % len(files))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    n = 0
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=HDR, extrasaction="ignore")
        w.writeheader()
        with Pool(a.jobs) as pool:
            for row in pool.imap_unordered(digest_one, files, chunksize=32):
                if row:
                    w.writerow(row); n += 1
                    if n % 5000 == 0:
                        print("  %d/%d" % (n, len(files)), flush=True)
    print("wrote %d rows, %.1f MB -> %s" % (n, os.path.getsize(a.out) / 1e6, a.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
