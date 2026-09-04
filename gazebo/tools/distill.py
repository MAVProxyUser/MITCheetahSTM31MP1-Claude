#!/usr/bin/env python3
"""Turn raw shm_trace snapshots into a small, judged dataset - then let the
raw ones go.

Operator, 2026-09-04: "rundata seems like it needs computed, parsed,
correlated, and judgement made no need to keep 6 gig of unusable logs if you
are done with them."

Right. A 20,000-record ring per run is the right thing to CAPTURE and the
wrong thing to KEEP: everything OPEN-26 actually uses is a handful of derived
quantities plus the ~2 s around the descent. This writes, per snapshot:

  * a summary row - outcome, attitude, the descent rate/duration/depth, the
    estimator-vs-detector height gap, peak leg retraction, inferred contact;
  * the descent window itself, downsampled to 50 Hz - enough to re-plot or
    re-measure the transient that all the interesting findings live in.

INFERRED CONTACT is computed here rather than read from a sensor, on purpose:
world foot speed < threshold, from the IMU and joint encoders alone. The
operator's EDU dog has no contact sensors, so any signal this project comes to
rely on has to be one the real robot can produce.

  distill.py [--archive DIR] [--out DIR] [--prune]

--prune deletes a raw snapshot ONLY after its distilled record is written and
read back successfully, and never touches one whose distillation failed.
"""
import argparse, json, os, sys, glob, statistics as st

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import paths

CONTACT_TH = 0.15          # m/s; see ISSUES OPEN-26 for where this came from

def plateau(v):
    return st.median(sorted(v)[len(v) // 2:])

def distill_one(path):
    d = json.load(open(path))
    R = [x for x in (d.get("records") or []) if x.get("z") is not None]
    if len(R) < 200:
        return None
    has_kin = R[0].get("kin_z") is not None
    has_spd = R[0].get("foot_fz0") is not None
    ez = [x["z"] for x in R]
    pe = plateau(ez)
    last = R[-1]
    ro, pi = last["roll"] * 57.2958, last["pitch"] * 57.2958
    kind = ("LEVEL" if (abs(ro) < 10 and abs(pi) < 15)
            else "TIP" if (abs(ro) > 25 or abs(pi) > 25) else "MIXED")
    # descent window: last time at plateau-0.03 down to plateau-0.15
    s = e = None
    for i in range(len(ez) - 1, -1, -1):
        if ez[i] <= pe - 0.15 and e is None: e = i
        if e is not None and ez[i] >= pe - 0.03: s = i; break
    rec = dict(file=os.path.basename(path), reason=d.get("reason"),
               run_id=d.get("run_id"), n=len(R), span_s=round(d.get("span_s") or 0, 2),
               kind=kind, end_roll=round(ro, 2), end_pitch=round(pi, 2),
               end_z=round(last["z"], 4), plateau=round(pe, 4),
               has_kin=has_kin, has_speed=has_spd)
    win = []
    # A window needs real DURATION, not just two samples. Without this guard
    # dz/dt divides by ~0 on a 1-2 sample window and produces values like
    # 1.4e6 m/s, which then poison any mean taken over the set. Caught by the
    # number being absurd on its face; a subtler one would not have been.
    MIN_WINDOW_S = 0.05
    if (s is not None and e is not None and e > s
            and (R[e]["t"] - R[s]["t"]) >= MIN_WINDOW_S):
        # PRE-ROLL. The first version started the window at descent onset,
        # which threw away the only interval that can answer whether anything
        # happens BEFORE the body starts down - the causal question. 2 s of
        # lead-in costs ~100 extra samples per run at 50 Hz.
        PRE_S = 2.0
        s_pre = s
        while s_pre > 0 and (R[s]["t"] - R[s_pre]["t"]) < PRE_S:
            s_pre -= 1
        W = R[s:e + 1]
        WP = R[s_pre:e + 1]
        dt = W[-1]["t"] - W[0]["t"]
        rec.update(desc_dt=round(dt, 4),
                   desc_dz=round(W[0]["z"] - W[-1]["z"], 4),
                   desc_rate=round((W[0]["z"] - W[-1]["z"]) / dt, 4) if dt >= MIN_WINDOW_S else None,
                   desc_vz_min=round(min(x["vz"] for x in W), 4),
                   went_negative=bool(min(x["z"] for x in W) < -0.001))
        if has_kin:
            rec["kin_gap_max"] = round(max(x["z"] - x["kin_z"] for x in W), 4)
            rec["retract_max"] = round(max(pe - x["kin_z"] for x in W), 4)
        if has_spd:
            # inferred contact: how many feet are slow enough to be planted
            feet = [sum(1 for i in range(4) if x["foot_fz%d" % i] < CONTACT_TH) for x in W]
            sched = [sum(1 for i in range(4) if x["c%d" % i] > 0) for x in W]
            rec["inferred_feet_min"] = min(feet)
            rec["inferred_feet_mean"] = round(sum(feet) / len(feet), 2)
            rec["sched_feet_mean"] = round(sum(sched) / len(sched), 2)
            rec["all_feet_off_ticks"] = sum(1 for f in feet if f == 0)
        # 50 Hz window, only the fields anything actually reads
        rec["onset_t"] = round(R[s]["t"], 4)
        step = max(1, int(round(0.02 / max(1e-6, (W[-1]["t"] - W[0]["t"]) / len(W)))))
        for x in WP[::step]:
            row = dict(t=round(x["t"], 4), z=round(x["z"], 4), vz=round(x["vz"], 4),
                       roll=round(x["roll"], 4), pitch=round(x["pitch"], 4))
            if has_kin: row["kin_z"] = round(x["kin_z"], 4)
            if has_spd:
                row["fs"] = [round(x["foot_fz%d" % i], 3) for i in range(4)]
                row["ph"] = [round(x["c%d" % i], 3) for i in range(4)]
            win.append(row)
    rec["window"] = win
    return rec

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--archive", default=paths.ARCHIVE_DIR)
    ap.add_argument("--out", default=os.path.join(paths.DATA_ROOT, "distilled"))
    ap.add_argument("--prune", action="store_true")
    ap.add_argument("--keep-recent", type=int, default=15,
                    help="raw snapshots to keep at full resolution even when "
                         "pruning. The distilled window is 50 Hz over the "
                         "descent, which is everything the current analyses "
                         "read - but a handful of complete rings should "
                         "survive for a question nobody has asked yet.")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    files = sorted(glob.glob(os.path.join(a.archive, "*.json")))
    keep = set(sorted(files, key=os.path.getmtime)[-a.keep_recent:]) if a.keep_recent else set()
    ok = fail = pruned = 0
    freed = 0
    for f in files:
        outp = os.path.join(a.out, os.path.basename(f))
        if os.path.exists(outp):
            rec = None
        else:
            try:
                rec = distill_one(f)
            except Exception as ex:
                sys.stderr.write("distill FAILED %s: %s\n" % (os.path.basename(f), ex))
                fail += 1
                continue
            if rec is None:
                fail += 1
                continue
            json.dump(rec, open(outp, "w"))
        ok += 1
        if a.prune:
            # read the distilled file back before deleting anything
            try:
                json.load(open(outp))
            except Exception:
                sys.stderr.write("distilled file unreadable, KEEPING raw: %s\n" % f)
                continue
            if f in keep:
                continue
            sz = os.path.getsize(f)
            os.remove(f)
            pruned += 1
            freed += sz
    print("distilled %d, failed %d" % (ok, fail))
    if a.prune:
        print("pruned %d raw snapshots, freed %.2f GB" % (pruned, freed / 1e9))
    tot = sum(os.path.getsize(x) for x in glob.glob(os.path.join(a.out, "*.json")))
    print("distilled set is %.1f MB" % (tot / 1e6))

if __name__ == "__main__":
    main()
