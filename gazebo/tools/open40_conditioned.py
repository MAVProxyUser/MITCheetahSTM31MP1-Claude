#!/usr/bin/env python3
"""Score an OPEN-40 advisory A/B by CONDITIONING on whether the guard fired.

WHY THIS EXISTS (2026-09-17, ISSUES OPEN-40 decision #4).

At the SHIPPED CTRL_MAX_PLEG_Y (0.240) only about 7 % of runs trip the leg-y
guard at all.  The advisory knob can only change the outcome of a run where the
guard FIRES - on every other run the two arms are running identical code.  So a
marginal pass rate mixes a handful of informative runs into a majority of runs
that carry no information, and it dilutes the effect toward nothing.  Chain CW
block 1 is the worked example: marginally stock 4/6 vs advisory 5/6, which says
nothing at all, while conditioned on tripping it is stock 0/2 vs advisory 2/2.

It also keeps two mistakes out of the numbers:

  * A fall in the advisory arm is only the advisory's if the guard actually
    engaged.  CW's 9859 fell with ZERO leg-y trips, on a bare pitch E-stop
    0.05 deg over the 28.65 limit - counting it against the advisory would be
    wrong, and the marginal rate does exactly that.
  * The `[locoadv]` log line is throttled at 20 per run, so its count is NOT the
    trip count.  The throttle-proof manipulation check is ">= 1 advisory line AND
    zero RecoveryStand cycles", which is what this asserts.

The continuous endpoint is RecoveryStand CYCLES and the height BLED, not the
verdict: the hazard is the sustained cycle, the bleed runs at ~2.29 mm a cycle,
and the advisory drives the cycle count to zero by construction.

The per-run facts come from each run's own ctrl log, which is UNCOMPRESSED in the
archive (only shm_trace/*.json.zst is packed), so this is cheap - a few hundred
small text reads, not a snapshot decompression.

usage:  python3 gazebo/tools/open40_conditioned.py [csv-glob]
        python3 gazebo/tools/open40_conditioned.py 'advship_cw*.csv'
"""
import csv, glob, os, re, sys, statistics as st
from math import comb

DATA = os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata"))
CAMP = os.path.join(DATA, "campaigns")
ARCH = os.path.join(DATA, "conductor", "archive")
PAT  = sys.argv[1] if len(sys.argv) > 1 else "advship_cw*.csv"

TRIP   = re.compile(r"y-position is bad \((-?[\d.]+) m, max ([\d.]+), (\d+) ticks\)")
RECOV  = re.compile(r"\[Recovery Balance\] body height is ([\d.]+)")
LOCOADV= re.compile(r"\[locoadv\]")
LOCOCAP= re.compile(r"\[lococap\]")
ORIENT = re.compile(r"Orientation safety check failed![^\n]*")

def fisher(a, b, c, d):
    n = a + b + c + d
    if n == 0 or (a + c) == 0:
        return 1.0
    obs = comb(a + b, a) * comb(c + d, c) / comb(n, a + c)
    tot = 0.0
    for i in range(0, min(a + b, a + c) + 1):
        k = a + c - i
        if (a + b - i) < 0 or k < 0 or (c + d - k) < 0:
            continue
        pr = comb(a + b, i) * comb(c + d, k) / comb(n, a + c)
        if pr <= obs + 1e-12:
            tot += pr
    return tot

rows = []
for f in sorted(glob.glob(os.path.join(CAMP, PAT))):
    rows += list(csv.DictReader(open(f)))
if not rows:
    sys.exit("no rows matched %s under %s" % (PAT, CAMP))

facts = []
for r in rows:
    rid = (r.get("run_id") or "").strip()
    logs = glob.glob(os.path.join(ARCH, "*_run%s_ctrl_0.log" % rid)) if rid else []
    if not logs:
        # A run's log is live until the NEXT launch archives it, so the most
        # recent row routinely has none yet. Say so rather than scoring it as 0.
        facts.append((r, None)); continue
    t = open(logs[0], errors="replace").read()
    hs = [float(m.group(1)) for m in RECOV.finditer(t)]
    o  = ORIENT.search(t)
    facts.append((r, {
        "trips":  len(TRIP.findall(t)),
        "adv":    len(LOCOADV.findall(t)),
        "cap":    len(LOCOCAP.findall(t)),
        "cycles": len(hs),
        "bled":   (hs[0] - min(hs)) * 1000 if hs else 0.0,
        "entry":  hs[0] if hs else 0.0,
        "orient": o.group(0) if o else "",
    }))

unread = sum(1 for _, f in facts if f is None)
print("  OPEN-40 conditioned scoring: %d rows from %s%s" % (
    len(rows), PAT, ("  (%d without an archived log yet - excluded)" % unread) if unread else ""))
print("  Conditioning on whether the leg-y guard FIRED, because the arms run identical")
print("  code on every run where it did not.\n")

arms = sorted({r["course"] for r in rows})
cond = {}
for arm in arms:
    sel = [(r, f) for r, f in facts if r["course"] == arm and f is not None]
    trp = [(r, f) for r, f in sel if f["trips"] > 0]
    non = [(r, f) for r, f in sel if f["trips"] == 0]
    cond[arm] = (sum(1 for r, _ in trp if r["verdict"] == "PASS"), len(trp),
                 sum(1 for r, _ in non if r["verdict"] == "PASS"), len(non))
    cyc = [f["cycles"] for _, f in trp]
    print("  %-9s  tripped %2d/%-2d PASS   never tripped %2d/%-2d PASS   trips %4d  cycles %4d (per tripping run: %s)  bled max %4.0f mm  [locoadv] %3d  [lococap] %3d" % (
        arm, cond[arm][0], cond[arm][1], cond[arm][2], cond[arm][3],
        sum(f["trips"] for _, f in sel), sum(f["cycles"] for _, f in sel),
        ",".join(str(c) for c in sorted(cyc, reverse=True)[:8]) or "-",
        max([f["bled"] for _, f in sel] or [0]), sum(f["adv"] for _, f in sel),
        sum(f["cap"] for _, f in sel)))

# Pairwise, because a three-arm chain (stock vs advisory vs a cycle cap) has no
# single comparison - what matters is whether the cap matches the advisory AND
# whether it beats stock, and those are different questions.
if len(arms) >= 2:
    print("\n  CONDITIONED on tripping, pairwise:")
    for i in range(len(arms)):
        for j in range(i + 1, len(arms)):
            a, b = cond[arms[i]], cond[arms[j]]
            print("    %-9s %d/%-2d  vs  %-9s %d/%-2d   Fisher p = %.4f" % (
                arms[i], a[0], a[1], arms[j], b[0], b[1],
                fisher(a[0], a[1]-a[0], b[0], b[1]-b[0])))
    print("  Marginal (all runs, the DILUTED numbers):")
    for arm in arms:
        c = cond[arm]
        print("    %-9s %d/%-2d" % (arm, c[0]+c[2], c[1]+c[3]))

print("\n  MANIPULATION CHECK - per run, not pooled:")
bad = 0
for r, f in facts:
    if f is None or f["trips"] == 0:
        continue
    arm = r["course"]
    # Three arm kinds, three different expectations. A CAPPED run routes its
    # suppressed trips through the same branch as the advisory, so it emits
    # [locoadv] too - expecting [locoadv] == 0 there would flag correct behaviour
    # as a violation. The distinguishing line is [lococap], and the distinguishing
    # NUMBER is the cycle count: the advisory abolishes cycles, the cap BOUNDS
    # them, so a cap arm must show cycles > 0 and cycles near its cap.
    if arm.startswith("cap"):
        ok  = f["cap"] > 0 and f["cycles"] > 0
        why = "a cap arm must show [lococap] > 0 and cycles > 0 (bounded, not abolished)"
    elif "advisory" in arm or arm == "adv":
        ok  = f["adv"] > 0 and f["cycles"] == 0
        why = "an advisory arm must show [locoadv] > 0 and ZERO cycles"
    else:
        ok  = f["adv"] == 0 and f["cap"] == 0
        why = "a stock arm must show neither [locoadv] nor [lococap]"
    if not ok:
        bad += 1
        print("    VIOLATION  run %s (%s): trips %d, [locoadv] %d, [lococap] %d, cycles %d" % (
            r.get("run_id"), arm, f["trips"], f["adv"], f["cap"], f["cycles"]))
        print("               %s" % why)
print("    %s" % ("all tripping runs consistent with their arm" if bad == 0 else
                  "%d row(s) inconsistent - the knob may not have taken; do not score those blocks" % bad))

# THE BUDGET MODEL, because it is now a validated predictor rather than a story.
# Height bleeds at ~2.29 mm per RecoveryStand cycle (fitted on 11 historical runs),
# and a run has (entry_h - 0.20 m) of it to spend before reaching the fold line. So
# spent/budget = cycles / ((entry_h - 0.20) / 2.29). Pooled over CW and CY, 17
# stock tripping runs at two different trigger values sorted by this ratio:
# every one of the 13 at >= 1.26 fell, four of the five below 1.20 survived,
# Fisher p = 2.1e-03. Quote the ratio, not the raw cycle count - a run that enters
# low has a small budget and dies on fewer cycles (one entered at 0.227 m and was
# 13.6x overspent at 159 cycles).
BLEED_MM_PER_CYCLE = 2.29
FOLD_M = 0.20
budget_rows = [(r, f) for r, f in facts
               if f is not None and f["cycles"] > 0 and f["entry"] > FOLD_M]
if budget_rows:
    print("\n  BUDGET MODEL - cycles / ((entry_h - %.2f m) / %.2f mm), sorted:" % (FOLD_M, BLEED_MM_PER_CYCLE))
    print("    arm        run     cycles  entry   bled mm   spent/budget  verdict")
    scored = []
    for r, f in budget_rows:
        budget = (f["entry"] - FOLD_M) * 1000.0 / BLEED_MM_PER_CYCLE
        scored.append((f["cycles"] / budget if budget > 0 else float("inf"), r, f))
    for ratio, r, f in sorted(scored, reverse=True):
        print("    %-9s %-7s %6d  %.3f %8.0f %13.2f  %s" % (
            r["course"], r.get("run_id"), f["cycles"], f["entry"], f["bled"], ratio, r["verdict"]))
    surv = [x[0] for x in scored if x[1]["verdict"] == "PASS"]
    fell = [x[0] for x in scored if x[1]["verdict"] != "PASS"]
    if surv and fell:
        print("    survivors spent up to %.2fx; the lowest-spending faller was %.2fx" % (max(surv), min(fell)))
    elif surv:
        print("    every run here survived, spending up to %.2fx - consistent if all are below ~1.2x" % max(surv))

print("\n  FALLS, with whether the guard was even involved:")
for r, f in facts:
    if r["verdict"] == "PASS":
        continue
    if f is None:
        print("    %-9s run %-6s  (no archived log yet)" % (r["course"], r.get("run_id"))); continue
    tag = "guard FIRED (%d trips, %d cycles, bled %.0f mm)" % (f["trips"], f["cycles"], f["bled"]) \
          if f["trips"] else "guard NEVER FIRED - not attributable to the knob"
    print("    %-9s run %-6s  %s" % (r["course"], r.get("run_id"), tag))
    if f["orient"]:
        print("        %s" % f["orient"])
