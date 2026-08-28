#!/usr/bin/env python3
"""Does host contention actually exist? Sweep CONCURRENCY at equal per-dog load.

WHY
---
This project has blamed "host contention" for multi-dog failures many
times, after the fact and qualitatively - and has been WRONG about it at
least three times in one night (pacing's "~50% coin-flip", walking's
"120-147.5 deg mid-band softness" and part of bounding's "bimodal" all
evaporated under solo re-testing; see CLAUDE.md's meta-lesson). Told
after the fact, it is unfalsifiable. This makes it a measurement.

THE DESIGN, and what makes it different from every past multi-dog batch
----------------------------------------------------------------------
Every previous batch here confounded two variables at once: how many dogs
were running AND how much work each was doing (different courses,
different speeds, different durations). So a failure could never be
attributed to concurrency rather than to the particular mission that
happened to be in the batch.

Here every dog runs the IDENTICAL mission at the IDENTICAL speed
(dash:100 @2.0 - a straight line, no cornering, no gait switching, so the
only thing that can differ between arms is how many are running). The
ONLY variable is N.

WHAT IS MEASURED
----------------
Not pass/fail - that is the crude end of the signal and this project has
already learned that a single fall proves little. The primary metric is
the CONTROL-LOOP PERIOD TAIL, which is what contention would physically
do: the loop targets 2.0 ms, and if the host cannot schedule it on time
the forces computed for a 2 ms step get applied for however long the tick
really took. CLAUDE.md's own threshold, established across many runs:
every failure sat at ~14% of intervals over 4 ms, every pass at <=3.9%.

So per dog we report max period seen, and how many of the once-a-second
health samples exceeded 4 ms. If contention is real, that tail should
grow monotonically with N. If it does not, contention is not what has
been failing these fleets and the past attributions were wrong.

USAGE
    contention_sweep.py            # N = 1,2,3, one rep each
    contention_sweep.py --reps 2   # two reps per arm
"""
import argparse
import json
import re
import subprocess
import sys
import time
import urllib.request

HERE = "/Users/kfinisterre/Desktop/Cheetah/Cheetah-Software"
API = "http://localhost:8420"
MISSION, GAIT, SPEED = "dash:100", "trotting", 2.0
OVER_MS = 4.0          # CLAUDE.md's own "unhealthy tick" threshold


def api(path):
    with urllib.request.urlopen(API + path, timeout=15) as r:
        return json.load(r)


def wait_idle():
    for _ in range(600):
        if api("/api/state").get("phase") not in ("running", "launching"):
            return True
        time.sleep(2)
    return False


def run_arm(n, rep):
    """Launch n identical dogs, wait, return per-dog loop stats."""
    wait_idle()
    cmd = [sys.executable, f"{HERE}/stm32mp1/gazebo/conductor/mission_runner.py"]
    for _ in range(n):
        cmd += ["--slot", MISSION, "--gait", GAIT, "--speed", str(SPEED), "--dash", "0"]
    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)
    elapsed = time.time() - t0
    out = p.stdout

    run_id = None
    m = re.search(r"launched run (\d+)", out)
    if m:
        run_id = int(m.group(1))

    dogs = []
    for i in range(n):
        try:
            txt = api(f"/api/logs/{i}?kind=ctrl&full=1")["text"]
        except Exception:
            dogs.append(dict(i=i, maxp=None, over=None, samples=0, runid=None))
            continue
        periods = [float(x) for x in re.findall(r"maxPeriod=([\d.]+) ms", txt)]
        ids = set(re.findall(r"\[RUNID\] run=(\d+)", txt))
        over = sum(1 for x in periods if x > OVER_MS)
        dogs.append(dict(
            i=i,
            maxp=max(periods) if periods else None,
            over=over,
            samples=len(periods),
            runid=sorted(ids),
        ))
    verdict = "PASS" if "VERDICT: PASS" in out else (
              "FELL" if "FELL" in out else "FAIL")
    return dict(n=n, rep=rep, run_id=run_id, wall=elapsed,
                verdict=verdict, dogs=dogs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--max-n", type=int, default=3)
    a = ap.parse_args()

    print(f"CONTENTION SWEEP - identical {MISSION} @ {SPEED} {GAIT} on every dog,")
    print("so the ONLY variable is how many run at once.\n")
    results = []
    for rep in range(1, a.reps + 1):
        for n in range(1, a.max_n + 1):
            r = run_arm(n, rep)
            results.append(r)
            tail = ", ".join(
                f"dog{d['i']}: max {d['maxp']:.2f}ms, {d['over']}/{d['samples']} over {OVER_MS}ms"
                if d["maxp"] is not None else f"dog{d['i']}: no data"
                for d in r["dogs"])
            print(f"N={n} rep{rep} run{r['run_id']}  {r['verdict']}  "
                  f"wall {r['wall']:.0f}s\n    {tail}", flush=True)

    print("\n" + "=" * 66)
    print("SUMMARY - worst loop period and over-threshold count, by concurrency")
    print("=" * 66)
    for n in range(1, a.max_n + 1):
        arms = [r for r in results if r["n"] == n]
        mx = [d["maxp"] for r in arms for d in r["dogs"] if d["maxp"] is not None]
        ov = [d["over"] for r in arms for d in r["dogs"] if d["over"] is not None]
        sm = sum(d["samples"] for r in arms for d in r["dogs"])
        vs = [r["verdict"] for r in arms]
        if not mx:
            print(f"N={n}: no data")
            continue
        print(f"N={n}  worst {max(mx):.2f} ms   over-{OVER_MS:.0f}ms "
              f"{sum(ov)}/{sm} samples   verdicts {vs}")
    print("\nIf contention is real, the tail grows with N. If it does not,")
    print("contention is not what has been failing these fleets.")
    json.dump(results, open("/tmp/contention_sweep.json", "w"), indent=1)
    print("raw -> /tmp/contention_sweep.json")


if __name__ == "__main__":
    main()
