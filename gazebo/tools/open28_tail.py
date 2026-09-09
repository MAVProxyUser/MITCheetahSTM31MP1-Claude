#!/usr/bin/env python3
"""Is OPEN-28 a distinct failure mode, or the tail of ordinary variation?

Ten hypotheses have now been eliminated, each with its own control: overspeed,
host stall, turning (correlational AND by direct corner sweep), gait changes,
any isolated corner 30-180 deg, long straights, chaining, recovery distance,
approach speed, and loss of support. Nothing about a run predicts whether it
falls. What is left is the possibility that there is nothing to predict.

The model that would explain that: every run draws a peak attitude excursion
from one smooth distribution, and a run falls exactly when its draw crosses
SafetyChecker's 28.65 deg. Under that model there is no trigger, no special
geometry and no precursor - only margin, and the fix is to shrink the
excursions rather than to find a cause.

The first version of this test assumed a run that passed never crossed the
limit, and fitted the passes as a sample truncated there. That is false: of
260 runs, 34 cross 28.65 deg and 20 of those RECOVER - the E-stop fires, the
recovery ladder stands the dog back up, and the run goes on to pass. So the
pass sample is not truncated and that fit was mis-specified.

Done properly the test is cleaner and not circular. Fit only the BODY of the
distribution - peaks below a cut well under the limit, with that cut's
truncation in the likelihood - then extrapolate to P(peak >= 28.65) and
compare against the observed crossing rate. The fit never sees the region it
is asked to predict, so agreement is a real out-of-sample result rather than
an artefact of the fitting window. Two cuts are reported: if the answer moves
with the cut, the extrapolation is not to be trusted.

Falling is then two stages, and both are measured separately:
    P(fall) = P(peak crosses the limit) x P(no recovery | crossed)

Usage: open28_tail.py [--limit 28.65]
"""
import csv, glob, os, math, argparse, statistics as st

ap = argparse.ArgumentParser()
ap.add_argument("--limit", type=float, default=28.65)
ap.add_argument("--campaigns", default="/Users/kfinisterre/Desktop/Cheetah/rundata/campaigns")
a = ap.parse_args()
L = a.limit


def load():
    """(family, peak, fell) for every run that recorded a peak attitude."""
    out = []
    for f in sorted(glob.glob(os.path.join(a.campaigns, "*.csv"))):
        try:
            rows = list(csv.DictReader(open(f)))
        except Exception:
            continue
        if not rows or "peak_pitch" not in rows[0]:
            continue
        seen = set()
        for r in rows:
            rid = r.get("run_id")
            if rid and rid in seen:          # stale ring: the run never started
                continue
            if rid:
                seen.add(rid)
            fam = r.get("course") or r.get("arm") or r.get("angle") or "?"
            if not r.get("peak_pitch"):
                continue
            try:
                pk = max(float(r["peak_pitch"]), float(r.get("peak_roll") or 0))
            except ValueError:
                continue
            if not (0 < pk < 180):
                continue
            fell = (r.get("fall") or "none") not in ("none", "")
            out.append((os.path.basename(f), fam, pk, fell))
    return out


def gumbel_fit_truncated(x, L, iters=400):
    """MLE for a Gumbel observed only below L.

    log f(x) = -(z + exp(-z)) - log(beta),  z = (x-mu)/beta
    log F(L)  = -exp(-(L-mu)/beta)
    Each observation contributes log f(x) - log F(L). Coarse grid then refine -
    the sample is small and the surface is smooth, so this beats hand-rolling
    a Newton step that can walk off a flat region.
    """
    def nll(mu, beta):
        if beta <= 1e-6:
            return 1e18
        s = 0.0
        for v in x:
            z = (v - mu) / beta
            s += z + math.exp(-z) + math.log(beta)
        s += len(x) * math.exp(-(L - mu) / beta)      # -n*log F(L)
        return s
    m0, s0 = st.mean(x), max(st.pstdev(x), 0.5)
    best = (m0, s0 * 0.7797)
    lo_mu, hi_mu = m0 - 3 * s0, m0 + 6 * s0
    lo_b, hi_b = 0.05 * s0, 4 * s0
    for _ in range(6):
        cand, bn = None, 1e18
        for i in range(41):
            mu = lo_mu + (hi_mu - lo_mu) * i / 40
            for j in range(41):
                be = lo_b + (hi_b - lo_b) * j / 40
                v = nll(mu, be)
                if v < bn:
                    bn, cand = v, (mu, be)
        best = cand
        dmu, db = (hi_mu - lo_mu) / 20, (hi_b - lo_b) / 20
        lo_mu, hi_mu = best[0] - dmu, best[0] + dmu
        lo_b, hi_b = max(1e-3, best[1] - db), best[1] + db
    return best


def gumbel_sf(x, mu, beta):
    return 1.0 - math.exp(-math.exp(-(x - mu) / beta))


def binom_ci(k, n):
    """Wilson 95%."""
    if n == 0:
        return (0.0, 1.0)
    p, z = k / n, 1.96
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


rows = load()
peaks = [r[2] for r in rows]
fell = [r[2] for r in rows if r[3]]
crossed = [p for p in peaks if p >= L]
crossed_fell = [r for r in rows if r[3] and r[2] >= L]
print(f"\n  {len(rows)} runs with a recorded peak attitude   (limit {L} deg)")
print(f"    crossed the limit : {len(crossed):>4}  ({len(crossed)/len(rows):.3f})")
print(f"    fell              : {len(fell):>4}  ({len(fell)/len(rows):.3f})")
print(f"    falls that crossed: {len(crossed_fell):>4} of {len(fell)}"
      f"   <- every fall crosses; nothing falls without crossing")
print(f"    recovered after crossing: {len(crossed)-len(fell):>4} of {len(crossed)}"
      f"  ({1-len(fell)/max(1,len(crossed)):.2f})")

print(f"\n  OUT-OF-SAMPLE TAIL TEST - fit the body, predict the crossing rate")
print(f"  {'fit cut':>8} {'n in body':>10} {'mu':>7} {'beta':>7} "
      f"{'predicted P(cross)':>19} {'observed':>10} {'95% CI':>16} {'':>8}")
lo, hi = binom_ci(len(crossed), len(rows))
for cut in (22.0, 25.0):
    body = [p for p in peaks if p < cut]
    if len(body) < 30:
        continue
    mu, beta = gumbel_fit_truncated(body, cut)
    pred = gumbel_sf(L, mu, beta)
    ok = lo <= pred <= hi
    print(f"  {cut:>8.1f} {len(body):>10} {mu:>7.2f} {beta:>7.2f} {pred:>19.3f} "
          f"{len(crossed)/len(rows):>10.3f} {('[%.3f, %.3f]' % (lo, hi)):>16} "
          f"{'CONSISTENT' if ok else 'NOT':>10}")

mu, beta = gumbel_fit_truncated([p for p in peaks if p < 25.0], 25.0)
print(f"\n  fit on peaks below 25 deg: mu={mu:.2f} beta={beta:.2f}. "
      f"How it does OUTSIDE that window:")
srt = sorted(peaks)
print(f"  {'threshold':>10} {'observed P(>=x)':>17} {'predicted':>11}")
for t in (25.0, 28.65, 32.0, 36.0, 40.0):
    obs = sum(1 for p in srt if p >= t) / len(srt)
    print(f"  {t:>10.2f} {obs:>17.3f} {gumbel_sf(t, mu, beta):>11.3f}")

pc = len(crossed) / len(rows)
pf = len(fell) / max(1, len(crossed))
print(f"\n  TWO STAGES, measured separately:")
print(f"    P(cross the limit)      = {pc:.3f}")
print(f"    P(fall | crossed)       = {pf:.3f}")
print(f"    product                 = {pc*pf:.3f}"
      f"   observed fall rate = {len(fell)/len(rows):.3f}")
print(f"\n  median run peaks at {st.median(peaks):.1f} deg against a {L} deg limit.")
