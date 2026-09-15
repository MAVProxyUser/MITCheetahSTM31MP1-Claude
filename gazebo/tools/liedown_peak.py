#!/usr/bin/env python3
"""Score the lie-down's SECOND stage per run: the peak |roll| and peak |roll
rate| after the damping hand-off (ISSUES OPEN-30/27, 2026-09-14).

Every lie-down on record ends the same way - STAND_UP holds the body at
~0.10 m (target 0.15) for 2.5 s, then the legs are handed to a pure damper
and the body drops onto its folded shanks. The roll starts at that hand-off
in every snapshot examined, 10-20 deg in ordinary runs and past 90 in the
finish tips, so the hand-off's peak is a continuous score of the tip
mechanism that every run provides (the tip itself is a 1-3 % tail).

  usage: liedown_peak.py label=CAMPAIGN_GLOB[:arm] ...
  e.g.   liedown_peak.py recipe=wkc26_liedown:recipe kd24=wkc26_liedown:kd24

Reads the campaign CSVs' snapshot column through snapio (.json or .json.zst).
The hold is found as the last stretch of >= 1.0 s with z within 0.10 +- 0.02
and |roll| < 5 deg; the window is from its end to 1.5 s later (or the record
end). Rows without a hold (a fall before the finish) are reported separately.
"""
import sys, os, csv, glob, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from snapio import load_json
DEG = 57.2958
DATA = os.environ.get("CHEETAH_DATA", os.path.expanduser("~/Desktop/Cheetah/rundata"))

def hold_end(R):
    """index of the damping hand-off: the last record of a flat hold (>= 1.0 s
    with z within 0.006 m of itself, inside 0.08..0.12, |roll| < 5 deg) that is
    FOLLOWED by a descent (z lower by >= 0.004 m half a second later). The
    belly rest at the end of the record can be flat inside the same band, so
    "flat" alone is not enough - the hold is the flat stretch the drop follows."""
    n = len(R)
    if n < 800: return None
    def flat(i):
        w = [x['z'] for x in R[i - 500:i + 1]]
        return (max(w) - min(w) < 0.006 and 0.08 <= R[i]['z'] <= 0.12 and abs(R[i]['roll']) * DEG < 5)
    i = n - 260
    while i > 500:
        if flat(i) and (R[i]['z'] - R[i + 250]['z']) >= 0.004:
            # walk forward to the last flat record of this hold (the hand-off itself)
            j = i
            while j + 1 < n and flat(j + 1): j += 1
            return j
        i -= 1
    return None

def held_end(R):
    """An arm that never hands off (WP_LIEDOWN_EDAMP=0 keeps STAND_UP's own PD
    through the second stage) has no descent to key on: the body sits in the
    hold until the judge and the process exits. Score it over the LAST 1.5 s of
    a flat hold that runs to the end of the record (within 0.5 s) - the same
    instants the stock arm's judge reads. Returns the index of the hold's last
    record, or None (a run that fell before the finish has no such hold)."""
    n = len(R)
    if n < 800: return None
    j = n - 1
    while j > n - 260 and not (0.08 <= R[j]['z'] <= 0.12 and abs(R[j]['roll']) * DEG < 5): j -= 1
    if j <= n - 260: return None
    w = [x['z'] for x in R[j - 500:j + 1]]
    return j if (max(w) - min(w) < 0.006) else None

# A score cache keyed by the snapshot path (the CSV's .json path - snapio resolves
# a later .zst, so the key is stable): the pooled scans grew to ~140 snapshots a
# block and were holding the rig idle for minutes in every gap (2026-09-15 08:23).
# A snapshot never changes once written, so its score never does.
_CACHE_PATH = os.path.join(DATA, "campaigns", "liedown_score_cache.json")
_cache = None
_cache_dirty = False

def _cache_load():
    global _cache
    if _cache is None:
        try:
            import json; _cache = json.load(open(_CACHE_PATH))
        except Exception:
            _cache = {}
    return _cache

def _cache_save():
    if _cache_dirty and _cache is not None:
        import json, tempfile
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(_CACHE_PATH), prefix=".liedown_cache."); os.close(fd)
        with open(tmp, "w") as f: json.dump(_cache, f)
        os.replace(tmp, _CACHE_PATH)

import atexit; atexit.register(_cache_save)

def score(snap):
    c = _cache_load()
    if snap in c: return c[snap]
    r = _score_uncached(snap)
    c[snap] = r
    global _cache_dirty; _cache_dirty = True
    return r

def _score_uncached(snap):
    d = load_json(snap); R = [x for x in d['records'] if isinstance(x, dict) and 'roll' in x and 'z' in x]
    if not R: return None
    k = hold_end(R); mode = "handoff"
    if k is None:
        k = held_end(R); mode = "held"
        if k is None: return None
        t0 = R[k]['t'] - 1.5; W = [x for x in R if t0 <= x['t'] <= R[k]['t']]
    else:
        t0 = R[k]['t']; W = [x for x in R if t0 <= x['t'] <= t0 + 1.5]
    if len(W) < 10: return None
    return dict(peak_roll=max(abs(x['roll']) for x in W) * DEG,
                peak_wx=max(abs(x.get('wx', 0.0)) for x in W),
                end_roll=abs(W[-1]['roll']) * DEG, t0=t0, mode=mode)

def main():
    for spec in sys.argv[1:]:
        label, sel = spec.split("=", 1)
        pat, _, arm = sel.partition(":")
        rolls, wxs, ends, nohold, held = [], [], [], 0, 0
        for f in sorted(glob.glob(os.path.join(DATA, "campaigns", pat + ".csv"))):
            for r in csv.DictReader(open(f)):
                if arm and r.get("course") != arm: continue
                if r.get("verdict") == "NONE" or not r.get("snapshot"): continue
                try: s = score(r["snapshot"])
                except Exception: s = None
                if s is None: nohold += 1; continue
                rolls.append(s["peak_roll"]); wxs.append(s["peak_wx"]); ends.append(s["end_roll"]); held += (s["mode"] == "held")
        if not rolls:
            print("%-14s no scored lie-downs (%d without a hold)" % (label, nohold)); continue
        rolls.sort()
        p90 = rolls[min(len(rolls) - 1, int(0.9 * len(rolls)))]
        print("%-14s n=%3d  stage-2 peak roll median %5.1f  p90 %5.1f  max %5.1f | peak roll rate median %4.2f rad/s | roll at the judge instant median %4.1f | %d without a hold%s"
              % (label, len(rolls), st.median(rolls), p90, rolls[-1], st.median(wxs), st.median(ends), nohold, (" | %d held through (no hand-off)" % held) if held else ""))

if __name__ == "__main__":
    main()
