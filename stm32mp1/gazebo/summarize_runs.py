#!/usr/bin/env python3
"""Consolidate every sweep result file into the two tables that matter:
max sustained speed per gait, and star-mission time per gait.

  summarize_runs.py <results-file> [more...]

Reads the fixed-width rows host_sweep.sh emits. Speeds come from the run LABEL
(..._v06 -> 0.6), so a label must carry its speed to be scored.
"""
import glob
import re
import sys
from collections import defaultdict

ROW = re.compile(
    r'^(\S+)\s+([-\d.]+)\s+(\d+)s\s+([\d.]+)m\s+([-\d.]+)\s+(.*?)\s*$')

GAIT_ORDER = ['walk', 'walk2', 'trot', 'pronk', 'bound', 'gallop', 'trotrun', 'pace']
PRETTY = {'walk': 'walking (20)', 'walk2': 'walking2 (21)', 'trot': 'trotting (9)',
          'pronk': 'pronking (2)', 'bound': 'bounding (1)', 'gallop': 'galloping (22)',
          'trotrun': 'trotRunning (5)', 'pace': 'pacing (8)'}


def parse_label(label):
    """-> (gait, speed, is_star, tag) ; speed None when the label carries none."""
    is_star = label.startswith('star_')
    body = label[5:] if is_star else label
    m = re.match(r'^(?:sync_|async_)?([a-z0-9]+?)_v(\d+)(.*)$', body)
    if not m:
        return None, None, is_star, ''
    gait, digits, tag = m.group(1), m.group(2), m.group(3)
    speed = float(digits[0] + '.' + digits[1:]) if len(digits) >= 2 else float(digits)
    return gait, speed, is_star, tag


def cruise_speed(label):
    """True cruising speed, from the saved pose trace.

    Distance/total-time badly understates it: every run spends ~18 s standing,
    entering BALANCE_STAND, engaging the gait and ramping velocity before it
    travels at all, so a gait holding 1.0 m/s scores ~0.5. Measure over the
    LAST 15 s of the trace instead, which is pure cruise.
    """
    hits = glob.glob(f'/tmp/host_sweep_*/{label}.trace')
    if not hits:
        return None
    rows = []
    for ln in open(max(hits), errors='ignore'):
        m = re.search(r't=\s*(\S+)s E=\s*(\S+) N=\s*(\S+) z=(\S+) dist=\s*(\S+)m', ln)
        if m:
            t, e, n, z, d = (float(x) for x in m.groups())
            rows.append((t, e, n, z, d))
    if len(rows) < 4:
        return None
    # only samples where the robot is actually up
    up = [r for r in rows if r[3] > 0.15]
    if len(up) < 4:
        return None
    end = up[-1]
    window = [r for r in up if r[0] >= end[0] - 15.0]
    if len(window) < 2:
        return None
    start = window[0]
    dt = end[0] - start[0]
    if dt <= 0:
        return None
    ds = ((end[1] - start[1]) ** 2 + (end[2] - start[2]) ** 2) ** 0.5
    return ds / dt


def main():
    runs = []
    for path in sys.argv[1:]:
        try:
            fh = open(path, errors='ignore')
        except OSError:
            continue
        for ln in fh:
            if ln.startswith('LABEL') or set(ln.strip()) <= {'-'}:
                continue
            m = ROW.match(ln.rstrip())
            if not m:
                continue
            label, dist, up, _loop, drift, verdict = m.groups()
            gait, speed, is_star, tag = parse_label(label)
            if gait is None:
                continue
            runs.append(dict(label=label, gait=gait, speed=speed, is_star=is_star,
                             tag=tag, dist=float(dist), up=int(up),
                             drift=float(drift), verdict=verdict,
                             ok='UPRIGHT' in verdict or 'STAR DONE' in verdict))

    # ---- max sustained speed -------------------------------------------------
    best = defaultdict(lambda: {'ok': [], 'bad': []})
    for r in runs:
        if r['is_star'] or r['tag'] or r['label'].startswith(('sync_', 'async_')):
            continue
        best[r['gait']]['ok' if r['ok'] else 'bad'].append(r)

    print('\n### Max sustained speed per gait (Mac SITL, cheater state, 26 ms MPC segment)\n')
    print('| gait | max held | distance | cruise speed | drift | fastest failure |')
    print('|---|---|---|---|---|---|')
    for g in GAIT_ORDER:
        if g not in best:
            continue
        ok, bad = best[g]['ok'], best[g]['bad']
        slowest_fail = min((r['speed'] for r in bad), default=None)
        if ok:
            top = max(ok, key=lambda r: r['speed'])
            cruise = cruise_speed(top['label'])
            cruise_s = f'{cruise:.2f} m/s' if cruise else 'n/a'
            fail = f'{slowest_fail:.1f} m/s' if slowest_fail else 'none tested'
            print(f'| {PRETTY.get(g, g)} | **{top["speed"]:.1f} m/s** | {top["dist"]:.2f} m '
                  f'| {cruise_s} | {abs(top["drift"]):.2f} m | {fail} |')
        else:
            worst = min(bad, key=lambda r: r['speed'])
            print(f'| {PRETTY.get(g, g)} | none held | {worst["dist"]:.2f} m before falling '
                  f'| - | - | {worst["speed"]:.1f} m/s |')

    # ---- star missions -------------------------------------------------------
    stars = [r for r in runs if r['is_star']]
    if stars:
        print('\n### Star mission: 5 legs x 10.1 m (33 ft), wp00 due north\n')
        print('| gait | commanded | result | mission time | avg over ground |')
        print('|---|---|---|---|---|')
        PERIM = 50.5
        for r in sorted(stars, key=lambda r: (r['gait'], -(r['speed'] or 0))):
            t = re.search(r'STAR DONE ([\d.]+)s', r['verdict'])
            if t:
                secs = float(t.group(1))
                print(f'| {PRETTY.get(r["gait"], r["gait"])} | {r["speed"]:.1f} m/s | '
                      f'COMPLETE 5/5 | **{secs:.0f} s** | {PERIM/secs:.2f} m/s |')
            else:
                print(f'| {PRETTY.get(r["gait"], r["gait"])} | {r["speed"]:.1f} m/s | '
                      f'{r["verdict"]} | - | - |')

    # ---- async A/B -----------------------------------------------------------
    ab = [r for r in runs if r['label'].startswith(('sync_', 'async_'))]
    if ab:
        print('\n### Async vs inline solve (the board can only run async at 26 ms)\n')
        print('| run | distance | upright | verdict |')
        print('|---|---|---|---|')
        for r in ab:
            print(f'| {r["label"]} | {r["dist"]:.2f} m | {r["up"]}s | {r["verdict"]} |')


if __name__ == '__main__':
    main()
