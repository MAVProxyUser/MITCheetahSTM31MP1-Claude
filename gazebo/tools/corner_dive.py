import csv, glob, math, os, re, sys, statistics as st
sys.path.insert(0, 'gazebo/tools'); from snapio import load_json
D = '/Users/kfinisterre/Desktop/Cheetah/rundata'; A = D + '/conductor/archive'; DEG = 180 / math.pi
def rows(camp, arm=None, verdict=None):
    out = []
    for r in csv.DictReader(open(f'{D}/campaigns/{camp}.csv')):
        if arm and r['course'] != arm: continue
        if verdict and r['verdict'] != verdict: continue
        out.append(r)
    return out
def estop_second(lines):
    hb = None
    for l in lines:
        if 'ctrl loop:' in l:
            m = re.search(r'\[stm32mp1\] ctrl loop', l); hb_idx = l
        if 'Orientation safety check failed' in l or 'Unsafe locomotion' in l:
            return hb_val
        m = re.match(r'.*ctrl loop: maxRuntime', l)
        if m:
            pass
    return None
def dive(r):
    rid = r['run_id']; L = glob.glob(f'{A}/*run{rid}_ctrl_0.log'); S = r.get('snapshot')
    if not L or not S: return None
    lines = open(L[0], errors='replace').read().split('\n')
    # E-stop line index and the controller second from the heartbeat before it (the heartbeat carries t in the snapshot's text_log, not here) -> use the snapshot text_log instead
    d = load_json(S); tl = d.get('text_log', []); R = [x for x in d['records'] if x.get('t', 0) >= 2.0 and 'pitch' in x]
    hb = None; ev = None; navs = []
    for e in tl:
        m = str(e.get('msg', ''))
        t = e.get('t', 0.0)
        if 'ctrl loop:' in m and t and t > 0: hb = float(t)
        if m.startswith('[nav] wp'): navs.append(m)
        if ('Orientation safety check failed' in m or 'Unsafe locomotion' in m) and ev is None:
            ev = hb; break
    if ev is None: return None
    # find the event more precisely: first record after hb where |pitch| or |roll| crosses 28.65 deg (the safety bar)
    t_ev = None
    for x in R:
        if x['t'] >= ev and (abs(x['pitch']) * DEG >= 28.65 or abs(x['roll']) * DEG >= 28.65):
            t_ev = x['t']; break
    if t_ev is None: t_ev = ev + 0.5
    def at(dt):
        best = min(R, key=lambda x: abs(x['t'] - (t_ev - dt)))
        return best
    out = {'run': rid, 't_ev': t_ev}
    for dt in (1.0, 0.5, 0.2, 0.0):
        x = at(dt)
        out[dt] = (x['vx'], x['wz'], x['pitch'] * DEG, x['roll'] * DEG, x.get('z', 0))
    nav = navs[-2:] if navs else []
    out['nav'] = [re.sub(r'N=[^ ]+ E=[^ ]+ ', '', n)[:70] for n in nav]
    return out
def show(title, rs):
    print('==', title)
    agg = {dt: [] for dt in (1.0, 0.5, 0.2, 0.0)}
    for r in rs:
        o = dive(r)
        if not o: print('  run', r['run_id'], 'no data'); continue
        print('  run %s wp=%s | %s' % (o['run'], r['waypoints'], ' | '.join(o['nav'])))
        print('      ' + '   '.join('t-%.1f: vx %.2f wz %+.2f pitch %+5.1f roll %+5.1f' % (dt, *o[dt][:4]) for dt in (1.0, 0.5, 0.2, 0.0)))
        for dt in agg: agg[dt].append(o[dt])
    for dt in (1.0, 0.5, 0.2, 0.0):
        if agg[dt]:
            print('   median t-%.1f: vx %.2f wz %+.2f pitch %+.1f roll %+.1f' % (dt, st.median(a[0] for a in agg[dt]), st.median(a[1] for a in agg[dt]), st.median(a[2] for a in agg[dt]), st.median(a[3] for a in agg[dt])))
show('wkc_finals 2.6 falls at wp2 (chain Y + AA, budgets 0.4/0.3/0.2)', [r for r in rows('wkc_lead1_top', 'v26') + rows('wkc26_alon') if r['verdict'] != 'PASS' and r['waypoints'] in ('2', '3')])
show('wkc_finals 2.6 falls at wp7 (box third corner)', [r for r in rows('wkc26_alon') if r['verdict'] != 'PASS' and r['waypoints'] == '7'][:6])
show('wkc_finals 2.6 falls at wp9 (reversal approach)', [r for r in rows('wkc_lead1_top', 'v26') + rows('wkc26_alon') if r['verdict'] != 'PASS' and r['waypoints'] == '9'])
show('hairpin 2.5 first-corner falls (wp1)', [r for r in rows('hp_ship_n10', 'v25') + rows('hp_ship_n10b', 'v25') + rows('hp_ship26', 'v25') if r['verdict'] != 'PASS'])
