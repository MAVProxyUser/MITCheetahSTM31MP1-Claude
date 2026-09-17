#!/bin/bash
# host_tenants.sh - list the host's real CPU tenants by SUSTAINED load.
#
# WHY THIS EXISTS (2026-09-17, ISSUES OPEN-35 decisions #5 and #7).
# `ps -o pcpu` is a DECAYED INSTANTANEOUS estimate, not sustained load, and every
# host-hygiene check in this tree used to read it. Two failures came from that in
# one hour:
#   - ANECompilerService sampled at 97.8 % pcpu and looked like an 8-day runaway.
#     It had used 7 min 57 s of CPU in 8 d 9 h = 0.04 %. Nothing to chase.
#   - interceptor-sim sampled 42.4 / 63.3 / 25.0 %, so I called it "spiky" and
#     RETRACTED A CORRECT CONCLUSION about data contamination. CPU time over
#     elapsed time said 36.0 % steady (5.25 CPU-hours) and the original reasoning
#     was right all along.
# The measure that answers "how much has this consumed" is CPU time / elapsed
# time. That is what this prints.
#
# It also finds what instantaneous load cannot: a WEDGED process. corespotlightd
# pid 80690 held 928 min of CPU over 6 d 9 h (10.1 % sustained, 15.5 CPU-HOURS)
# while its two sibling corespotlightd instances had used 0.75 s over 23 days and
# 9.07 s over 4 days. One instance burning CPU-hours while its siblings idle is a
# stuck process, and it had been there through every campaign since 09-11. That is
# the second multi-CPU-day straggler here, after a `gz topic -e -n 1` that burned
# 5.8 CPU-days over 12 days.
#
# REPORT ONLY by default. CULL=1 kills ONLY the names the operator has authorized.
#
# usage:  bash gazebo/tools/host_tenants.sh [TOPN]
#         CULL=1 bash gazebo/tools/host_tenants.sh
#         MIN_SUST=5 MIN_CPUH=1 bash gazebo/tools/host_tenants.sh
set -u
TOPN="${1:-12}"
MIN_SUST="${MIN_SUST:-5}"      # flag at/above this sustained %
MIN_RATIO="${MIN_RATIO:-1}"     # ...and CPU-hours only counts above this sustained %
MIN_CPUH="${MIN_CPUH:-1}"      # or at/above this many CPU-hours consumed
CULL="${CULL:-0}"

# The operator authorized culling exactly these. Everything else is reported and
# left alone - the user's own processes are not mine to kill. Explicitly NOT here:
# photoanalysisd, duetexpertd, photolibraryd, NinjaPilotGCS, NinjaPilot,
# BambuStudio, ESP-IDF builds, and any second simulator the operator is running.
AUTHORIZED='corespotlightd|mediaanalysisd|mds_stores|mds$|mdworker_shared'

# The rig's OWN processes. A campaign calling this at its start would otherwise
# flag its own simulator as the biggest tenant on the box - a live gz sim sits
# near 60 % sustained BY DESIGN. Matched against the full argv, not the basename.
# Several of these run with a bare script name from a cwd, so the repo path is
# NOT in their argv - the bridge is launched as `Python -u cheetah_gazebo_bridge.py`
# and the conductor as `Python server.py`. Match the script names too.
RIG='rundata/conductor|gazebo/conductor|gazebo/tools/(open28|campaign|wkc)|mit_ctrl_sim'
RIG="$RIG"'|cheetah_gazebo_bridge|server[.]py|unittests/test_validated|fleet[.]sdf'

ps -Axo pid,ppid,time,etime,pcpu,args | TOPN="$TOPN" MIN_SUST="$MIN_SUST" \
  MIN_CPUH="$MIN_CPUH" MIN_RATIO="$MIN_RATIO" CULL="$CULL" AUTH="$AUTHORIZED" RIG="$RIG" python3 -c '
import sys, os, re
def secs(t):
    t = t.strip(); d = 0
    if "-" in t: d, t = t.split("-"); d = int(d)
    p = [float(x) for x in t.split(":")]
    while len(p) < 3: p.insert(0, 0.0)
    return d*86400 + p[0]*3600 + p[1]*60 + p[2]
topn     = int(os.environ["TOPN"])
min_sust = float(os.environ["MIN_SUST"])
min_cpuh = float(os.environ["MIN_CPUH"])
min_ratio = float(os.environ["MIN_RATIO"])
cull     = os.environ["CULL"] == "1"
auth     = re.compile(os.environ["AUTH"])
rig      = re.compile(os.environ["RIG"])
rows = []
for ln in sys.stdin.read().split("\n")[1:]:
    m = re.match(r"\s*(\d+)\s+(\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(.*)$", ln)
    if not m: continue
    pid, ppid, cput, et, pc, args = m.groups()
    try: c, e = secs(cput), secs(et)
    except Exception: continue
    if e < 60: continue                      # too young to have a ratio
    rows.append((c/e*100.0, c, e, int(pid), int(ppid), pc, args))
rows.sort(reverse=True)
print("  [tenants] SUSTAINED = CPU time / elapsed. pcpu shown only to contrast.")
print("  [tenants] %7s %8s %10s %9s  %s" % ("sust%","pcpu","CPU-time","elapsed","process"))
for sust, c, e, pid, ppid, pc, args in rows[:topn]:
    print("  [tenants] %6.1f%% %7s%% %8.2f h %7.1f h  %s" % (sust, pc, c/3600.0, e/3600.0, args[:66]))
# CPU-hours alone is not a tenant: a daemon alive 570 h reaches 1 CPU-hour at
# 0.2 % sustained. Both criteria therefore require real sustained load.
flag = [r for r in rows if r[0] >= min_sust or (r[1]/3600.0 >= min_cpuh and r[0] >= min_ratio)]
print("  [tenants] %d process(es) at/above %.0f%% sustained, or %.0f CPU-hour(s) while still above %.0f%%:" % (len(flag), min_sust, min_cpuh, min_ratio))
# A wedged instance shows up as one busy process among idle siblings of the same
# name - that comparison is why this groups by basename.
byname = {}
for r in rows:
    byname.setdefault(r[6].split()[0].split("/")[-1] if r[6].split() else "?", []).append(r)
for sust, c, e, pid, ppid, pc, args in flag:
    name = args.split()[0].split("/")[-1] if args.split() else "?"
    sibs = [x for x in byname.get(name, []) if x[3] != pid]
    note = ""
    if sibs:
        worst = max(x[1] for x in sibs)
        if sust >= min_sust and c > 600 and worst < c/20.0:
            note = "  <- WEDGED? %d sibling(s) of this name used <= %.1f s" % (len(sibs), worst)
    mine = rig.search(args) is not None
    ok   = auth.search(name) is not None and not mine
    if mine:   tag = "RIG (expected, not a tenant)"
    elif ok:   tag = "AUTHORIZED-to-cull"
    else:      tag = "not mine to kill"
    print("  [tenants]   pid %-6d %5.1f%% sustained  %6.2f CPU-h  %-28s %s%s" % (
        pid, sust, c/3600.0, name[:28], tag, note))
    if cull and ok:
        try:
            os.kill(pid, 9); print("  [tenants]   -> killed %d (%s), it had held %.2f CPU-hours" % (pid, name, c/3600.0))
        except Exception as ex:
            print("  [tenants]   -> could not kill %d: %s" % (pid, ex))
if not cull:
    print("  [tenants] report only. CULL=1 kills the AUTHORIZED ones and nothing else.")
'
