#!/bin/bash
# status.sh - one screen answering "is the rig working right now?"
# Counts with `grep -v grep` + `wc -l`, never `grep -c`: the grep's own
# command line contains the pattern, which measured 6 against 3 real.
set -u
BASE="http://127.0.0.1:8420"
echo "== conductor =="
if curl -s -o /tmp/.st.json -m 8 -w "  http %{http_code}  ttfb %{time_starttransfer}s\n" "$BASE/api/state" 2>/dev/null; then
  python3 - <<'PY'
import json
try: d=json.load(open("/tmp/.st.json"))
except Exception as e: print("  unparseable:",e); raise SystemExit
c=d.get("campaign") or {}
print("  phase   %s   run %s" % (d.get("phase"), d.get("run_id")))
print("  campaign%s" % ("  %s | %s" % (c.get("name"), c.get("stage")) if c.get("name") else "  none"))
h=d.get("health") or {}
flag = "  <-- LEAK" if h.get("leaking") else ""
print("  threads %s (baseline %s, drift %s, children %s, viewers %s)%s" % (
    h.get("threads"), h.get("baseline"), h.get("drift"), h.get("children"),
    h.get("mjpeg_viewers"), flag))
l=d.get("host_load") or {}
print("  host    cpu %s%%  gpu %s%%" % (l.get("cpu_pct"), l.get("gpu_pct")))
cams=d.get("cameras") or {}
print("  cameras %s" % (json.dumps(cams) if cams else "none streaming"))
PY
else
  echo "  NOT ANSWERING"
fi
echo
echo "== campaign scripts (queued work) =="
Q=$(ps -eo command | grep -E "^(/bin/)?bash /tmp/[a-z0-9_]+\.sh" | grep -v grep | sed 's|^.*/tmp/||' | sort -u | tr '\n' ' ')
echo "  ${Q:-NONE - the rig has nothing queued}"
echo
echo "== processes =="
for p in 'gz[ ]sim' 'mit_ctrl[_]sim' 'cheetah[_]gazebo[_]bridge' '[s]erver.py' '[c]am_feed.py' '[p]ose_feed.py'; do
  printf '  %-28s %s\n' "$(echo "$p" | tr -d '[]')" "$(pgrep -f "$p" 2>/dev/null | wc -l | tr -d ' ')"
done
echo
echo "== is work actually happening? =="
A=$(curl -s -m 6 "$BASE/api/state" 2>/dev/null | python3 -c "import json,sys;print(json.load(sys.stdin).get('run_id'))" 2>/dev/null)
sleep 45
B=$(curl -s -m 6 "$BASE/api/state" 2>/dev/null | python3 -c "import json,sys;print(json.load(sys.stdin).get('run_id'))" 2>/dev/null)
if [ "${A:-x}" != "${B:-y}" ]; then echo "  YES - run $A -> $B in 45s"
else echo "  run number did not advance in 45s (run $A) - idle, or one long mission"; fi
