#!/bin/bash
# "How can I be sure the queue is running?" - answer it without asking me.
#
# Four independent checks, each of which can fail on its own, so a green
# line means something. The failure modes this session actually hit are all
# covered: a wedged conductor (alive, holding :8420, answering nothing), a
# queue script deadlocked in a sleep, a finished chain with nothing behind
# it, and a campaign grinding out cells with no verdicts.
cd "$(dirname "$0")/../../.."
say(){ printf "%-26s %s\n" "$1" "$2"; }

# 1. does the conductor ANSWER (not just exist)?
code=$(curl -s -o /tmp/.qs -w "%{http_code}" --max-time 6 \
        http://127.0.0.1:8420/api/state 2>/dev/null)
if [ "$code" = "200" ]; then say "conductor" "answering (HTTP 200)"
else say "conductor" "NOT ANSWERING (http=$code) <-- wedged or down"; fi

# 2. what does it say it is doing?
python3 - <<'PY' 2>/dev/null || echo "state              unreadable"
import json, time
s = json.load(open("/tmp/.qs"))
c = s.get("campaign") or {}
el = s.get("elapsed_s")
print("%-26s %s  run %s%s" % ("phase", s.get("phase"), s.get("run_id"),
      ("  (%ds into this run)" % int(el)) if el else ""))
if c.get("name"):
    age = time.time() - (c.get("updated") or 0)
    print("%-26s %s | %s%s" % ("campaign", c["name"], (c.get("stage") or "")[:44],
          "   <-- STALE (>5 min)" if age > 300 else ""))
else:
    print("%-26s none published" % "campaign")
PY

# 3. is a queue script actually alive?
# grep -v grep, and count with wc rather than grep -c: the grep process's
# OWN command line contains the pattern, so `grep -c` reports one more than
# is really running. Measured here: 6 vs the true 3. That is the same
# self-match family as the pgrep gate that deadlocked this queue for 40
# minutes, and it is worth being pedantic about in a tool whose entire job
# is answering "is anything actually running?"
# Match ANY /tmp/*.sh campaign script rather than a hand-maintained list:
# the list went stale the first time a new stage was queued (relief.sh and
# park.sh were both running while this reported "0 alive"), and a status
# tool that under-reports is as bad as one that over-reports.
n=$(ps -eo command | grep -E "^/bin/bash /tmp/[a-z0-9_]+\.sh" \
      | grep -v grep | wc -l | tr -d " ")
say "queue scripts alive" "$n"

# 4. is the RUN NUMBER advancing? the only check that cannot be faked by a
#    process merely existing. A cell takes 40-150 s, so give it 90.
a=$(python3 -c "import json;print(json.load(open('/tmp/.qs')).get('run_id'))" 2>/dev/null)
echo -n "run number advancing        watching 90s ... "
sleep 90
curl -s -o /tmp/.qs2 --max-time 6 http://127.0.0.1:8420/api/state 2>/dev/null
b=$(python3 -c "import json;print(json.load(open('/tmp/.qs2')).get('run_id'))" 2>/dev/null)
if [ "$a" != "$b" ]; then echo "YES ($a -> $b)"
else
  ph=$(python3 -c "import json;print(json.load(open('/tmp/.qs2')).get('phase'))" 2>/dev/null)
  if [ "$ph" = "running" ] || [ "$ph" = "launching" ]; then
    echo "still on run $a, phase=$ph (a long cell - normal)"
  else
    echo "NO - run $a, phase=$ph <-- nothing is being launched"
  fi
fi

# 5. the newest result lines, whichever campaign is live
echo; echo "last results:"
for f in /tmp/night2.log /tmp/w2pace.log /tmp/run_all.log; do
  [ -f "$f" ] && [ -n "$(find "$f" -mmin -10 2>/dev/null)" ] && \
    { echo "  ($f, modified in the last 10 min)"; \
      grep -E "^--- |VERDICT|METRICS|^####" "$f" | tail -6 | sed 's/^/  /'; break; }
done
