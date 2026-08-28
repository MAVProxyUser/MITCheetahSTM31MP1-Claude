#!/usr/bin/env python3
"""conductor_ctl - restart the conductor server the SAFE way, for harnesses.

OPEN-21: the server's in-process gz-transport pose feed decays with
accumulated launches (measured dying after ~6-25 launches, accelerating),
and the in-process self-heal is only transient. Until the pose
subscription moves to a per-run subprocess, long campaigns must recycle
the server whenever a NOFEED verdict appears. This is the one sanctioned
way to do that: /api/stop FIRST (routes through _reap_and_confirm so no
child is orphaned - the documented unsafe-restart contamination class),
then kill, relaunch under the same venv python, and wait for idle.
"""
import os
import subprocess
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
PYBIN = ("/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/"
         "ground/gazebo_bridge/venv/bin/python3")
BASE = "http://127.0.0.1:8420"


def _state():
    try:
        with urllib.request.urlopen(BASE + "/api/state", timeout=5) as r:
            import json
            return json.loads(r.read())
    except Exception:  # noqa: BLE001
        return None


def restart_server(reason=""):
    print("[conductor_ctl] restarting server%s"
          % ((" - " + reason) if reason else ""), flush=True)
    try:
        urllib.request.urlopen(
            urllib.request.Request(BASE + "/api/stop", data=b"{}",
                                    method="POST"), timeout=15).read()
    except Exception:  # noqa: BLE001 - already down is fine
        pass
    time.sleep(2)
    subprocess.run("kill $(lsof -ti :8420) 2>/dev/null", shell=True)
    time.sleep(2)
    subprocess.Popen([PYBIN, "server.py"], cwd=HERE,
                     stdout=open("/tmp/conductor_server.log", "a"),
                     stderr=subprocess.STDOUT,
                     start_new_session=True)
    for _ in range(20):
        time.sleep(1)
        s = _state()
        if s and s.get("phase") == "idle":
            print("[conductor_ctl] server back, idle", flush=True)
            return True
    print("[conductor_ctl] server did NOT come back idle in 20s", flush=True)
    return False
