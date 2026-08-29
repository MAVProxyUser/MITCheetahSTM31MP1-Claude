#!/usr/bin/env python3
"""campaign - let a long-running harness tell the PANEL what it is doing.

Operator, twice: "the orchestrator is literally sitting there doing
nothing". It was not - it was 40 minutes into a 42-cell sweep - but from
the panel there is no way to know that. The conductor only ever knew about
ONE run at a time, so a campaign of eighty runs looked like eighty
unrelated launches with quiet gaps between them, and a cell that takes
150 s (a fall plus teardown plus trace archiving) looks exactly like a
stall.

This is the missing channel: a harness writes one small JSON file, the
server reads it into /api/state, and the panel shows "OPEN-8 wide angle
grid - cell 12/42" with the elapsed time. No coupling beyond the file, so
a harness that crashes just leaves a stale record rather than wedging
anything, and `clear()` on exit removes it.
"""
import json
import os
import time

RUN_DIR = "/tmp/cheetah_conductor"
PATH = os.path.join(RUN_DIR, "campaign.json")


def set_stage(name, stage="", done=0, total=0, note=""):
    """Publish where a campaign is. Cheap enough to call per cell."""
    try:
        os.makedirs(RUN_DIR, exist_ok=True)
        prev = read() or {}
        rec = dict(name=name, stage=stage, done=done, total=total, note=note,
                   updated=time.time(),
                   started=prev.get("started") if prev.get("name") == name
                            else time.time())
        with open(PATH, "w") as f:
            json.dump(rec, f)
    except Exception:  # noqa: BLE001 - telemetry must never break a sweep
        pass


def read():
    try:
        with open(PATH) as f:
            return json.load(f)
    except Exception:  # noqa: BLE001
        return None


def clear():
    try:
        os.remove(PATH)
    except OSError:
        pass


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "clear":
        clear()
    else:
        # campaign.py "<name>" "<stage>" <done> <total> ["note"]
        a = sys.argv[1:]
        set_stage(a[0] if a else "campaign",
                  a[1] if len(a) > 1 else "",
                  int(a[2]) if len(a) > 2 else 0,
                  int(a[3]) if len(a) > 3 else 0,
                  a[4] if len(a) > 4 else "")
