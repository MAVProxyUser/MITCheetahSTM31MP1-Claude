#!/usr/bin/env python3
"""
Ring-buffer archive of test_validated_missions.py runs - the "tome" of
historic success/failure the suite builds up over time, so a future agent
(or the operator) can ask "has this case been flaky lately" or "what did
the last three atom failures actually look like" without re-running
anything or re-deriving it from CLAUDE.md prose.

Same ring-buffer shape this project already uses elsewhere (TRAIL_MAX in
the conductor, server.py's own `self.log[-60:]` orchestration window) -
capped size, oldest evicted, so this file cannot grow without bound across
months of iteration. One JSON array on disk, read-modify-write; this is a
single-operator local tool run at most a few times a minute, not a
service under concurrent write load, so no locking is implemented.
"""
import json
import re
import time
from pathlib import Path

HISTORY_PATH = Path(__file__).resolve().parent / "history.json"
MAX_ENTRIES = 1000  # ring buffer cap - oldest entries evicted past this


def _load():
    if not HISTORY_PATH.exists():
        return []
    try:
        return json.loads(HISTORY_PATH.read_text())
    except (json.JSONDecodeError, OSError):
        return []  # corrupt/partial file - start fresh rather than crash the suite


def _save(entries):
    entries = entries[-MAX_ENTRIES:]
    HISTORY_PATH.write_text(json.dumps(entries, indent=1))


def extract_failure_signature(stdout: str) -> str:
    """Pull the same kind of detail CLAUDE.md has always captured by hand
    for a failure (roll/pitch/z, or which waypoint) out of raw runner
    output, so the ring buffer entry is diagnostic, not just a verdict."""
    fall = re.findall(r"\[FALL\][^\n]*", stdout)
    if fall:
        return fall[-1]
    safety = re.findall(r"SAFETY CHECK FAILED[^\n]*", stdout)
    if safety:
        return safety[-1]
    stall = re.findall(r"NO PROGRESS[^\n]*|STALLED[^\n]*", stdout)
    if stall:
        return stall[-1]
    return ""


def extract_shm_trace_paths(stdout: str) -> list:
    """Every failing dog's own per-tick forensic trace (roll/pitch/z/velocity
    at every 2ms tick leading up to the fall - tens of MB, tens of thousands
    of records) is already archived by the conductor itself whenever a dog
    falls ("dog%d: shm trace archived -> <path>", see server.py). Reference
    those paths from the lightweight history entry instead of duplicating
    that data here - this file stays small and scannable (a real ring
    buffer, not an ever-growing forensic dump); the full tick-by-tick record
    for any specific historical failure is one path away when actually
    needed."""
    return re.findall(r"shm trace archived -> (\S+)", stdout)


def record(case_name: str, config: dict, returncode: int, wall_s: float, stdout: str):
    verdict = {0: "PASS", 1: "FAIL", 2: "INCONCLUSIVE"}.get(returncode, f"exit={returncode}")
    entry = {
        "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
        "case": case_name,
        "config": config,
        "verdict": verdict,
        "wall_s": round(wall_s, 1),
    }
    if verdict != "PASS":
        sig = extract_failure_signature(stdout)
        if sig:
            entry["failure_signature"] = sig
        traces = extract_shm_trace_paths(stdout)
        if traces:
            entry["shm_trace_paths"] = traces
    entries = _load()
    entries.append(entry)
    _save(entries)
    return entry


def summarize(case_name: str = None, last_n: int = 20):
    entries = _load()
    if case_name:
        entries = [e for e in entries if e["case"] == case_name]
    if not entries:
        print(f"No history yet{f' for {case_name}' if case_name else ''}.")
        return
    by_case = {}
    for e in entries:
        by_case.setdefault(e["case"], []).append(e)
    for name, runs in by_case.items():
        passes = sum(1 for r in runs if r["verdict"] == "PASS")
        print(f"\n{name}: {passes}/{len(runs)} PASS all-time")
        for e in runs[-last_n:]:
            line = f"  {e['ts']}  {e['verdict']:12s} {e['wall_s']:6.1f}s"
            if e.get("failure_signature"):
                line += f"   {e['failure_signature']}"
            print(line)
            for p in e.get("shm_trace_paths", []):
                print(f"      full tick-by-tick trace: {p}")


if __name__ == "__main__":
    import sys
    summarize(sys.argv[1] if len(sys.argv) > 1 else None)
