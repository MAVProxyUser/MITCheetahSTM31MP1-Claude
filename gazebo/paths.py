"""Where this project's run data lives. One definition, imported everywhere.

WHY THIS EXISTS. Operator, 2026-09-04: "stop working out of /tmp for fuck
sake!" - after `/tmp/go1_raw.sdf` went missing and I had to work around it.
They were right, and the exposure was much worse than that one file: the
conductor's whole run tree, including 5.8 GB of shm_trace fall snapshots that
every OPEN-26 number is derived from, was sitting in a directory macOS is free
to purge at any reboot or under disk pressure. Losing it would have turned a
night of measurements into unreproducible claims. It had already bitten twice -
the raw SDF was gone when I went to regenerate the world, and I had to write
fall_index.py specifically to distil findings out of files I could not trust to
survive.

DATA_ROOT defaults to a sibling of the repo, NOT a subdirectory of it: the
archive is multi-gigabyte and has no business anywhere git might see it.
Override with CHEETAH_DATA if you want it elsewhere.
"""
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, ".."))

DATA_ROOT = os.environ.get(
    "CHEETAH_DATA", os.path.abspath(os.path.join(_REPO, "..", "rundata")))

RUN_DIR      = os.path.join(DATA_ROOT, "conductor")
ARCHIVE_DIR  = os.path.join(RUN_DIR, "archive", "shm_trace")
REPORTS_DIR  = os.path.join(RUN_DIR, "reports")
CAMPAIGN_DIR = os.path.join(DATA_ROOT, "campaigns")
LOG_DIR      = os.path.join(DATA_ROOT, "logs")

def ensure():
    for d in (RUN_DIR, ARCHIVE_DIR, REPORTS_DIR, CAMPAIGN_DIR, LOG_DIR):
        try:
            os.makedirs(d, exist_ok=True)
        except Exception:
            pass
    return DATA_ROOT

ensure()
