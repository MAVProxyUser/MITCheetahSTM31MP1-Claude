#!/usr/bin/env python3
"""
Conductor - a fleet control panel for the star/oval/atom/dash missions.

Cannibalised from a colleague's drone-swarm "Conductor" (parrot_CHUCK,
commit 69b8381: fleet lifecycle, restart-world, live status cards) - stealing
its dark ops-console look, its play-card/fleet-card vocabulary, its
"freeze the config, then launch" discipline, and (per-dog front + nadir camera
tiles) its `.ahrs-cam` bring-up. Left out: everything drone-specific with no
quadruped analogue - MAVLink, ArduPilot profiles, wind corridors, IAMSAR
search patterns, rail launch.

RENDERING, v2. The first version drove `gz sim -g` (the native GUI) and had
trail_daemon.py post gz Markers into that scene - i.e. it still asked Gazebo
to do the showing. The first live 3-dog run under that setup collapsed all
three dogs, and the leading suspect was exactly that: GUI render load and
marker-service traffic landing on the SAME headless engine the controllers
depend on for physics, in a configuration (3 heterogeneous missions, one
engine) never tried before. So: Conductor now does its OWN rendering, the way
the original parrot_CHUCK actually works (`#flock-map`, drawn in the page,
not the sim's window) - it drives `gz sim -s -r` HEADLESS and nothing else
touches that process's transport bus for visualization. This server (running
under the venv Python so it can use gz.transport13 directly) subscribes to
world pose itself, in-process, and serves positions + the precomputed planned
paths as JSON; the browser draws both on a canvas. Gazebo now only ever does
physics for this tool.

What's kept from the original is the part that transfers directly regardless
of rendering strategy: a small always-on local server driving real
subprocesses, with one page that shows what they are doing right now.

DESIGN RULE, taken directly from this session's own post-mortem: a run must
be launched with ONE locked configuration and never touched again until it
finishes. /api/launch refuses a second call while a fleet is active - the
UI has no "edit mid-run" path at all, because "code changes mid-test mean
rerunning the test" is exactly the mistake this exists to make impossible.

Stdlib only. No build step, no framework - this repo's tests do not need one
more moving part than the C++ toolchain and Python venv it already has.
"""
import http.server
import json
import math
import os
import re
import socketserver
import shutil
import subprocess
import sys
import threading
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
GAZEBO_DIR = os.path.abspath(os.path.join(HERE, ".."))
REPO_ROOT = os.path.abspath(os.path.join(GAZEBO_DIR, "..", ".."))
HOST_RUN = os.path.join(REPO_ROOT, "host-run")
RUN_DIR = "/tmp/cheetah_conductor"
PARTITION = "cheetah_fleet"
WORLD = "go1_world"
PORT = 8420

PYBIN = ("/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/"
         "ground/gazebo_bridge/venv/bin/python3")
OPMODELS = ("/Users/kfinisterre/Desktop/OP Revo Redux/NinjaPilot-15.02.ninja/"
            "ground/gazebo_bridge/models")

# GZ_PARTITION has to be set in THIS process's own environment before the
# transport library initialises, or the Node this server creates for its own
# pose subscription lands on the default partition while gz sim runs on
# "cheetah_fleet" - they never see each other's traffic. Every subprocess
# below gets GZ_PARTITION through its own env dict; this line is what makes
# the SERVER's in-process subscriber agree with them. Found by testing: the
# topic was confirmed alive and publishing correctly, this process's own
# subscription just never received anything - a partition mismatch prints no
# error on either side, so it must be reasoned about, not caught.
os.environ["GZ_PARTITION"] = PARTITION
# GZ_IP pins gz-transport's discovery to loopback. Without it, discovery
# multicasts over whatever interface the OS's routing picks (en0 here) -
# fine until that interface's multicast route stops actually working
# (network state drifted over a long-running session: DHCP renewal, sleep/
# wake, a VPN toggle - something changed the picture underneath a Mac that
# had been up for 1+ day). Symptom, found the hard way: gz.log filling with
# "Exception sending a multicast message: No route to host" on every send,
# every dog reporting "0/1 came up" at the sensor-advertise wait, and the
# simulated robot frozen at its spawn point forever (v pinned at v_min,
# bridge log all zeros - imu_az=0, gps=(0,0) - because pose/IMU never
# actually got discovered, so the whole run just sat there looking like a
# dead controller). This host is single-machine, single-user - every peer
# in this whole system already talks over 127.0.0.1 (see the "peer=
# 127.0.0.1" in every ctrl log) - there was never a reason for gz-transport's
# OWN discovery to leave loopback in the first place.
os.environ["GZ_IP"] = "127.0.0.1"

# mission_waypoints() is pure geometry with no gz imports triggered at call
# time, but the FILE it lives in imports gz.transport13 at module level - safe
# to import here only because this server now runs under PYBIN (see
# conductor.sh), which has those bindings. Do not run this under system
# python3 any more.
sys.path.insert(0, GAZEBO_DIR)
from trail_daemon import mission_waypoints  # noqa: E402
from mission_geometry import mission_opening_bearing_rad  # noqa: E402
import shm_reaper  # noqa: E402 - "even the launcher itself can reap"
import gz.transport13 as _gz_transport      # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V     # noqa: E402
from gz.msgs10.image_pb2 import Image as _GzImage  # noqa: E402
from gz.msgs10.pose_pb2 import Pose as _GzPose      # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean as _GzBoolean  # noqa: E402
import base64                               # noqa: E402
import io                                   # noqa: E402
from PIL import Image as _PILImage          # noqa: E402

CAMERAS = ("front_cam", "nadir_cam", "chase_cam")
CAM_FLAG_KEYS = {"front_cam": "cam_front", "nadir_cam": "cam_nadir", "chase_cam": "cam_chase"}
# Cameras default OFF. Nine live feeds were repeatedly implicated in
# host-load fleet failures (GPU 44-49% with them, 0% without), and a server
# restart used to silently reset drafts back to all-on - which re-armed
# them under the operator mid-session ("KILL THE CAMERAS"). Fail dark;
# checking a box is deliberate.
DEFAULT_CAM_SLOT = dict(cam_front=False, cam_nadir=False, cam_chase=False,
                         chase_distance=3.0, chase_height=1.2, chase_degree=90.0)
# "Close final leg" defaults ON, per direct instruction. Some courses end
# where they started (the periodic curves - lissajous/spiro/atom/oval land
# 0.00-1.20 m from home) and some just stop wherever their generator's math
# ran out (circle 6.9 m, sector 15.0, expsquare 18.0, parallel 46.1), which
# shows up on the overlay as a drawn plan with its last leg missing.
# Unchecking it restores the raw generator path. It is a no-op for a dash
# (one waypoint - closing would silently make it an out-and-back) and for
# an already-closed curve; see WaypointNav::closeFinalLeg.
DEFAULT_CLOSE_LEG = True


def kind_slot_defaults(kind):
    """Per-mission-kind defaults for the two "what happens at the end"
    fields, in ONE place so the panel, the REST API and any script all get
    the same answer (this file has been bitten repeatedly by the same fact
    living in two places and drifting).

    A standalone `dash:` mission is the exception, and both fields flip:

      dash=0        - "100m dash when done" appends a 100 m sprint AFTER a
                      mission that already IS a 100 m sprint. Nonsense as a
                      default, and it invites a 200 m run nobody asked for.
      close_leg=0   - closing a dash's final leg walks it back to the origin,
                      i.e. turns it into an out-and-back. That is a DIFFERENT
                      mission this file already distinguishes by name, and
                      makeDash() exists precisely because "a dash is ONE
                      straight leg, ending at its own final waypoint" (see
                      "The standalone dash: it was never a reversal").

    Both are currently no-ops for a 1-waypoint dash by construction -
    appendDash() returns at `_n < 2` and closeFinalLeg() skips `_n < 2` - so
    this is not fixing live misbehaviour. It is making the CHECKBOXES honest
    (they claimed two things that never happened) and removing the reliance
    on that coupling, so a future dash variant with 2+ waypoints cannot
    silently become an out-and-back-plus-sprint.

    Every other mission keeps the dash finish and the closing leg, which is
    what makes the loop courses rehearse the full stop/lie-down/stand/sprint
    sequence.
    """
    if kind in ("dash", "outback"):
        return {"dash": 0, "close_leg": False}
    return {"dash": 100, "close_leg": DEFAULT_CLOSE_LEG}
import terrain  # noqa: E402 - conductor/terrain.py, procedural heightmaps

# ---------------------------------------------------------------------------
# THE HARD SPEED CAP. Not a UI suggestion - enforced here, server-side, on
# every path that sets SIM_VX, because the browser's <input max> can be
# edited in devtools and the intent ("stop pushing toward peak spec") has to
# survive that. 4.7 m/s (17 km/h) is Unitree's sprint/marketing peak; 3.5-3.7
# is the real sustained envelope (Pro/EDU rated max). 3.9 leaves a hair of
# headroom over EDU's 3.7 without ever approaching the peak number.
# ---------------------------------------------------------------------------
HARD_SPEED_CAP = 3.9

# Per-model hard cap, independent of and in ADDITION to HARD_SPEED_CAP above -
# a real Go1 Air cannot do 3.5 m/s no matter what the mission asks for, so
# picking a model must clamp the same way changing the mission's speed field
# does, enforced here server-side (never trust the browser alone). EDU's
# figure is Unitree's own "peak sprint, optimised test conditions" number
# (17 km/h) rather than its sustained rating (3.7) - kept as the ceiling
# because it is still below HARD_SPEED_CAP's ceiling and a user who
# explicitly picks EDU has already opted into its top end.
MODEL_MAX_SPEED = {"air": 2.5, "pro": 3.5, "edu": 4.7}
DEFAULT_MODEL = "edu"

# Per-mission-kind recipe: gait, and the extra knobs THIS SESSION measured to
# be the best validated configuration for that course shape (see CLAUDE.md,
# "CAMPAIGN RESULTS"). The UI can override gait and speed; it does not expose
# a way to change these, on purpose - they are not tunables, they are the
# answer.
RECIPES = {
    "star": dict(gait=5, speed=3.5, extra="WP_ACCEPT=1.5 WP_ALAT=3.25 WP_CORRIDOR_MIN=0.1",
                 note="trotRunning, lateral budget 3.25, graded corridor - UNDER RE-TUNE"),
    # VSUS 2.6 was the campaign-era edge and no longer holds on the current
    # dynamics lineage: a controlled bisect (last night's binary vs HEAD,
    # semi-interleaved, solo) showed BOTH fall mid-180 at ~50-75% with the
    # yaw command saturated at the lateral cap - the cell was always
    # marginal, the old 6/6 belongs to a much older build. 2.4 is the
    # re-measured envelope: its first run under the operator's own eyes ran
    # the curves clean AND the full stop/lie-down/stand-up/dash sequence.
    # CHANGED 2026-08-27: the trotRunning+analyzer-switch config below this
    # comment (kept in history, not deleted) was investigated at length
    # after a direct challenge not to accept its ~1-in-3 failure rate as
    # unfixable. Three separate fixes to the SWITCH's own timing/dynamics
    # gating all failed; the decisive test (disabling the switch entirely)
    # showed trotRunning cannot hold this course's R=4.47m sustained curve
    # AT ALL, independent of switching, down to 1.8 m/s. See CLAUDE.md
    # "THE OVAL'S MID-COURSE FALL" for the full investigation. trotting for
    # the whole course is slower (~46s vs the old ~30.5s best case) but
    # PASSED 2/2 clean where the old config's own documented history was
    # marginal. Do not revert without re-reading that section first.
    "oval": dict(gait=9, speed=2.4, extra="WP_ANALYZER=0",
                 note="trotting, whole course, no analyzer - trotRunning cannot hold this curve at any tested speed (see CLAUDE.md)"),
    # OLD (marginal, ~1-in-3 failure at the sustained-curve entry):
    # "oval": dict(gait=5, speed=3.5, extra="WP_ANALYZER=1 WP_VSUS=2.4",
    #              note="trotRunning, analyzer, sustained cap 2.4 (re-swept 2026-08-24; 2.6 is past the current envelope)"),
    # THE ATOM'S FAILURE IS PITCH, NOT ROLL - so the lever is LONGITUDINAL.
    # Every trip measured on this course reads pitch-dominant: 30.5, 33.0,
    # 35.6, 35.9, 36.6, 36.9 deg of PITCH with roll in the teens. Lowering
    # the lateral budget (tried, WP_ALAT=1.8) governs ROLL and did nothing,
    # because roll was never what tripped.
    #
    # Pitch is fore/aft, i.e. braking and driving. plan() picks a_lon from
    # cruise speed: `(v_cruise >= 2.2) ? 0.4 : 1.5`. Star and oval cruise at
    # 3.5 and get the gentle 0.4; the atom cruises at 2.1 and therefore
    # falls into the 1.5 branch - 3.75x more longitudinal demand than either
    # of them, against a body measured to track ~1.2 m/s^2, and on the ONLY
    # course whose curvature varies continuously (so it is braking and
    # driving the whole lap, never coasting). That is a pitch excitation
    # applied nonstop. WP_ALON=0.4 gives it the same gentle profile the two
    # courses that pass already use.
    #
    # The speed-keyed default is the real bug here: it assumes slow = safe
    # to brake hard, which is exactly backwards for a slow course that is
    # ALWAYS turning.
    "atom": dict(gait=9, speed=2.1, extra="WP_ALON=0.4",
                 note="trotting, bare (analyzer adds nothing here) - 58.97s @ 6/6"),
    # Spirograph rosette (makeSpirograph) - per direct challenge to render a
    # specific reference Spirograph image as a mission, "shoot for the moon."
    # Same trochoid family as atom (same formula, k=lobes not lobes-1, depth
    # near 1.0 instead of clamped well short of it - see WaypointNav.cpp's
    # own comment on makeSpirograph for the exact relationship) - starting
    # from atom's own proven-good gait/speed/tuning as the baseline, same
    # reasoning: a smooth continuous curve, not discrete sharp vertices, so
    # trotting's own established comfort with curvy/continuous courses
    # applies here too pending its own live results.
    "spiro": dict(gait=9, speed=1.8, extra="WP_ALON=0.4",
                  note="trotting @ 1.8 m/s, atom's own bare tuning as a "
                       "starting point - untested, first attempt pending"),
    "dash": dict(gait=5, speed=3.0, extra="",
                 note="trotRunning straight-line - UNDER REVIEW, see README"),
    # Search-and-rescue patterns (International Aeronautical and Maritime
    # Search and Rescue Manual) and Lissajous search curves (Steckenrider
    # et al. 2024). None of these have campaign data behind them yet -
    # walking at a modest speed is this port's most broadly reliable
    # combination (see the gait-matrix findings in CLAUDE.md), used here
    # as an honest, conservative default rather than a measured one.
    # First attempt at 2.0 m/s bare fell AFTER cleanly capturing all 8
    # waypoints - a severe pitch-dominant tip (roll=-32 pitch=67) in the
    # end-of-mission stop, not the course itself. Same standard tuning fixed
    # it - but "constant R=7.48m the whole way, no corners to speak of" (the
    # ORIGINAL note here) undersold what that number actually meant: circle's
    # 45deg per-vertex turns sit BELOW turn_soft's default 80deg threshold,
    # so the corridor-grading mechanism never engages AT ALL - every corner
    # was cut at a fillet radius comparable to the course's own 9m radius,
    # at full 1.5 m/s cruise, no braking whatsoever. Visually: an 8-point
    # circle search reading as a soft, undersized blob instead of tracing
    # its own vertices. Per direct report ("SAR variants still round
    # corners off real bad"). Probed like sector: turn_soft/turn_hard
    # narrowed to bracket circle's OWN 45deg angle (was tuned for star's
    # 144/162deg) brings the fillet to R=1.43m - a 5x tightening - using
    # the same corridor_scale_min=0.07 sector already shipped with.
    "circle": dict(gait=20, speed=1.5,
                    extra="WP_ACCEPT=1.5 WP_CORRIDOR_MIN=0.07 WP_ALON=0.4 "
                          "WP_TURN_SOFT=0.3 WP_TURN_HARD=0.79",
                    note="walking @ 1.5 m/s, graded corridor + gentle WP_ALON + "
                         "turn-grading ramp narrowed for circle's own 45deg "
                         "corners, R=1.43m - PASS 30.2s (was 32.8s/34.3s at "
                         "earlier steps; R=7.48m/no braking untightened). "
                         "ALL TIMES HERE ARE LEG-OPEN. This course stops "
                         "6.89m from home, so 'Close final leg' (default ON) "
                         "adds a real leg: measured +6.8% (58.5->62.5s) at "
                         "trotting 2.5, +7.8% at bounding 1.0, +8.8% at "
                         "galloping 0.8 - the slower the gait, the longer "
                         "that same walk home takes. Still passes either way."),
    # First attempt fell twice: once with the default corridor (tightest
    # First attempt fell twice: once with the default corridor (tightest
    # corner R=0.39-0.52m, untouched) at t~22s, once with only
    # WP_CORRIDOR_MIN=0.1 graded (R tightens further to 0.14m once graded,
    # correctly, but the fall was PITCH-dominant - the same signature the
    # atom needed WP_ALON for). WP_ALON=0.4 fixed it: PASS 112.8s, clean.
    # WP_TURN_SOFT/WP_TURN_HARD added per direct request to track sector's
    # corners as tightly as star's. Root cause (measured via a standalone
    # BodyPathPlanner probe, not guessed): the corridor-grading curve's
    # defaults (turn_soft=80deg, turn_hard=160deg) were tuned around STAR'S
    # OWN two corner angles (144/162deg) - star's mildest corner already
    # sits 80% up that ramp. Sector's corners are ALL 120-147.5deg, direction
    # changes that are real corners by the field comment's own definition
    # ("1.4 rad = 80 deg (a real corner)") but land only ~50% up star's ramp,
    # so the SAME already-working mechanism was firing at half strength:
    # sector's dominant 120deg corner got R=0.83m/v=1.0m/s versus star's
    # 144deg corner at R=0.19m/v=0.23m/s - 4x wider AND 4x faster, by
    # geometry, not by a tracking failure. Narrowing the ramp to 46-115deg
    # (so 120deg sits near ITS top) brings sector's dominant corner to
    # R=0.15m/v=0.18m/s and its sharpest to R=0.058m - star's own ballpark.
    # Scoped to this recipe only (WP_TURN_SOFT/WP_TURN_HARD default to
    # 1.4/2.8 rad when unset), so star/oval/atom/dash are untouched.
    #
    # WP_CORRIDOR_MIN 0.1 -> 0.07 per direct follow-up ("still a bit weak
    # vs star... tighten it up just a hair"): at turn_soft=0.8/turn_hard=2.0,
    # ALL of sector's corner angles (120/127.5/147.5deg) already sit ABOVE
    # turn_hard, so the turn_soft/turn_hard ramp is fully saturated (f=1.0)
    # for every one of them - there was no more headroom left in THAT knob
    # (confirmed via the probe: lowering turn_soft/turn_hard further changed
    # nothing). The remaining lever is corridor_scale_min itself, the FLOOR
    # that ramp clamps to: eff_corridor = corridor * corridor_scale_min at
    # f=1. Probed 0.10/0.07/0.05/0.03; picked 0.07 as the requested "hair" -
    # a 30% radius cut (120deg corner: R=0.150m -> 0.105m, comfortably
    # inside star's own 0.028-0.191m corner range) without the much larger
    # speed cost 0.05/0.03 would add (v_min 0.180 -> 0.126 m/s, vs 0.090/
    # 0.054 at the more aggressive settings).
    # WP_FINAL_ACCEPT=0.3 added per direct report: the flown-trail plot
    # visibly fell short of the true start point on a closed course. Root
    # cause (see WaypointNav.hpp's final_accept_radius field comment):
    # WP_ACCEPT doubles as the legacy waypoint-arrival radius AND the
    # BodyPathPlanner cornering corridor width, so tuning it for tight
    # corners (1.5m here) also means the MISSION-END waypoint counts as
    # "arrived" - ending the run - a full 1.5m short of its literal
    # coordinate. Confirmed in the raw log before touching anything:
    # "reached wp15 (N=-0.00 E=0.00) dist=1.49" - the dog stopped 1.49m
    # from the true origin. New, opt-in final_accept_radius decouples the
    # two: 0.3m closes the gap to a few tenths of a metre without the
    # dog hunting/oscillating at an unrealistically tight tolerance.
    "sector": dict(gait=20, speed=2.0,
                    extra="WP_ACCEPT=1.5 WP_CORRIDOR_MIN=0.07 WP_ALON=0.4 "
                          "WP_TURN_SOFT=0.8 WP_TURN_HARD=2.0 WP_FINAL_ACCEPT=0.3",
                    note="walking, graded corridor + gentle WP_ALON + narrowed "
                         "turn-grading ramp + tightened corridor floor + tight "
                         "final-waypoint closure for star-tight corners - "
                         "ends 15.0m from home, so 'Close final leg' "
                         "(default ON) adds that walk; its sharpest angle "
                         "is unchanged (32.5deg either way) since this "
                         "course already had one that tight. Times below "
                         "are LEG-OPEN and not yet re-measured closed. "
                         "PASS 141.5s/141.8s (2/2), final wp closes to "
                         "dist=0.25-0.27m (was 1.49m) "
                         "(prior step: PASS 140.9s at CORRIDOR_MIN=0.07; "
                         "was 112.8s untightened, the expected cost of "
                         "hugging every vertex instead of cutting it)"),
    # First fall was NOT the 180-degree end-of-pass turn itself (tracked
    # that cleanly at full speed, hdg swinging smoothly 0->180) - it was
    # the SHORT 5m connector immediately after, still carrying a full 2.0
    # m/s of straight-line momentum into a sharp turn. Same corridor/ALON
    # tuning as sector did not fix it at 2.0; dropping cruise to 1.5 did -
    # less momentum to shed before the connector, not a geometry problem.
    #
    # WP_TURN_SOFT/WP_TURN_HARD/WP_CORRIDOR_MIN=0.07 added per direct
    # report ("SAR variants still round corners off real bad"), same
    # probe-first methodology as sector and circle: parallel's every
    # corner is a constant 90deg turn, which sits just barely inside the
    # DEFAULT grading window (turn_soft=80deg) - only 12.5% graded - so
    # every turn was a fillet at R=2.25m, full 1.5 m/s cruise, no braking
    # at all. Narrowing the ramp to bracket 90deg (turn_hard=86deg, so
    # 90deg sits fully past it) plus corridor_scale_min=0.07 (matching
    # sector/circle) brings it to R=0.253m, v_min=0.304 m/s - real,
    # visible corners instead of a smoothed-over lawnmower pattern.
    # WP_CORRIDOR_MIN 0.07 -> 0.05: a further step per "tighter angles,
    # less corner rounding" - parallel's 30m straights give far more
    # braking distance than expsquare's short spiral legs (which stayed
    # at 0.07 for exactly that reason - it already fell once at the
    # LOOSER setting, more braking demand is the wrong direction there).
    # R 0.253m -> 0.181m, v_min 0.304 -> 0.217 m/s. PASS x2 (182.3s,
    # 182.0s), negligible extra time cost for a visibly sharper corner.
    "parallel": dict(gait=20, speed=1.5,
                      extra="WP_ACCEPT=1.5 WP_CORRIDOR_MIN=0.05 WP_ALON=0.4 "
                            "WP_TURN_SOFT=0.6 WP_TURN_HARD=1.5",
                      note="walking @ 1.5 m/s, graded corridor + gentle WP_ALON + "
                           "turn-grading ramp narrowed for parallel's own 90deg "
                           "corners, tightened further to R=0.181m - PASS 181-182s "
                           "x2 LEG-OPEN (was 158.2s at the original untightened "
                           "R=2.25m/no-braking baseline). This course ends 46.1m "
                           "from home - the LARGEST gap in the catalog - so "
                           "'Close final leg' (default ON) adds a substantial "
                           "walk plus a 90->49.4deg closing corner: MEASURED "
                           "PASS 212.8s closed, +17% over the 181-182s open "
                           "baseline - the largest close-leg cost in the "
                           "catalog, as its gap predicts."),
    # Same fix, same reasoning, same numbers as parallel immediately above -
    # expsquare's corners are ALSO a constant 90deg turn (its 5 non-trivial
    # legs alternate direction by exactly 90deg each), so it had the
    # identical R=2.25m/full-cruise problem and gets the identical
    # WP_TURN_SOFT/WP_TURN_HARD/WP_CORRIDOR_MIN treatment.
    # Deliberately NOT tightened past 0.07 the way parallel was (same
    # 90deg angle, same formula) - expsquare's early spiral legs are only
    # 5m (parallel's straights are 30m), giving far less distance to shed
    # speed into an even-tighter corner. Already saw ONE fall at this
    # exact setting (SAFETY CHECK FAILED, wp01->wp02, the two shortest
    # legs in the whole course) before shiftFirstToOrigin existed;
    # confirmed since as PASS x4 / FELL x1 across 5 attempts (109.0-109.2s
    # each pass) - more braking demand is the wrong direction on this
    # course, not an oversight.
    # THE PER-ANGLE CORNERING PROBE. corner:<leg_m>:<angle_deg> is one
    # isolated corner with a real approach and a real exit - built for the
    # "empirical per-gait, per-angle cornering envelope" question, where
    # every other course only offers whatever angles its own shape happens
    # to contain (45 from circle, 90 from parallel/expsquare, 120-147.5
    # from sector, 144/162 from star).
    #
    # It had NO recipe entry at all until now, which is exactly why it
    # never worked: with no entry it launched with zero tuning while every
    # other course got its own, and the resulting overshoot/pitch blowup
    # (53.8 deg at settle) was read as a planner bug in the mission. Given
    # the same graded-corridor + gentle-WP_ALON treatment every other
    # cornering course needed, it just works.
    #
    # TURN_SOFT/HARD are deliberately WIDE (0.3-2.0 rad = 17-115 deg) rather
    # than bracketing one angle, because a probe mission is swept ACROSS
    # angles - a per-angle-tuned window would be the one thing that makes
    # the sweep measure its own tuning instead of the robot. Verified with
    # this single tuning at three angles: 45 deg PASS 61.3s, 90 deg PASS
    # 54.9s, 135 deg PASS 47.8s, every settle under 0.7 deg roll.
    "corner": dict(gait=20, speed=1.5,
                    extra="WP_ACCEPT=1.5 WP_CORRIDOR_MIN=0.07 WP_ALON=0.4 "
                          "WP_TURN_SOFT=0.3 WP_TURN_HARD=2.0",
                    note="walking @ 1.5 m/s - ONE isolated corner of "
                         "<angle> deg, the per-angle cornering probe. Same "
                         "graded-corridor + gentle-WP_ALON treatment every "
                         "other cornering course needed; it had no recipe at "
                         "all before, which is why it used to overshoot and "
                         "pitch out. Turn-grading window kept WIDE on purpose "
                         "so a sweep across angles measures the ROBOT, not a "
                         "per-angle tuning. Verified 45/90/135 deg all PASS "
                         "(61.3/54.9/47.8s), settle roll <=0.7 deg."),
    "expsquare": dict(gait=20, speed=1.5,
                       extra="WP_ACCEPT=1.5 WP_CORRIDOR_MIN=0.07 WP_ALON=0.4 "
                             "WP_TURN_SOFT=0.6 WP_TURN_HARD=1.5",
                       note="walking @ 1.5 m/s, graded corridor + gentle WP_ALON + "
                            "turn-grading ramp narrowed for expsquare's own 90deg "
                            "corners, R=0.253m - PASS x4/FELL x1 in 5 attempts, "
                            "109.0-109.2s LEG-OPEN; with 'Close final leg' "
                            "(default ON) PASS 121.3s, +11%, and the closing "
                            "corner is the SHARPEST on the course (90->33.7deg) "
                            "so this turn-grading tuning was never measured "
                            "against it - it passes, but re-check if retuning. "
                            "(was PASS 87.4s untightened, "
                            "R=2.25m/no braking at every corner). NOT tightened "
                            "further - see comment above."),
    # PASS 96.2s on 1:2 (141m), PASS 345.9s on 5:7 (558m) - same tuning as
    # the SAR patterns. Higher ratios are genuinely LONG missions (558m for
    # 5:7 alone) - the controller process's own safety-net timeout was
    # raised 240s -> 900s in server.py's launch command specifically
    # because 240 silently killed a healthy 5:7 run two-thirds through
    # with no error at all (looked exactly like a crash - see CLAUDE.md).
    "lissajous": dict(gait=20, speed=1.5, extra="WP_ACCEPT=1.5 WP_CORRIDOR_MIN=0.1 WP_ALON=0.4",
                       note="walking @ 1.5 m/s - PASS 1:2 96.2s, 5:7 345.9s, 11:9 561.7s"),
}
GAITS = {"trotting": 9, "trotRunning": 5, "walking": 20, "walking2": 21, "pacing": 8,
         # Flight gaits, added to run the pronk/gallop/bound re-test across the
         # mission catalog now that qpOASES+WBIC damping is the shipped config -
         # not part of any recipe's default, opt in explicitly via --gait.
         "pronking": 2, "galloping": 22, "bounding": 1}
GAIT_NAMES = {v: k for k, v in GAITS.items()}


def _gname(num_str):
    try:
        return GAIT_NAMES.get(int(num_str), num_str)
    except ValueError:
        return num_str


# Discrete events worth a line in the orchestration log, matched against NEW
# ctrl_%d.log text each poll tick (see _start_poller). This is deliberately
# NOT the continuous per-tick telemetry ([nav] wp.../d=.../v=... already
# drives the live fleet-card text) - only state transitions: init, standing
# up, gait engagement, mission pre-planning, gait switches, the dash
# interlude, and anything that trips a safety/governor path. Each entry is
# (compiled regex, formatter(match) -> message). Order matters only in that
# earlier patterns are tried first per line; every line matches at most one.
EVENT_PATTERNS = [
    (re.compile(r"\[mit-sim\] Gazebo SITL \| peer=(\S+) robot=(\S+) user=(\S+)"),
     lambda m: "initialising Cheetah MIT controller (peer=%s, %s / %s)" % m.groups()),
    (re.compile(r"\[stm32mp1\] REAL ESTIMATOR: (.+)"),
     lambda m: "estimator: %s" % m.group(1)),
    (re.compile(r"\[sim\] control_mode -> STAND_UP"),
     lambda m: "standing up"),
    (re.compile(r"\[sim\] control_mode -> BALANCE_STAND"),
     lambda m: "balance stand - settling before locomotion"),
    (re.compile(r"\[sim\] control_mode -> 4\b"),
     lambda m: "entering LOCOMOTION"),
    (re.compile(r"\[nav\] LOCOMOTION engaged - holding ([\d.]+) s"),
     lambda m: "gait engaged - holding %ss for it to settle before nav takes over" % m.group(1)),
    (re.compile(r"\[nav\] taking the stick at t=([\d.]+)s \(mission (\S+)\)"),
     lambda m: "nav taking the stick - mission %s under way" % m.group(2)),
    (re.compile(r"\[plan\] (\d+) pts, ([\d.]+) m, tightest R=([\d.]+) m -> ([\d.]+) m/s "
                r"\(cruise ([\d.]+), a_lat ([\d.]+), corridor ([\d.]+)\)"),
     lambda m: ("pre-planning mission: %s pts, %sm path, tightest corner "
                "R=%sm -> %sm/s (cruise %sm/s, a_lat %s)" % m.groups()[:6])),
    (re.compile(r"\[mission\] (\d+) segments over ([\d.]+) m"),
     lambda m: "mission analyzer: %s segments over %sm" % m.groups()),
    (re.compile(r"\[mission\] ([\d.]+) s lost to constraints; costliest FEATURE "
                r"([+\-\d.]+) s at s=([\d.]+) \((\S+), R=([\d.]+) m\)"),
     lambda m: ("analyzer: %ss lost to constraints - costliest feature %ss at "
                "s=%sm (%s, R=%sm)" % m.groups())),
    (re.compile(r"\[mission\] (\d+) sustained-curve segments, (\d+) gait changes planned"),
     lambda m: "analyzer plan: %s sustained-curve segments, %s gait changes scheduled" % m.groups()),
    (re.compile(r"\[gait\] (\d+) -> (\d+) at v=([\d.]+) \(planned ([\d.]+)\) t=([\d.]+)s"),
     lambda m: "gait change: %s -> %s at v=%sm/s (planned %sm/s)" % (
         _gname(m.group(1)), _gname(m.group(2)), m.group(3), m.group(4))),
    (re.compile(r"\[mission\] gait (\d+) -> (\d+) entering (\S+) at s=([\d.]+) "
                r"\(R=([\d.]+), cost ([+\-\d.]+) s\) t=([\d.]+)s"),
     lambda m: "gait change (pre-planned): %s -> %s entering %s terrain (R=%sm)" % (
         _gname(m.group(1)), _gname(m.group(2)), m.group(3), m.group(5))),
    (re.compile(r"\[HGOV\] .*dep=([+\-\d.]+)"),
     lambda m: "height governor active (departure %sm) - trading speed for stance height" % m.group(1)),
    (re.compile(r"\[nav\] loop complete at t=([\d.]+)s - stop, lie down, "
                r"stand back up before the (\S+) dash"),
     lambda m: "loop complete - stopping, lying down before the dash finish"),
    (re.compile(r"\[nav\] back up at t=([\d.]+)s - dashing the final leg"),
     lambda m: "back on its feet - dashing the final leg"),
    (re.compile(r"\[mission\] settle: z=([\d.]+) roll=([\d.]+) pitch=([\d.]+) -> (ok|BAD)"),
     lambda m: "settled on its feet: z=%sm roll=%s pitch=%s -> %s" % m.groups()),
    (re.compile(r"\[mission\] laydown: z=([\d.]+) roll=([\d.]+) pitch=([\d.]+) -> (ok|BAD)"),
     lambda m: "lying down: z=%sm roll=%s pitch=%s -> %s" % m.groups()),
    (re.compile(r"\[mission\] RESULT: (PASS|FAIL)"),
     lambda m: "mission result: %s" % m.group(1)),
    (re.compile(r"Orientation safety check failed!"),
     lambda m: "SAFETY CHECK FAILED - orientation exceeded the trip limit"),
    (re.compile(r"STATE ESTIMATE WENT NON-FINITE"),
     lambda m: "state estimate went non-finite - reinitialising"),
]


def mission_kind(spec):
    kind = spec.split(":", 1)[0]
    # The 100 m dash is spelled "outback:<m>" in WaypointNav (out and back
    # along one line) but its RECIPES entry is keyed "dash", which is what
    # the panel's dropdown calls it. Without this alias the lookup misses,
    # recipe comes back empty and launch() dies on recipe["note"] - i.e.
    # the dash course could not be launched from the panel AT ALL, and the
    # failure was silent except for a KeyError in the server log.
    # "dash:<m>" is the real straight sprint; "outback:<m>" is the legacy
    # out-and-back (100 out, 100 BACK) and is NOT what the panel means by dash.
    return "dash" if kind in ("outback", "dash") else kind


def archive_log(path, prev_run_id):
    """Archive an existing per-run log before the next launch's fresh
    open(path, "w") truncates it. Per direct request, after losing exactly
    the evidence needed for an expsquare fall to the very next test launched
    a couple of minutes later: gz.log/bridge_N.log/ctrl_N.log all get
    reopened in "w" mode on every single launch, in place, with nothing
    keeping the PREVIOUS run's copy around - fine for a quick A/B, a real
    loss the moment something needs investigating after the fact.

    Same directory (RUN_DIR/archive/), date-stamped AND run-id-stamped, so
    a past run's logs are both sorted chronologically and unambiguously
    tied to the run number already stamped into every orchestration log
    line. No retention/pruning - that is a separate, explicit decision for
    whoever needs it, not bundled into this fix.
    """
    if not os.path.exists(path):
        return
    try:
        archive_dir = os.path.join(RUN_DIR, "archive")
        os.makedirs(archive_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        dest = os.path.join(archive_dir, "%s_run%s_%s" % (
            stamp, prev_run_id, os.path.basename(path)))
        shutil.move(path, dest)
    except Exception:  # noqa: BLE001 - archiving must never block a launch
        pass


def clamp_speed(v, cap, model=None):
    v = max(0.3, min(float(v), HARD_SPEED_CAP))
    cap = max(0.3, min(float(cap), HARD_SPEED_CAP))
    if model is not None:
        cap = min(cap, MODEL_MAX_SPEED.get(model, HARD_SPEED_CAP))
    return round(min(v, cap), 2)


def read_host_load():
    """CPU 1-minute load average (normalised by core count, as a 0-100+ %
    figure - can exceed 100 under real contention, which is the honest
    answer) and GPU device utilisation, both stdlib/no-sudo on macOS. Called
    from a background sampler, not per-request - ioreg is a subprocess spawn,
    cheap but not free, and nothing about this needs sub-second freshness.
    Same motivation as the MISSED CONTROL DEADLINES section in CLAUDE.md:
    this project has already been bitten once by machine load silently
    explaining an "unexplained" sim failure, so make it visible instead of
    assumed quiet."""
    try:
        load1, _, _ = os.getloadavg()
        cpu_pct = round(100.0 * load1 / max(1, os.cpu_count()), 1)
    except (OSError, AttributeError):
        cpu_pct = None
    gpu_pct = None
    try:
        out = subprocess.run(["ioreg", "-r", "-d", "1", "-c", "IOAccelerator"],
                              capture_output=True, text=True, timeout=2).stdout
        m = re.search(r'"Device Utilization %"\s*=\s*(\d+)', out)
        if m:
            gpu_pct = int(m.group(1))
    except (subprocess.SubprocessError, OSError):
        pass
    return dict(cpu_pct=cpu_pct, gpu_pct=gpu_pct)


# ---------------------------------------------------------------------------
# Fleet manager - owns every subprocess and the polled state derived from
# their logs.
# ---------------------------------------------------------------------------
class Fleet:
    def __init__(self):
        self.lock = threading.Lock()
        self.phase = "idle"          # idle | launching | running | done | error
        self.slots = []              # locked config, set at launch
        self.status = []             # per-slot live status, updated by poller
        self.log = []                # tail of orchestration events
        self.procs = []              # every Popen this run started
        self.started_at = None
        self._poll_thread = None
        self.planned = {}            # index -> [[x,y], ...] world frame, frozen at launch
        self.positions = {}          # index -> {"x","y","z","yaw","speed","trail":[[x,y],...]}
        self._name_to_index = {}     # "go1_2" -> 2, for the pose subscriber
        self._last_pose = {}         # index -> (x, y, t) for the speed EMA
        self._gz_node = None         # keep the Node alive - gc'ing it drops the subscription
        self._chase_stop = None      # threading.Event - signals _follow_chase_cams to exit
        # THE ASYNC-TEARDOWN RACE, closed structurally. self.phase = "done"
        # was being set the INSTANT the poller noticed every dog finished,
        # while the actual process kill (terminate() -> sleep(1) -> kill())
        # ran afterward, unlocked, on the same background thread - so a
        # caller (a human clicking Launch again, or a test harness driving
        # mission_runner.py back-to-back) could see phase go idle/done and
        # fire a NEW launch while the OLD gz/bridge/controller processes
        # were still alive and, critically, still holding open file handles
        # on the very ctrl_N.log/bridge_N.log files the new launch was about
        # to truncate - a few of the dying run's last buffered lines could
        # land AFTER the new run's fresh open(path, "w"), silently
        # contaminating the new run's own log with old content. Reproduced
        # directly: a star mission immediately followed by an atom mission
        # returned a bogus PASS in 10.4s (atom's real floor is 55+s) because
        # the new ctrl log's tail was actually star's. This Event is SET
        # whenever no teardown is in flight and CLEARED the moment one
        # starts; it is only set again once every process from the previous
        # run has been CONFIRMED dead (a real wait(), not just kill() and
        # assume) - see _reap_and_confirm(). launch() waits on it before
        # doing anything else, so a new launch is now structurally unable to
        # start while a previous teardown is still completing, regardless of
        # how fast the caller fires the next request.
        self._teardown_done = threading.Event()
        self._teardown_done.set()
        # DRAFT config, mutated by /api/slots* and /api/speed_cap. This is
        # what "+ Add dog", editing a field, or dragging the cap slider does
        # in the browser - moved server-side so every one of those actions is
        # independently REST-automatable, not just the final Launch call.
        # dash=100: per direct instruction, the 100 m dash is how a course
        # ENDS, not an opt-in toggle - stop, lie down, stand back up, then
        # sprint 100 m on the closing heading, after every loop mission.
        # cam_front/cam_nadir/cam_chase default all-on (matches what already
        # shipped); chase_distance/height/degree are the side-view default
        # ("side view chase camera... hover over the dogs chasing").
        # DERIVED FROM RECIPES, never duplicated. These three used to carry
        # their own literal gait/speed, which is a second source of truth for
        # the same fact - and it drifted, exactly the way a duplicated
        # decision always does here. The oval's recipe was re-tuned to
        # trotting @ 2.4 when trotRunning was measured unable to hold its
        # sustained curve (see "THE OVAL'S MID-COURSE FALL"), RECIPES was
        # updated, and this list was not - so the panel's DEFAULT oval slot
        # went on launching trotRunning @ 3.5, the configuration that
        # investigation had just finished proving broken, under a note that
        # correctly described trotting @ 2.4. Operator-spotted from the
        # panel's own "not this course's validated combo" warning.
        # Same defect shape as the atom spin-out (run17) further up this
        # file: a UI showing the right label over the wrong command.
        # draft_add_slot() already derived from RECIPES correctly; only this
        # initial draft did not, so now both use the one source.
        _CORE_MISSIONS = [("star", "star:10.514:5"), ("oval", "oval:40:5.0"),
                          ("atom", "atom:9.0:6")]
        self.draft_slots = [
            dict(mission=spec,
                 gait=next(g for g, n in GAITS.items() if n == RECIPES[kind]["gait"]),
                 speed=RECIPES[kind]["speed"], model=DEFAULT_MODEL,
                 **kind_slot_defaults(kind), **DEFAULT_CAM_SLOT)
            for kind, spec in _CORE_MISSIONS
        ]
        self.draft_cap = 3.5
        # Terrain, from terrain.py. "flat" reproduces the EXACT ground_plane
        # every campaign result was measured on; anything else is new,
        # unvalidated ground and stays opt-in for exactly that reason.
        self.draft_terrain = "flat"
        self.cameras = {}             # index -> {"front_cam": "data:...", "nadir_cam": "data:..."}
        self._gz_cam_nodes = []       # one Node per camera subscription, kept alive
        # RUN NUMBER. Monotonic, persisted across server restarts, so the
        # operator can say "run 47's atom did X" and both of us mean the same
        # run. Shown in the panel, stamped into every orchestration log line,
        # and passed to each controller as $SIM_RUN_ID so it lands in that
        # dog's own ctrl log too.
        self.run_id = 0
        try:
            with open(os.path.join(RUN_DIR, "run_seq.txt")) as f:
                self.run_id = int(f.read().strip() or 0)
        except Exception:  # noqa: BLE001 - first boot, or a wiped /tmp
            self.run_id = 0
        self.host_load = read_host_load()
        threading.Thread(target=self._host_load_loop, daemon=True).start()

    def _host_load_loop(self):
        while True:
            load = read_host_load()
            with self.lock:
                self.host_load = load
            time.sleep(1.5)

    def _note(self, msg):
        tag = ("run%d " % self.run_id) if self.run_id else ""
        self.log.append("[%s] %s%s" % (time.strftime("%H:%M:%S"), tag, msg))
        self.log = self.log[-200:]
        print(msg, flush=True)

    def snapshot(self):
        with self.lock:
            return {
                "phase": self.phase,
                "slots": self.slots,
                "status": self.status,
                "log": self.log[-60:],
                "run_id": self.run_id,
                "hard_cap": HARD_SPEED_CAP,
                "model_max_speed": MODEL_MAX_SPEED,
                "recipes": RECIPES,
                "gaits": GAITS,
                "planned": self.planned,
                "positions": self.positions,
                "draft_slots": self.draft_slots,
                "draft_cap": self.draft_cap,
                "draft_terrain": self.draft_terrain,
                "terrain_types": terrain.TERRAIN_TYPES,
                "cameras": self.cameras,
                "host_load": self.host_load,
            }

    # ---- draft editing: one method per interactive element ---------------
    # Every one of these is what a click/edit in the browser does; each is
    # also its own REST route, so the panel and a script hit the exact same
    # code path and can never drift apart.
    def draft_add_slot(self):
        with self.lock:
            if len(self.draft_slots) >= 3:
                return False, "max 3 slots"
            used = {mission_kind(s["mission"]) for s in self.draft_slots}
            # "+ Add dog" only cycles the three courses with an actual
            # default mission STRING below (star/oval/atom) - dash and
            # every SAR/Lissajous kind are selected via the mission
            # dropdown instead (which snaps gait/speed/extra to the right
            # recipe on selection), same reason dash was already excluded:
            # this dict has no default mission string for them, and a
            # wrong fallback (silently defaulting to star's) would launch
            # a mismatched recipe under the wrong course's own note text.
            CORE = ("star", "oval", "atom")
            nxt = next((k for k in CORE if k not in used), "star")
            r = RECIPES[nxt]
            self.draft_slots.append(dict(
                mission={"star": "star:10.514:5", "oval": "oval:40:5.0",
                         "atom": "atom:9.0:6"}.get(nxt, "star:10.514:5"),
                gait=next(g for g, n in GAITS.items() if n == r["gait"]),
                speed=r["speed"], model=DEFAULT_MODEL,
                **kind_slot_defaults(nxt), **DEFAULT_CAM_SLOT))
            return True, self.draft_slots

    def draft_remove_slot(self, i):
        with self.lock:
            if not (0 <= i < len(self.draft_slots)):
                return False, "no such slot"
            # The last slot IS removable. It used to be pinned ("at least one
            # slot required"), which meant dog 0 had no delete button at all
            # once it was the only one - operator-reported, and the wrong
            # trade: an empty draft is a perfectly sensible thing to want as a
            # starting point for building a fleet from scratch, and it is
            # launch() - not this - that has to refuse an empty fleet. Guard
            # added there instead, where the actual constraint lives.
            self.draft_slots.pop(i)
            return True, self.draft_slots

    def draft_clear_slots(self):
        """Remove every draft slot at once. Operator-requested: rebuilding a
        fleet meant clicking remove up to three times, each one a separate
        round trip that re-indexed the remaining slots (the same re-indexing
        that caused the removal race documented in CLAUDE.md). One call, one
        server-truth response, no intermediate states to race."""
        with self.lock:
            self.draft_slots = []
            return True, self.draft_slots

    def draft_set_slot(self, i, fields):
        with self.lock:
            if not (0 <= i < len(self.draft_slots)):
                return False, "no such slot"
            s = self.draft_slots[i]
            if "mission" in fields:
                prev_kind = mission_kind(s.get("mission", ""))
                s["mission"] = str(fields["mission"])
                new_kind = mission_kind(s["mission"])
                # Switching INTO or OUT OF a standalone dash re-applies that
                # kind's own end-of-mission defaults, because they are
                # opposites: a dash wants no sprint-finish and no closing leg,
                # every loop course wants both. Only on an actual kind CHANGE,
                # so re-selecting the same kind never stomps a deliberate
                # per-slot override. An explicit dash/close_leg in the SAME
                # request still wins - the branches below run after this and
                # overwrite it, which is what lets the panel send
                # mission+dash+close_leg together in one POST.
                if new_kind != prev_kind:
                    s.update(kind_slot_defaults(new_kind))
            if "gait" in fields and fields["gait"] in GAITS:
                s["gait"] = fields["gait"]
            if "model" in fields and fields["model"] in MODEL_MAX_SPEED:
                s["model"] = fields["model"]
                # Re-clamp immediately - switching to a slower model with an
                # already-fast speed set must visibly drop it, not silently
                # cap it only at launch time.
                s["speed"] = clamp_speed(s["speed"], HARD_SPEED_CAP, s["model"])
            if "speed" in fields:
                s["speed"] = clamp_speed(fields["speed"], HARD_SPEED_CAP, s.get("model", DEFAULT_MODEL))
            if "dash" in fields:
                # 0/None/"" = no finish. The loop mission is unchanged either
                # way - this only appends one more waypoint after it closes.
                try:
                    v = float(fields["dash"])
                except (TypeError, ValueError):
                    v = 0.0
                s["dash"] = max(0.0, min(v, 200.0))
            if "close_leg" in fields:
                s["close_leg"] = bool(fields["close_leg"])
            for flag in ("cam_front", "cam_nadir", "cam_chase"):
                if flag in fields:
                    s[flag] = bool(fields[flag])
            for key, lo, hi in (("chase_distance", 0.5, 10.0),
                                 ("chase_height", 0.1, 5.0),
                                 ("chase_degree", -360.0, 360.0)):
                if key in fields:
                    try:
                        s[key] = max(lo, min(float(fields[key]), hi))
                    except (TypeError, ValueError):
                        pass
            if "extra" in fields:
                # Per-slot env overrides, appended AFTER the recipe's extra so
                # they win (env A=1 A=2 keeps the last). Needed for A/B work
                # (e.g. WP_VSUS sweeps) without editing RECIPES. STRICTLY
                # token-validated - this string is interpolated into a
                # bash -c command line, so only bare KEY=VALUE survives.
                toks = str(fields["extra"]).split()
                ok_toks = [t for t in toks
                           if re.fullmatch(r"[A-Z][A-Z0-9_]*=[-A-Za-z0-9_.:]*", t)]
                s["extra"] = " ".join(ok_toks)
            return True, s

    def draft_set_cap(self, value):
        with self.lock:
            self.draft_cap = max(0.3, min(float(value), HARD_SPEED_CAP))
            return True, self.draft_cap

    def draft_set_terrain(self, kind):
        with self.lock:
            if kind not in terrain.TERRAIN_TYPES:
                return False, "unknown terrain %r - choices: %s" % (
                    kind, list(terrain.TERRAIN_TYPES))
            self.draft_terrain = kind
            return True, self.draft_terrain

    # ---- launch ----------------------------------------------------------
    def launch(self, slots=None, speed_cap=None, terrain_kind=None):
        # No body (or an empty one) launches whatever is currently in the
        # draft - i.e. exactly what the panel is showing right now. An
        # explicit body still works for direct automation that wants to skip
        # the draft entirely and specify a full config in one call.
        #
        # THE ASYNC-TEARDOWN RACE GATE. Deliberately OUTSIDE self.lock and
        # BEFORE anything else in this method: a previous run's phase can
        # already read "done"/"idle" while its processes are still being
        # reaped on another thread (see _teardown_done's own comment for the
        # exact contamination this produced - a bogus 10.4s PASS on a
        # mission whose real floor is 55+s). Waiting on the lock instead
        # would work too but would hold it for the whole wait, blocking
        # every other request (state polls, slot edits) for no reason - this
        # Event is normally already set, so the wait is a no-op almost every
        # time, and only actually blocks during the narrow window a launch
        # request genuinely races a teardown.
        if not self._teardown_done.wait(timeout=10.0):
            self._note("launch: a previous teardown did not confirm within "
                      "10s - proceeding anyway rather than deadlocking, but "
                      "this is worth investigating if it ever fires")
        with self.lock:
            if slots is None:
                slots = self.draft_slots
            if speed_cap is None:
                speed_cap = self.draft_cap
            if terrain_kind is None:
                terrain_kind = self.draft_terrain
            if terrain_kind not in terrain.TERRAIN_TYPES:
                return False, "unknown terrain %r" % terrain_kind
            # An empty fleet is refused HERE, not by pinning the last draft
            # slot as unremovable - that pin is what left dog 0 with no
            # delete button. "You cannot launch nothing" is a launch-time
            # constraint; "you may not empty the draft" never was one.
            if not slots:
                return False, ("no dogs in the fleet - add at least one slot "
                               "before launching")
            if self.phase in ("launching", "running"):
                return False, "a fleet is already active - stop it first"
            # TIME MACHINE GATE (operator-diagnosed): hourly backups on this
            # Mac start around :38, and the two same-wall-second multi-dog
            # stall kills landed at 14:38:09 and 16:44:09 (six minutes into
            # the 16:38 backup). A backup's I/O burst is exactly the kind of
            # host stall that applies 2 ms control forces for 18 ms. Refuse
            # to launch while one is in flight; `sudo tmutil disable` (or
            # `tmutil stopbackup`) is the operator-side fix for a session.
            try:
                tm = subprocess.run(["tmutil", "status"], capture_output=True,
                                     text=True, timeout=5).stdout
                if "Running = 1" in tm:
                    return False, ("Time Machine backup in progress - it has "
                                   "killed fleets before (I/O stall = 9x force "
                                   "impulse). Wait for it, or `tmutil "
                                   "stopbackup` / `sudo tmutil disable`.")
            except Exception:  # noqa: BLE001 - tmutil missing/slow never blocks
                pass
            if not (1 <= len(slots) <= 3):
                return False, "1 to 3 slots only"
            self.phase = "launching"
            self.run_id += 1
            try:
                os.makedirs(RUN_DIR, exist_ok=True)
                with open(os.path.join(RUN_DIR, "run_seq.txt"), "w") as f:
                    f.write(str(self.run_id))
            except Exception:  # noqa: BLE001 - numbering is not worth failing a launch
                pass
            self.log = []
            self.procs = []
            # STALE BRIDGE/CONTROLLER GATE: a bridge or controller left running
            # from an earlier manual test (no gz sim behind it, or no
            # controller behind it) can still hold a dog's UDP port pair -
            # neither side sets SO_REUSEADDR, on purpose, so a stale occupant
            # is detected rather than silently shared. This is what silently
            # corrupted an entire pronking speed-ladder sweep once: every run
            # looked like an identical, reproducible physics failure (frozen
            # roll/z) until the stale pid on port 9100 was found by hand. The
            # bridge now clears its own port on its own startup too
            # (cheetah_gazebo_bridge.py's _clear_stale_port) - this is the
            # second, redundant check, at the one place that knows the FULL
            # port list for the fleet about to launch, before ANY process for
            # this run exists yet.
            for i in range(len(slots)):
                for port in (9100 + 10 * i, 9101 + 10 * i):
                    try:
                        out = subprocess.run(["lsof", "-ti", "udp:%d" % port],
                                              capture_output=True, text=True,
                                              timeout=5).stdout
                        for pid_s in out.split():
                            pid = int(pid_s)
                            self._note("stale process pid %d held port %d "
                                       "(dog%d) from a previous run - killing "
                                       "it before launch" % (pid, port, i))
                            os.kill(pid, 9)
                    except Exception:  # noqa: BLE001 - lsof missing/slow never blocks
                        pass
            # STALE TAIL-TEXT REAPER GATE. shm_reaper.py --tail-text holds NO
            # network port at all - it is a pure file-tailing process - so the
            # port sweep above can never see it. Found by direct evidence: a
            # fleet's natural "done" path clears self.procs/self.phase to
            # idle IMMEDIATELY (inside the lock), then does terminate() ->
            # sleep(1) -> kill() on a background thread - so /api/state can
            # report "idle" a full second or more before the old tail-text
            # reaper (and possibly the old bridge/controller too) actually
            # receives its kill signal. A launch that lands in that window
            # races the old reaper: it re-opens whatever file now sits at
            # ctrl_%d.log on its next 0.2s poll (rather than holding one fd
            # for its whole life) and keeps appending the PREVIOUS run's text
            # into the NEW run's fresh log - old and new content simply
            # concatenate in file order, no visible corruption, just a wrong
            # verdict. Reproduced live: a trotRunning dash launched moments
            # after a galloping run's fall showed the galloping run's own
            # "[SCHED] gait changed 22 -> 4" tail before the new mission's own
            # "[nav] dash mission" line ever printed. Kill by COMMAND LINE
            # PATTERN, not port, and do it for every dog index about to
            # launch, same as the controller itself doing this on the wrong
            # SIM_INSTANCE would (this call is intentionally broader than
            # just this fleet's own indices, since a stale process from an
            # entirely different prior slot count could still be at index 0).
            for i in range(len(slots)):
                try:
                    out = subprocess.run(
                        ["pgrep", "-f", "shm_reaper.py.*--tail-text %d\\b" % i],
                        capture_output=True, text=True, timeout=5).stdout
                    for pid_s in out.split():
                        pid = int(pid_s)
                        self._note("stale tail-text reaper pid %d for dog%d "
                                   "from a previous run - killing it before "
                                   "launch" % (pid, i))
                        os.kill(pid, 9)
                except Exception:  # noqa: BLE001 - pgrep missing/slow never blocks
                    pass

        # Freeze the configuration NOW. Nothing below this point reads the
        # request again - a second call while running is refused above.
        locked = []
        for i, s in enumerate(slots):
            spec = s["mission"]
            kind = mission_kind(spec)
            # The fallback for a mission kind with no RECIPES entry (e.g.
            # "corner:" - a hand-tuned test primitive, never meant to carry
            # its own default gait/speed) MUST carry every key launch() reads
            # below, or a mission kind that is merely undocumented crashes
            # the whole launch request instead of just running with defaults
            # - exactly the KeyError('note') class of bug already documented
            # above for "outback"/"dash", now hit for real by "corner".
            recipe = RECIPES.get(kind, dict(gait=5, speed=2.5, extra="", note=""))
            gait = GAITS.get(s.get("gait", ""), recipe["gait"])
            gait_name = s.get("gait") if s.get("gait") in GAITS else next(
                (g for g, n in GAITS.items() if n == gait), str(gait))
            # Model cap is enforced HERE, not only in draft_set_slot - this is
            # the one path that cannot be bypassed by a direct /api/launch
            # call carrying its own "slots" body with an unclamped speed.
            speed = clamp_speed(s.get("speed", recipe["speed"]), speed_cap,
                                 s.get("model", DEFAULT_MODEL))
            # Slot-level overrides go AFTER the recipe's own extra: with
            # `env A=1 A=2 cmd` the later assignment wins, so a slot can
            # override a recipe knob (WP_VSUS etc.) for A/B work.
            # Bubble the panel's own "not this course's validated combo"
            # mismatch warning into the orchestration log too, at the one
            # moment it actually matters (about to launch) - the browser-only
            # version (app.js's renderSlots) is silent to anything driving
            # this via mission_runner.py/curl/another tab, and even in the
            # browser it is easy to miss sitting quietly on a slot card. This
            # is the same comparison app.js makes (recipe gait/speed vs the
            # slot's actual gait/speed), just server-side and logged, so a
            # mismatch is visible wherever this run's log is - live panel,
            # streamed mission_runner.py output, or the archived file.
            recipe_gait_name = next((g for g, n in GAITS.items() if n == recipe.get("gait")), None)
            if recipe_gait_name and gait_name != recipe_gait_name:
                self._note("dog%d: NOT this course's validated combo - running "
                           "%s @ %.2f, recipe is %s @ %.2f%s" %
                           (i, gait_name, speed, recipe_gait_name, recipe["speed"],
                            " (" + recipe["extra"] + ")" if recipe["extra"] else ""))
            elif isinstance(recipe.get("speed"), (int, float)) and abs(speed - recipe["speed"]) > 0.05:
                self._note("dog%d: NOT this course's validated combo - running "
                           "%s @ %.2f, recipe speed is %.2f" %
                           (i, gait_name, speed, recipe["speed"]))
            extra = (recipe["extra"] + " " + str(s.get("extra") or "")).strip()
            # Only ever passed to turn it OFF - mit_sim_main.cpp already
            # defaults it ON when the variable is absent, so the common case
            # adds nothing to the launch line.
            close_leg = bool(s.get("close_leg", DEFAULT_CLOSE_LEG))
            if not close_leg:
                extra = (extra + " WP_CLOSE_LEG=0").strip()
            dash = float(s.get("dash") or 0.0)
            if dash > 0:
                # Appended after whatever the recipe's own mission builds -
                # the loop runs exactly as validated, then keeps going straight
                # for `dash` more metres on the heading it closes on.
                extra = (extra + " WP_DASH=%.1f" % dash).strip()
            locked.append(dict(index=i, mission=spec, kind=kind, gait=gait,
                                gait_name=gait_name, speed=speed, extra=extra,
                                dash=dash, note=recipe["note"],
                                close_leg=close_leg,
                                cam_front=bool(s.get("cam_front", True)),
                                cam_nadir=bool(s.get("cam_nadir", True)),
                                cam_chase=bool(s.get("cam_chase", True)),
                                chase_distance=float(s.get("chase_distance", 3.0)),
                                chase_height=float(s.get("chase_height", 1.2)),
                                chase_degree=float(s.get("chase_degree", 90.0))))
        with self.lock:
            self.slots = locked
            self.status = [dict(index=s["index"], phase="pending", text="",
                                 t="", waypoints="") for s in locked]
            self.planned = {}
            self.positions = {}
            self._chase_stop = threading.Event()

        threading.Thread(target=self._run, args=(locked, terrain_kind),
                         daemon=True).start()
        return True, "launching %d dog(s) on %s terrain" % (len(locked), terrain_kind)

    def _run(self, locked, terrain_kind="flat"):
        try:
            os.makedirs(RUN_DIR, exist_ok=True)
            self._note("building fleet world: %s (terrain=%s)"
                        % (", ".join(s["mission"] for s in locked), terrain_kind))
            world_out = os.path.join(RUN_DIR, "fleet.sdf")
            cam_cfgs = [dict(front=s["cam_front"], nadir=s["cam_nadir"],
                              chase=s["cam_chase"], distance=s["chase_distance"],
                              height=s["chase_height"], degree=s["chase_degree"])
                        for s in locked]
            r = subprocess.run(
                [sys.executable, os.path.join(HERE, "fleet_world.py"),
                 "--terrain=%s" % terrain_kind,
                 "--cam_config=%s" % json.dumps(cam_cfgs),
                 os.path.join(GAZEBO_DIR, "worlds/go1_speedway.sdf"),
                 world_out] + [s["mission"] for s in locked],
                capture_output=True, text=True)
            if r.returncode != 0:
                self._note("world build FAILED: " + r.stderr[-500:])
                with self.lock:
                    self.phase = "error"
                return
            for line in r.stdout.strip().splitlines():
                self._note(line)

            # Planned paths, frozen now, in WORLD frame (local mission coords
            # + this slot's own spawn offset from the SAME layout() call
            # fleet_world.py used to place the robots - so a drawn path lines
            # up with where the robot actually is, not an independently
            # recomputed guess). Locked before anything moves, matching the
            # "config is frozen at launch" rule for everything else here.
            sys.path.insert(0, HERE)
            from fleet_world import layout  # noqa: E402
            placed = layout([s["mission"] for s in locked])
            planned = {}
            for s, (spec, north, east, bbox) in zip(locked, placed):
                # close_leg passed through so the DRAWN plan is the path the
                # robot actually flies - the whole point of this overlay
                # reading the same generator the controller does. Runs before
                # the dash append below for the same ordering reason
                # mit_sim_main.cpp closes before calling appendDash.
                pts = mission_waypoints(spec, close_leg=s.get("close_leg", True))
                if s.get("dash") and len(pts) >= 2:
                    # Mirrors WaypointNav::appendDash() exactly (same source
                    # of truth this whole overlay already leans on) so the
                    # dim planned line shows the finish before the dog ever
                    # runs it, not just after, in the flown trail. TWO points,
                    # not one: an explicit return to wp0 (closing the shape
                    # for real - "it never went back to the first waypoint"),
                    # then the dash outward from there. Heading for the final
                    # leg is last-waypoint -> wp0 (the course's own closing
                    # tangent), computed before appending wp0 since it's the
                    # same two points either way - see the C++ comment for why
                    # not the raw final-leg vector (a star tip's own heading
                    # shoots the dash out at the tip's oblique angle instead
                    # of continuing the shape).
                    (n2, e2), (n0, e0) = pts[-1], pts[0]
                    dn, de = n0 - n2, e0 - e2
                    length = math.hypot(dn, de)
                    if length > 1e-3:
                        dn, de = dn / length, de / length
                        dash = float(s["dash"])
                        pts = pts + [(n0, e0), (n0 + dn * dash, e0 + de * dash)]
                if len(pts) == 1:
                    # A standalone dash is ONE waypoint (the whole mission,
                    # not a finish appended to a loop), so it never enters
                    # the appendDash overlay above. But a single point is
                    # nothing to draw a line to - the panel's canvas only
                    # renders a planned track when it has >1 point - so
                    # anchor it at the local origin the dog actually starts
                    # from, same as the real controller's own path (see the
                    # BodyPathPlanner comment: "the path must start where
                    # the robot is").
                    pts = [(0.0, 0.0)] + pts
                planned[s["index"]] = [[e + east, n + north] for (n, e) in pts]
            with self.lock:
                self.planned = planned

            # STRAGGLER GUARD. A leftover `gz sim` from a previous launch shares
            # this fixed PARTITION and keeps publishing its OWN "go1_0" etc. on
            # the same pose topic - measured consequence: two zombie sims left
            # a trail buffer alternating between two frozen points, which
            # rendered as a scrambled scribble that looked like a tracking bug
            # and was not one. Refuse to proceed until nothing is left.
            for i in range(20):
                stale = subprocess.run(
                    ["pgrep", "-f", "gz[ ]sim -s -r"],
                    capture_output=True, text=True).stdout.split()
                if not stale:
                    break
                for pid in stale:
                    subprocess.run(["kill", "-9", pid], capture_output=True)
                time.sleep(0.5)
            else:
                self._note("could not clear a stale gz sim - aborting launch")
                with self.lock:
                    self.phase = "error"
                return

            env = os.environ.copy()
            env["GZ_SIM_RESOURCE_PATH"] = "%s/unitree_ros/robots:%s/models:%s" % (
                GAZEBO_DIR, GAZEBO_DIR, OPMODELS)
            env["GZ_PARTITION"] = PARTITION
            env["PATH"] = "/opt/homebrew/bin:" + env.get("PATH", "")

            self._note("starting Gazebo HEADLESS (no GUI, no marker traffic - "
                       "this page renders the fleet itself)")
            archive_log(os.path.join(RUN_DIR, "gz.log"), self.run_id - 1)
            gz_log = open(os.path.join(RUN_DIR, "gz.log"), "w")
            p = subprocess.Popen(["gz", "sim", "-s", "-r", world_out],
                                  cwd=GAZEBO_DIR, env=env, stdout=gz_log,
                                  stderr=subprocess.STDOUT)
            self.procs.append(p)

            self._note("waiting for %d dog(s) to advertise sensors" % len(locked))
            deadline = time.time() + 30
            ready = set()
            while time.time() < deadline and len(ready) < len(locked):
                topics = subprocess.run(["gz", "topic", "-l"], env=env,
                                         capture_output=True, text=True,
                                         timeout=5).stdout
                for s in locked:
                    if "/go1_%d/imu" % s["index"] in topics:
                        ready.add(s["index"])
                time.sleep(1)
            if len(ready) < len(locked):
                self._note("only %d/%d dogs came up - continuing anyway"
                            % (len(ready), len(locked)))

            self._name_to_index = {"go1_%d" % s["index"]: s["index"] for s in locked}
            self._subscribe_pose(env)
            self._subscribe_cameras(locked)
            threading.Thread(target=self._follow_chase_cams, args=(locked,),
                              daemon=True).start()

            for s in locked:
                i = s["index"]
                name = "go1_%d" % i
                senv = env.copy()
                senv["SIM_INSTANCE"] = str(i)
                senv["SIM_MODEL"] = name
                archive_log(os.path.join(RUN_DIR, "bridge_%d.log" % i), self.run_id - 1)
                blog = open(os.path.join(RUN_DIR, "bridge_%d.log" % i), "w")
                bp = subprocess.Popen(
                    [PYBIN, "-u", "cheetah_gazebo_bridge.py"],
                    cwd=GAZEBO_DIR, env=dict(senv, BRIDGE_CONV="mit"),
                    stdout=blog, stderr=subprocess.STDOUT)
                self.procs.append(bp)
                self._note("dog%d bridge up (%s, ports %d/%d)"
                            % (i, name, 9100 + 10 * i, 9101 + 10 * i))

            time.sleep(2)

            with self.lock:
                self.phase = "running"
                self.started_at = time.time()

            for s in locked:
                i = s["index"]
                cenv = env.copy()
                cenv["SIM_INSTANCE"] = str(i)
                # STAGGER THE RAMP ACROSS THE FLEET. Every dog used a fixed
                # 4 s delay, so N dogs launched together reached commanded
                # cruise in the SAME instant - and gz is ONE process serving
                # all of them, so that instant is the fleet's peak physics
                # demand. Measured 2026-08-24 18:28: three dogs, three
                # different courses, three different waypoints, all down at
                # t=12.4-12.5 s of their own clocks, with every per-dog
                # instrument clean (loop max 3.0 ms, 0 over-4 ms, bridges at
                # a steady 500 cmd/s). Per-dog instruments CANNOT see a
                # shared-simulator stall: the control loops are wall-clock
                # timed, so when physics falls behind, forces computed for a
                # 2 ms step get integrated over a longer one and every dog
                # is hit at once. Staggering by 5 s per slot spreads that
                # peak; a solo dog is unaffected (i=0 keeps the validated 4 s).
                delay_s = 4 + 5 * i
                # This `timeout` is a hard safety net (kill a genuinely hung
                # controller), not a per-mission budget - it used to be 240s,
                # sized for star/oval/atom/dash, which all finish in well
                # under that. The Lissajous missions do not: 5:7 is a 558m
                # course that legitimately needs 400-500s, and 240 silently
                # SIGKILLed it two-thirds of the way through with no crash
                # report, no [FALL], nothing - it looked exactly like a
                # segfault (a genuinely confusing failure mode: `pgrep
                # mit_ctrl_sim` simply returns nothing, same as a real crash
                # would). Raised to 900s, comfortable headroom for even the
                # much longer 11:9 ratio, while still bounding a genuine hang.
                # Same bearing fleet_world.py already used to aim this dog's
                # SPAWN pose (mission_spawn_yaw_rad) - the controller needs
                # it too, to correct its own heading datum for the same
                # reason. See mit_sim_main.cpp's "bearing" comment: without
                # this, nav silently assumes spawn-heading-equals-true-north,
                # which stopped being true the moment spawn heading became
                # mission-specific instead of a universal fixed north.
                spawn_bearing_deg = math.degrees(mission_opening_bearing_rad(s["mission"]))
                cmd = (
                    "env DYLD_LIBRARY_PATH=. SIM_RUN_ID=%d SIM_INSTANCE=%d SIM_GAIT=%d SIM_VX=%s "
                    "SIM_VX_DELAY_S=%d SIM_VX_RAMP_S=8 WP_MISSION=%s WP_PLANNER=1 "
                    "WP_MAX_YAWRATE=1.2 WP_SPAWN_BEARING_DEG=%.4f %s timeout 900 ./mit_ctrl_sim 127.0.0.1 "
                    "stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml"
                    % (self.run_id, i, s["gait"], s["speed"], delay_s, s["mission"],
                       spawn_bearing_deg, s["extra"])
                )
                ctrl_log_path = os.path.join(RUN_DIR, "ctrl_%d.log" % i)
                archive_log(ctrl_log_path, self.run_id - 1)
                clog = open(ctrl_log_path, "w")
                cp = subprocess.Popen(["bash", "-c", cmd], cwd=HOST_RUN,
                                       env=cenv, stdout=clog, stderr=subprocess.STDOUT)
                self.procs.append(cp)
                # THE PRINTF REPLACEMENT'S OTHER HALF. RobotRunner.cpp and
                # mit_sim_main.cpp no longer write their debug/event lines
                # to stdout at all (see ShmTrace.h) - they go into a SHM
                # text ring instead, cheap enough to run at any rate with
                # no stdio locking/buffering/fflush cost. Without this
                # bridge tailing that ring back into ctrl_%d.log, every
                # regex/substring match this file already does against
                # that log (waypoint progress, gait changes, "[FALL]",
                # "[mission] RESULT", "MISSION COMPLETE", ...) would see
                # NOTHING from those two files ever again. Started right
                # after clog's truncating open above (never before it -
                # the bridge only APPENDS, so it must not race a
                # concurrent truncation of the same path), spawned
                # unconditionally alongside the controller since the text
                # ring is the ONLY place those lines exist now, not an
                # optional extra. Torn down with every other process on
                # stop/done (self.procs is killed as one list).
                tbp = subprocess.Popen(
                    ["python3", os.path.join(GAZEBO_DIR, "shm_reaper.py"),
                     "--tail-text", str(i), "--append-to", ctrl_log_path,
                     "--poll", "0.2",
                     # ONE RUN NUMBER EVERYWHERE. The reaper follows only
                     # the ring stamped with THIS run id, so a previous
                     # run's segment - which outlives its process, since
                     # ShmTrace only unlinks at startup - can never be
                     # replayed into this run's fresh log. It did exactly
                     # that earlier tonight and produced a false PASS.
                     "--expect-run-id", str(self.run_id)])
                self.procs.append(tbp)
                dash_note = (" +dash %.0fm" % s["dash"]) if s.get("dash") else ""
                self._note("dog%d LOCKED: %s gait=%s cmd=%.2f m/s (cap %.2f) %s%s"
                            % (i, s["mission"], s["gait_name"], s["speed"],
                               HARD_SPEED_CAP, s["extra"], dash_note))

            self._start_poller(locked)

        except Exception as e:  # noqa: BLE001 - report, don't crash the server
            self._note("launch error: %r" % e)
            with self.lock:
                self.phase = "error"
        except SystemExit as e:
            # A helper (e.g. mission_waypoints on an unrecognised mission
            # kind) raising SystemExit instead of a plain Exception used to
            # kill this thread SILENTLY - threading swallows SystemExit at
            # the bootstrap level with no traceback, so phase stayed
            # "launching" forever with no error logged anywhere. Caught here
            # so every launch failure is visible, not just the ones that
            # happen to raise Exception.
            self._note("launch error (SystemExit): %s" % e)
            with self.lock:
                self.phase = "error"

    # ---- Conductor's OWN rendering: subscribe to world pose ourselves -----
    def _subscribe_pose(self, env):
        """One subscription covers every dog - Gazebo publishes ALL models'
        poses in a single message on this topic (trail_daemon.py already
        relied on that, filtering to one name; here we keep every name we
        placed). Runs for the life of the fleet; gz.transport13 fires the
        callback on its own thread, so this just has to register once."""
        node = _gz_transport.Node()
        self._gz_node = node  # keep alive - a gc'd Node drops the subscription
        SEG_MIN = 0.15        # m between recorded trail points, matches
                               # trail_daemon's own decimation logic
        # ROLLING WINDOW, and it was too short. Measured cause of a real
        # report: Lissajous 11:9 (914.6m path) genuinely flew all 606
        # waypoints in order (confirmed via the ctrl log - zero skipped
        # indices) but the operator watched it on screen and saw several
        # early legs of the pattern just missing. At SEG_MIN=0.15m, 914.6m
        # needs a MINIMUM of ~6100 trail points before accounting for extra
        # segments at corners - the old 4000 cap silently evicted the
        # OLDEST ~2000+ points (the early loops) well before the mission
        # finished. The navigation was never wrong; only the live trail
        # (and this snapshot, and any report built from it) was truncated.
        # Sized for headroom over the longest current mission, not just to
        # patch this one course: 20000 covers a ~3000m path at this
        # resolution, comfortably above anything in the mission catalog
        # today (Lissajous 11:9 is the longest at 914.6m).
        TRAIL_MAX = 20000

        def on_pose(msg):
            now = time.time()
            with self.lock:
                for p in msg.pose:
                    idx = self._name_to_index.get(p.name)
                    if idx is None:
                        continue
                    x, y, z = p.position.x, p.position.y, p.position.z
                    # orientation is a quaternion, not Euler - yaw about world Z
                    o = p.orientation
                    yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))
                    # Ground speed from consecutive fixes, not a field Pose_V
                    # carries - lightly EMA'd (alpha 0.3) so the per-dog
                    # readout does not flicker at whatever rate gz publishes,
                    # while staying responsive enough to show a real change.
                    speed = 0.0
                    prev_t = self._last_pose.get(idx)
                    cur = self.positions.get(idx)
                    if prev_t is not None:
                        px, py, pt = prev_t
                        dt_s = now - pt
                        if 1e-3 < dt_s < 1.0:  # skip the first fix after a launch/gap
                            raw = math.hypot(x - px, y - py) / dt_s
                            prev_speed = cur["speed"] if cur else raw
                            speed = 0.3 * raw + 0.7 * prev_speed
                    self._last_pose[idx] = (x, y, now)
                    trail = cur["trail"] if cur else []
                    if not trail or ((x - trail[-1][0]) ** 2
                                      + (y - trail[-1][1]) ** 2) >= SEG_MIN ** 2:
                        trail = (trail + [[round(x, 3), round(y, 3)]])[-TRAIL_MAX:]
                    self.positions[idx] = dict(x=round(x, 3), y=round(y, 3),
                                                z=round(z, 3), yaw=round(yaw, 3),
                                                speed=round(speed, 2), trail=trail)

        ok = node.subscribe(Pose_V, "/world/%s/dynamic_pose/info" % WORLD, on_pose)
        self._note("pose subscriber %s" % ("up" if ok else "FAILED to register"))

    def _follow_chase_cams(self, locked):
        """Live chase-camera positioning. fleet_world.make_chase_cam_model
        spawns each enabled dog's chase_cam as a FREE-FLOATING model rather
        than bolting it to the body specifically so this loop can move it -
        a body-mounted sensor's pose is baked into the SDF at world-build
        time and cannot change without a full relaunch, which is exactly
        the "slot settings don't react live" gap this closes.

        Every ~100ms, for each dog whose chase cam was enabled AT LAUNCH
        (cam_chase off at launch means no model was ever spawned - same
        launch-time-only constraint _subscribe_cameras already documents
        for the mute case, and for the same reason: there is no service to
        add a whole new model to a running world short of a relaunch):
        read that dog's live world pose from self.positions (already kept
        current for the trail overlay, so this adds no new subscription)
        and the CURRENT distance/height/degree from the DRAFT slot (not
        the locked one - reading the draft live, the same pattern
        _subscribe_cameras' on/off mute already uses, is what makes a
        slider drag mid-run visible within one tick instead of on the next
        launch), then teleport the camera model via the world's set_pose
        service.

        Deliberately ignores the dog's own roll/pitch - only yaw composes
        the body-local offset into world frame. A truly rigid attachment
        would also inherit gait-cycle bob and any tilt, but a camera that
        pitches and rolls with a trotting dog is closer to nauseating than
        useful; stabilising against everything but heading is the common
        chase-camera convention for exactly that reason, not a limitation
        of this approach.

        request() is BLOCKING (confirmed against this gz.transport13 build -
        no async variant is exposed), so this runs on its own Node and its
        own thread, never sharing one with the pose/camera subscribers
        whose callbacks must never stall.
        """
        node = _gz_transport.Node()
        stop_event = self._chase_stop
        indices = [s["index"] for s in locked if s["cam_chase"]]
        if not indices:
            return
        service = "/world/%s/set_pose" % WORLD
        while not stop_event.is_set():
            for i in indices:
                with self.lock:
                    pos = self.positions.get(i)
                    d = self.draft_slots[i] if i < len(self.draft_slots) else {}
                if pos is None:
                    continue
                distance = float(d.get("chase_distance", 3.0))
                height = float(d.get("chase_height", 1.2))
                degree = float(d.get("chase_degree", 90.0))
                rad = math.radians(degree)
                lox = -distance * math.cos(rad)   # 0 deg = behind (-X, body fwd +X)
                loy = distance * math.sin(rad)    # 90 deg = left (+Y)
                yaw = pos["yaw"]
                # Rotate the body-local offset into world frame by the
                # dog's OWN current heading - what a rigid attachment gives
                # for free, done explicitly since this model is free-floating.
                wx = pos["x"] + lox * math.cos(yaw) - loy * math.sin(yaw)
                wy = pos["y"] + lox * math.sin(yaw) + loy * math.cos(yaw)
                wz = pos["z"] + height
                look_yaw = yaw + math.atan2(-loy, -lox)   # look back at the dog
                pitch = math.atan2(height, max(0.05, distance))
                cy, sy = math.cos(look_yaw * 0.5), math.sin(look_yaw * 0.5)
                cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
                req = _GzPose()
                req.name = "go1_%d_chasecam" % i
                req.position.x = wx
                req.position.y = wy
                req.position.z = wz
                # roll=0, pitch, yaw -> quaternion (standard ZYX Euler
                # composition, matches SDF's own <pose> convention)
                req.orientation.w = cp * cy
                req.orientation.x = -sp * sy
                req.orientation.y = sp * cy
                req.orientation.z = cp * sy
                try:
                    node.request(service, req, _GzPose, _GzBoolean, 100)
                except Exception:  # noqa: BLE001 - one missed tick is invisible; never worth killing the loop over
                    pass
            time.sleep(0.1)

    def _subscribe_cameras(self, locked):
        """Front / nadir / chase feed per dog, Chuck-UI style (.ahrs-cam
        tiles) - only the ones each slot's checkboxes actually enabled.
        A camera the world never spawned (see fleet_world.apply_camera_config)
        has no topic to subscribe to, so this must match that exactly or the
        subscribe call just silently gets nothing; both read the same
        cam_front/cam_nadir/cam_chase slot fields. One gz.transport13
        subscription per enabled camera topic, each decoding the latest
        frame to a JPEG data URL. Deliberate choices:
          - keep only the LATEST frame, never a queue - this is a live tile,
            not a recording, and a queue would only let the browser fall
            behind and then catch up on stale frames;
          - encode on receipt at the sensor's own 10 Hz rather than on
            request, so a slow poller never blocks the transport callback.
        """
        for s in locked:
            i = s["index"]
            for cam in CAMERAS:
                if not s.get(CAM_FLAG_KEYS[cam], True):
                    continue
                node = _gz_transport.Node()
                self._gz_cam_nodes.append(node)

                def on_image(msg, idx=i, camname=cam):
                    # DYNAMIC per-frame gate: the checkbox edits the DRAFT
                    # slot, and this reads it live, so unchecking a camera
                    # mid-run stops (and un-publishes) its stream instantly
                    # and re-checking resumes it. Launch-time flags still
                    # decide whether the sensor exists in the world at all -
                    # a camera disabled at launch has no topic and can never
                    # be re-enabled mid-run, only mid-run muted/unmuted.
                    with self.lock:
                        d = self.draft_slots[idx] if idx < len(self.draft_slots) else {}
                        if not d.get(CAM_FLAG_KEYS[camname], True):
                            self.cameras.get(idx, {}).pop(camname, None)
                            return
                    try:
                        img = _PILImage.frombytes(
                            "RGB", (msg.width, msg.height), msg.data)
                        buf = io.BytesIO()
                        img.save(buf, format="JPEG", quality=60)
                        url = "data:image/jpeg;base64," + base64.b64encode(
                            buf.getvalue()).decode("ascii")
                    except Exception as e:  # noqa: BLE001 - one bad frame must
                        print("[camera] %s dog%d error: %r" % (camname, idx, e),
                              flush=True)
                        return                                # not kill the feed
                    with self.lock:
                        self.cameras.setdefault(idx, {})[camname] = url

                topic = "/go1_%d/%s" % (i, cam)
                ok = node.subscribe(_GzImage, topic, on_image)
                if not ok:
                    self._note("camera subscribe FAILED: %s" % topic)

    # ---- live status, parsed from the controller logs --------------------
    def _start_poller(self, locked):
        def poll():
            done = set()
            seen_bytes = {}   # index -> bytes of ctrl_%d.log already scanned for EVENT_PATTERNS
            while True:
                with self.lock:
                    if self.phase not in ("running",):
                        return
                for s in locked:
                    i = s["index"]
                    if i in done:
                        continue
                    path = os.path.join(RUN_DIR, "ctrl_%d.log" % i)
                    try:
                        with open(path, errors="ignore") as f:
                            text = f.read()
                    except FileNotFoundError:
                        continue
                    # Incremental scan: only NEW complete lines since last
                    # tick, so a discrete event is logged once, not re-noted
                    # every poll while it sits in the file. A trailing
                    # partial line (still being written) is left for the
                    # next tick rather than matched half-formed.
                    start = seen_bytes.get(i, 0)
                    new_text = text[start:]
                    cut = new_text.rfind("\n")
                    if cut >= 0:
                        for line in new_text[:cut].splitlines():
                            for pat, fmt in EVENT_PATTERNS:
                                mo = pat.search(line)
                                if mo:
                                    self._note("dog%d: %s" % (i, fmt(mo)))
                                    break
                        seen_bytes[i] = start + cut + 1
                    st = dict(index=i, phase="running", text="", t="", waypoints="")
                    m = re.findall(r"\[nav\] wp(\d+)/(\d+).*?d=([\d.]+).*?v=([\d.]+)",
                                    text)
                    if m:
                        wp, tot, d, v = m[-1]
                        wp, tot = int(wp), int(tot)
                        # The PERIODIC status line above is 0-indexed and
                        # stops updating once the dog reaches its last
                        # waypoint - the arrival itself prints as a
                        # DIFFERENT line ("[nav] reached wpN ..."), which
                        # this regex does not match. Left alone, the last
                        # few seconds of every mission display "wp N-1/N"
                        # even after the Nth (final) waypoint has genuinely
                        # been captured - not a skip, just this counter
                        # freezing one index early (confirmed live: raw
                        # logs show every "reached wpNN" in sequence with
                        # no gaps, and "[mission] RESULT" always reports
                        # the full N/N). Catch the arrival line too and
                        # take whichever implies MORE progress - "reached
                        # wpK" (0-indexed) means K+1 waypoints are now
                        # done, matching the 1-based count the final
                        # RESULT/MISSION COMPLETE lines already use.
                        rm = re.findall(r"\[nav\] reached wp(\d+)", text)
                        if rm:
                            wp = max(wp, int(rm[-1]) + 1)
                        st["waypoints"] = "%d/%d" % (wp, tot)
                        st["text"] = "d=%sm v=%sm/s" % (d, v)
                    if "[mission] RESULT" in text:
                        # A dog is DONE at its JUDGE line, not at "MISSION
                        # COMPLETE": the judge prints ~9 s later, after the
                        # end-of-mission settle + lie-down. Keying done on
                        # MISSION COMPLETE made the teardown-on-done kill
                        # single-dog runs (and the last fleet finisher)
                        # mid-lie-down, truncating their verdict - two
                        # perfect 114.2 s star guards were reported as
                        # failures by exactly this.
                        tm = re.search(r"MISSION COMPLETE t=([\d.]+)s", text)
                        st["phase"] = "complete"
                        st["t"] = tm.group(1) + "s" if tm else ""
                        done.add(i)
                        self._note("dog%d COMPLETE t=%s" % (i, st["t"]))
                    elif "[FALL]" in text:
                        # Checked BEFORE "MISSION COMPLETE", not after: a dog
                        # can complete its loop+dash, print MISSION COMPLETE,
                        # and THEN fall during the final settle/lie-down -
                        # before ever printing a judge RESULT. `text` is the
                        # log's full content re-read every tick, so once
                        # MISSION COMPLETE appears it is present on every
                        # future tick too; checking it first would classify
                        # that dog as "finishing" FOREVER; done never reaches
                        # len(locked), and the whole fleet's phase is stuck
                        # at "running" permanently even though the dog's
                        # controller process has already exited. Caught live:
                        # two dogs judged PASS, a third fell after its own
                        # MISSION COMPLETE and the fleet phase never left
                        # "running" - only a manual /api/stop cleared it.
                        st["phase"] = "fell"
                        done.add(i)
                        self._note("dog%d FELL" % i)
                        # Grab the SHM trace NOW, before this dog's next
                        # launch reuses SIM_INSTANCE=i and shm_unlink()s the
                        # very segment that has the fall we want to keep -
                        # this is the launcher-as-reaper path (no separate
                        # `shm_reaper.py --watch` process needed for the
                        # conductor's own fleet runs).
                        try:
                            path = shm_reaper.dump_snapshot(i, "FALL", run_id=self.run_id)
                            if path:
                                self._note("dog%d: shm trace archived -> %s" % (i, path))
                        except Exception as e:  # noqa: BLE001 - archiving must
                            self._note("dog%d: shm archive failed: %r" % (i, e))
                            # never mask the real FALL event above it
                    elif "MISSION COMPLETE" in text:
                        # Loop+dash finished; lie-down/judge still running -
                        # show it, but do NOT count it done yet.
                        tm = re.search(r"MISSION COMPLETE t=([\d.]+)s", text)
                        st["phase"] = "finishing"
                        st["t"] = tm.group(1) + "s" if tm else ""
                    with self.lock:
                        for j, old in enumerate(self.status):
                            if old["index"] == i:
                                self.status[j] = st
                if len(done) == len(locked):
                    with self.lock:
                        self.phase = "done"
                        procs = list(self.procs)
                        self.procs = []
                        # Clear the SUBSCRIPTION state too, exactly as stop()
                        # does - killing the processes but keeping _gz_node
                        # left the next launch holding a stale subscription
                        # to a dead gz, so its pose feed never came up: the
                        # canvas froze on the previous run's final positions
                        # while the new controllers ran blind ("the interface
                        # was out of sync" - it was the server, not the
                        # browser).
                        self._name_to_index = {}
                        self._last_pose = {}
                        self._gz_node = None
                        self._gz_cam_nodes = []
                        self._teardown_done.clear()
                    self._note("fleet run complete")
                    # KILL THE SIM ON DONE. Leaving gz alive "for the next
                    # launch" left it idling at ~a full core simulating an
                    # empty world - which held ambient load at 3.8-4.3 (and
                    # got blamed on the operator's web browser), and once
                    # stacked a SECOND physics engine under a new launch,
                    # producing upside-down dogs and below-floor estimates
                    # that read exactly like a code regression. A finished
                    # run owns nothing; tear it all down - and CONFIRM it is
                    # actually down (_reap_and_confirm waits for real exit)
                    # before the next launch is allowed to proceed at all;
                    # see the comment on _teardown_done for the race this
                    # closes.
                    self._reap_and_confirm(procs)
                    self._note("sim torn down (gz + bridges + controllers)")
                    return
                time.sleep(1)
        threading.Thread(target=poll, daemon=True).start()

    # ---- teardown ----------------------------------------------------------
    def _reap_and_confirm(self, procs):
        """terminate() -> brief grace -> kill(), then ACTUALLY WAIT for each
        process to exit (reaping it) before declaring teardown done - not
        "sent a signal and hoped". Sets self._teardown_done only once every
        process is confirmed gone (or definitively unreapable), which is
        what makes it safe for launch() to treat that Event as a real
        guarantee rather than an optimistic guess. Bounded per-process so one
        stuck process cannot hang teardown forever - it is logged and the
        Event is still set, matching the file's own stance elsewhere that a
        detected-but-unkillable process is a hardware/OS problem to surface,
        not something to spin on."""
        for p in procs:
            try:
                p.terminate()
            except Exception:  # noqa: BLE001
                pass
        deadline = time.time() + 1.0
        for p in procs:
            remaining = max(0.0, deadline - time.time())
            try:
                p.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                pass
            except Exception:  # noqa: BLE001
                pass
        stuck = []
        for p in procs:
            if p.poll() is None:
                try:
                    p.kill()
                except Exception:  # noqa: BLE001
                    pass
        for p in procs:
            if p.poll() is None:
                try:
                    p.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    stuck.append(p.pid)
                except Exception:  # noqa: BLE001
                    pass
        if stuck:
            self._note("teardown: pid(s) %r survived SIGKILL+wait - "
                       "OS/zombie issue, not retrying further" % stuck)
        self._teardown_done.set()

    def stop(self):
        with self.lock:
            procs = list(self.procs)
            self.procs = []
            self.phase = "idle"
            self._name_to_index = {}
            self._last_pose = {}       # a stale (x,y,t) would spike the next launch's speed EMA
            self._gz_node = None       # drops the pose subscription
            self._gz_cam_nodes = []    # drops every camera subscription
            self.cameras = {}
            if self._chase_stop is not None:
                self._chase_stop.set()   # tells _follow_chase_cams to exit its loop
            self._teardown_done.clear()
        self._reap_and_confirm(procs)
        self._note("fleet stopped")


FLEET = Fleet()


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=os.path.join(HERE, "static"), **kw)

    def log_message(self, fmt, *args):
        pass  # the on-page log is the log that matters here

    def guess_type(self, path):
        # SimpleHTTPRequestHandler's default Content-Type carries no
        # charset ("text/html", "text/javascript" - confirmed via `curl
        # -I`). app.js's " · RUN <n>" separator is correctly UTF-8 on
        # disk (bytes 0xc2 0xb7), and index.html has no <meta charset>
        # either - with no encoding declared ANYWHERE (header or markup),
        # the browser has to guess, and guessed Latin-1 for these mostly-
        # ASCII files: each UTF-8 middle-dot byte pair got read back as
        # two separate Latin-1 characters (0xc2 -> "Â", 0xb7 -> "·"),
        # rendering literally as "Â·" in the page. Declaring charset=utf-8
        # on every text response removes the guesswork instead of only
        # patching the one string that happened to get noticed.
        ctype = super().guess_type(path)
        if ctype.startswith("text/") and "charset=" not in ctype:
            ctype += "; charset=utf-8"
        return ctype

    def end_headers(self):
        # This is a local, single-operator dev panel whose static/*.js and
        # *.css get edited and reloaded constantly - there is no caching
        # benefit worth the cost of ever serving stale code. Without this,
        # SimpleHTTPRequestHandler's default (Last-Modified only, no
        # Cache-Control) let a browser's heuristic freshness rules serve a
        # cached app.js indefinitely - a hard reload (Cmd+Shift+R) did not
        # even reliably bypass it. A page open from before a fix landed
        # would keep running the OLD, already-buggy code with no visible
        # sign anything was stale - exactly the kind of silent staleness
        # this port's harness rules already warn against for logs.
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/state":
            return self._json(FLEET.snapshot())
        # /docs (no extension) -> the static docs.html file. Explicit rather
        # than relying on SimpleHTTPRequestHandler's directory-index rules,
        # which only kick in for a trailing-slash directory path - this way
        # /docs works exactly like typing it looks like it should.
        if self.path in ("/docs", "/docs/"):
            self.path = "/docs.html"
            return super().do_GET()
        m = re.match(r"^/api/logs/(\d+)(?:\?(.*))?$", self.path)
        if m:
            return self._logs(int(m.group(1)), m.group(2) or "")
        return super().do_GET()

    def _logs(self, i, query):
        """Raw log for dog `i` - the FULL text mit_ctrl_sim/the bridge wrote,
        not just the curated one-line-per-event orchestration log /api/state
        already carries. ?kind=ctrl|bridge (default ctrl) picks which
        process; ?tail=<n> returns only the last n lines (default 500 - full
        logs run to thousands of lines on a long mission, and most uses want
        "what just happened", not the whole file); ?full=1 overrides tail
        and returns everything."""
        params = urllib.parse.parse_qs(query)
        kind = params.get("kind", ["ctrl"])[0]
        if kind not in ("ctrl", "bridge"):
            return self._json({"ok": False, "error": "kind must be ctrl or bridge"}, 400)
        path = os.path.join(RUN_DIR, "%s_%d.log" % (kind, i))
        try:
            with open(path, errors="ignore") as f:
                lines = f.readlines()
        except FileNotFoundError:
            return self._json({"ok": False, "error": "no log for dog %d (not launched yet, "
                                "or run_dir was cleared)" % i}, 404)
        full = params.get("full", ["0"])[0] == "1"
        if not full:
            tail = int(params.get("tail", ["500"])[0])
            lines = lines[-tail:]
        return self._json({"ok": True, "index": i, "kind": kind,
                            "lines": len(lines), "text": "".join(lines)})

    def _body(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        return json.loads(raw or b"{}")

    def do_POST(self):
        try:
            body = self._body()
        except json.JSONDecodeError:
            return self._json({"ok": False, "error": "bad json"}, 400)

        m = re.match(r"^/api/slots/(\d+)$", self.path)
        if self.path == "/api/launch":
            ok, msg = FLEET.launch(body.get("slots"), body.get("speed_cap"),
                                    body.get("terrain"))
            return self._json({"ok": ok, "message": msg})
        if self.path == "/api/stop":
            FLEET.stop()
            return self._json({"ok": True})
        if self.path == "/api/slots/add":
            ok, res = FLEET.draft_add_slot()
            return self._json({"ok": ok, "slots": res if ok else None,
                                "message": None if ok else res})
        if m:
            ok, res = FLEET.draft_set_slot(int(m.group(1)), body)
            return self._json({"ok": ok, "slot": res if ok else None,
                                "message": None if ok else res})
        if self.path == "/api/speed_cap":
            ok, res = FLEET.draft_set_cap(body.get("value", HARD_SPEED_CAP))
            return self._json({"ok": ok, "speed_cap": res})
        if self.path == "/api/terrain":
            ok, res = FLEET.draft_set_terrain(body.get("value", "flat"))
            return self._json({"ok": ok, "terrain": res if ok else None,
                                "message": None if ok else res})
        return self._json({"ok": False, "error": "no such route"}, 404)

    def do_DELETE(self):
        # Checked BEFORE the indexed route so "/api/slots" cannot be read as
        # a malformed "/api/slots/{i}".
        if self.path == "/api/slots":
            ok, res = FLEET.draft_clear_slots()
            return self._json({"ok": ok, "slots": res if ok else None,
                                "message": None if ok else res})
        m = re.match(r"^/api/slots/(\d+)$", self.path)
        if m:
            ok, res = FLEET.draft_remove_slot(int(m.group(1)))
            return self._json({"ok": ok, "slots": res if ok else None,
                                "message": None if ok else res})
        return self._json({"ok": False, "error": "no such route"}, 404)


def main():
    os.makedirs(RUN_DIR, exist_ok=True)
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", PORT), Handler) as httpd:
        print("Conductor on http://127.0.0.1:%d" % PORT, flush=True)
        httpd.serve_forever()


if __name__ == "__main__":
    main()
