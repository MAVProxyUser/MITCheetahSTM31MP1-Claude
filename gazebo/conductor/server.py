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
import signal
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request

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
# ...AND GZ_IP IS NOT ENOUGH, which is what OPEN-21/22 turned out to be.
# GZ_IP only sets the address a participant ADVERTISES. DISCOVERY itself
# still multicasts to 239.255.0.7, and this host's own routing table sends
# 224.0.0/4 out over en0/en1 - the physical interfaces - never loopback:
#
#   $ netstat -rn -f inet | grep 224
#   224.0.0/4   link#20  UmCS   en0 !
#   224.0.0/4   link#17  UmCSI  en1 !
#
# So every discovery packet in a single-host simulation was leaving the
# machine's real network interface, and any moment that path is unhealthy -
# a flapping Wi-Fi link, a VPN toggle, a DHCP renewal, an interface asleep -
# discovery silently fails. It fails SILENTLY because gz.log records nothing
# when the send merely goes nowhere (as opposed to erroring, which is the
# already-documented "No route to host" case). That is exactly the shape of
# both open items: OPEN-22 sees it as "no topics ever advertised", OPEN-21
# sees it as "subscribe -> ok and then not one message" - the same handshake
# not happening, observed from the two different ends.
#
# GZ_RELAY makes discovery ALSO unicast to the listed peers. On a
# single-host rig that is 127.0.0.1, and it means a participant can be found
# without any multicast packet succeeding at all. Cheap (one extra unicast
# datagram per discovery beat), no privileges (the alternative is a
# root-only `route add -net 239.0.0.0/8 -interface lo0`), and it changes
# nothing about how data flows once peers are connected.
os.environ.setdefault("GZ_RELAY", "127.0.0.1")

# mission_waypoints() is pure geometry with no gz imports triggered at call
# time, but the FILE it lives in imports gz.transport13 at module level - safe
# to import here only because this server now runs under PYBIN (see
# conductor.sh), which has those bindings. Do not run this under system
# python3 any more.
sys.path.insert(0, GAZEBO_DIR)
from trail_daemon import mission_waypoints  # noqa: E402
from mission_geometry import spawns_behind_wp0 as _spawns_behind_wp0  # noqa: E402
import campaign as _campaign                # noqa: E402

DISCOVERY_STATS = os.path.join(RUN_DIR, "discovery_stats.json")
_GPS_RE = re.compile(r"gps=\(([-0-9.]+),([-0-9.]+)\)")


def _plan_span(pts):
    """Bounding-box diagonal of a planned path, metres - the scale the GPS
    span is compared against, since a trail LENGTH and a bounding BOX are
    different quantities and must not be compared directly."""
    if not pts or len(pts) < 2:
        return 0.0
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    return ((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2) ** 0.5
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
# Video knobs, env-overridable, defaults EXACTLY the values the base64 path
# used - so switching to the MJPEG transport changes the transport and
# nothing else, and any later quality/rate change is a separate, measured
# decision rather than a side effect of this one. CAM_MAX_FPS is a ceiling
# on top of the sensor's own <update_rate> (10 Hz, fleet_world.py), not a
# floor: raising it alone does nothing.
# Threads above the boot baseline before the leak canary shouts. The
# measured camera leak ran at ~+1.2 threads per launch, so 8 catches it
# inside a handful of launches while still clearing an ordinary run's own
# short-lived workers.
THREAD_DRIFT_ALARM = int(os.environ.get("THREAD_DRIFT_ALARM", "8"))
CAM_JPEG_QUALITY = int(os.environ.get("CAM_JPEG_QUALITY", "60"))
CAM_MAX_FPS = float(os.environ.get("CAM_MAX_FPS", "30"))
CAM_MAX_WIDTH = int(os.environ.get("CAM_MAX_WIDTH", "0"))   # 0 = never scale
# Chase-camera FOLLOW tick. The camera model is teleported by set_pose from
# Python, so THIS is the rate at which the viewpoint itself moves - a
# separate ceiling from the display rate, and the one that makes a smooth
# 30 fps stream still look stepped. 0.1 s is what OPEN-19 measured
# (~200-250 ms end to end, about 0.8 m of rubber-band at sprint).
CHASE_FOLLOW_DT = float(os.environ.get("CHASE_FOLLOW_DT", "0.1"))
# Cameras default OFF. Nine live feeds were repeatedly implicated in
# host-load fleet failures (GPU 44-49% with them, 0% without), and a server
# restart used to silently reset drafts back to all-on - which re-armed
# them under the operator mid-session ("KILL THE CAMERAS"). Fail dark;
# checking a box is deliberate.
# cam_chase defaults ON (operator, 2026-08-28) - one chase feed measured
# at zero control-loop cost and a few % GPU; the nine-feed GPU incident
# that created fail-dark was front+nadir+chase x3 dogs. Front/nadir stay
# opt-in.
DEFAULT_CAM_SLOT = dict(cam_front=False, cam_nadir=False, cam_chase=True,
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
    # THE FAST OVAL, RESTORED (2026-08-28): trotRunning the whole way at
    # 3.5, with the analyzer's SPEED CAP governing the sustained curves
    # (WP_VSUS=2.4) and the gait SWITCH explicitly disabled by setting the
    # sustained-segment gait to trotRunning itself (WP_GAIT_CORNER=5).
    #
    # Why cap-only: the mid-course gait switch is what killed every fast
    # config, and phase-gating the switch (real fix, kept - see
    # ConvexMPCLocomotion's PHASE-GATED GAIT ADOPTION) removed the
    # contact-table collapse but a real 9->5/5->9 swap at 2.4+ m/s still
    # fell every rep (earlier lead, lower cap - all fell). Cap-only passed
    # 4/4 at 37.0-37.1s, a 0.1s spread, vs ~80s for trotting-only. And the
    # historical "analyzer oval PASS" (fleet-complete-20260824) never
    # actually switched either - the pre-fix SIM_GAIT override silently
    # discarded the analyzer's cmpc_gait writes, so what that milestone
    # really validated WAS cap-only trotRunning. This recipe makes the
    # historically-validated behavior explicit instead of an accident of a
    # since-fixed bug. "trotRunning cannot hold this curve" (the previous
    # note) was measured UNCAPPED at 3.5 - capped at 2.4 it holds it fine.
    "oval": dict(gait=5, speed=3.5,
                 extra="WP_ANALYZER=1 WP_VSUS=2.4 WP_GAIT_CORNER=5",
                 note="trotRunning @ 3.5, analyzer speed cap 2.4 in the "
                      "sustained curves, NO gait switch (WP_GAIT_CORNER=5 "
                      "keeps trotRunning) - 4/4 PASS at 37.0-37.1s. The "
                      "conservative fallback is trotting @ 2.4 with "
                      "WP_ANALYZER=0 (~80s, long validated)."),
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
                    note="walking @ 1.5 m/s. NOTE circle:R:N is an N-gon: N=8 is an "
                         "OCTAGON (45deg/vertex - the dropdown now says so) and "
                         "N=36 is the functionally-smooth real circle; both share "
                         "this recipe. The turn-grading below was tuned for the "
                         "octagon's 45deg vertices and is a structural no-op on "
                         "the 36-gon (10deg turns sit below turn_soft=17deg, so "
                         "no grading fires - it just walks the smooth arc). "
                         "Graded corridor + gentle WP_ALON + turn-grading ramp "
                         "narrowed for the 45deg corners, R=1.43m - PASS 30.2s (was 32.8s/34.3s at "
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


def mission_label(spec):
    """Human-honest course name for log lines - the spec string stays
    stable everywhere (recipes/history/generators key on it), but the
    OPERATOR reads these lines, and 'circle:9:8' is an OCTAGON. Asked for
    three times before it landed; spec strings alone are not a display."""
    kind = spec.split(":")[0]
    if kind == "circle":
        try:
            n = int(spec.split(":")[2])
        except (IndexError, ValueError):
            n = 0
        if n and n < 24:
            return "octagon" if n == 8 else "%d-gon polygon" % n
        return "smooth circle (%d-gon)" % n if n else "circle"
    return {"star": "5-point star", "oval": "oval", "atom": "atom",
            "dash": "straight dash", "sector": "sector search",
            "parallel": "parallel-track search", "expsquare": "expanding square",
            "lissajous": "Lissajous", "spiro": "spirograph rosette",
            "corner": "corner probe", "outback": "out-and-back"}.get(kind, kind)


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
        self._gps_tail = {}          # index -> byte offset + last fix in bridge_N.log
        self._discovery_failed_this_run = False   # OPEN-22
        self._name_to_index = {}     # "go1_2" -> 2, for the pose subscriber
        self._last_pose = {}         # index -> (x, y, t) for the speed EMA
        self._gz_node = None         # keep the Node alive - gc'ing it drops the subscription
        self._pose_proc = None       # OPEN-21: the per-run pose_feed.py subprocess
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
        # cam_front/cam_nadir/cam_chase default OFF (fail dark - see
        # DEFAULT_CAM_SLOT); chase_distance/height/degree are the side-view
        # default ("side view chase camera... hover over the dogs chasing").
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
        self.run_terrain = None       # terrain of the ACTIVE/last run, for the panel label
        # FLEET SIZE CAP (operator-ordered 2026-08-28: "any time 3 dogs has
        # trouble downgrade to 2 dogs and leave it there by warning the
        # user"). Starts at 3; a 3-dog run in which any dog does not finish
        # cleanly drops it to 2 PERMANENTLY (persisted to RUN_DIR, so it
        # survives restarts - "leave it there"). Raise it back only
        # deliberately: DELETE /api/fleet_cap.
        self.fleet_cap = 3
        self.fleet_cap_reason = ""
        try:
            with open(os.path.join(RUN_DIR, "fleet_cap.txt")) as _f:
                _parts = _f.read().split("|", 1)
                self.fleet_cap = max(1, min(3, int(_parts[0].strip())))
                self.fleet_cap_reason = _parts[1].strip() if len(_parts) > 1 else ""
        except (OSError, ValueError):
            pass
        self.cameras = {}             # legacy field, now unused: /api/state
                                      # serves CAMHUB.manifest() instead
        self._cam_proc = None         # per-run cam_feed.py child (OPEN-19)
        self._cam_restarts = 0
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
        threading.Thread(target=self._host_load_loop, name="host_load",
                          daemon=True).start()

    def _host_load_loop(self):
        while True:
            load = read_host_load()
            with self.lock:
                self.host_load = load
            time.sleep(1.5)

    # BRIDGE GPS AS THE INDEPENDENT ARBITER (2026-08-28 night, operator
    # report: "sometimes I look up and the dog is moving and that message
    # pops other times it's just stopped dead" - both happen, and the
    # instruments could not tell them apart).
    #
    # Every world-motion instrument on this panel - the drawn trail, the
    # live DESYNC monitor, the post-run INVALID gate - reads ONE source:
    # the in-process gz pose feed. That feed is the subject of OPEN-21: it
    # degrades and dies as launches accumulate. When it goes quiet the
    # dog's own displacement reads ZERO, so a perfectly healthy run gets
    # accused of hallucinating. Measured, twice, the night this was added:
    #   run869  DESYNC fired twice ("world/GPS moving 0.00 m/s") on a dash
    #           whose BRIDGE GPS moved 37.4275 -> 37.4284 lat, ~100 m, and
    #           which passed 1/1 waypoints.
    #   run876  gated INVALID at "flew 43.1m of a 178.4m plan" on a sector
    #           whose bridge GPS spans 16.7 x 18.6 m - the right box for a
    #           15 m flower - with 17/17 waypoints and RESULT: PASS.
    # The bridge writes GPS from the sim's own NavSat sensor over a
    # completely separate path (UDP to the controller, no gz-transport),
    # so it is genuinely independent of the failing feed. Read
    # incrementally by byte offset - this runs once per dog per second.
    # ---- OPEN-22 discovery statistics -------------------------------------
    # "tracking how often it occurs can help us decide if we put more effort
    # into a fix" (operator, 2026-08-29). Persisted across server restarts,
    # because the whole point is a rate over many launches - an in-memory
    # counter would reset exactly when the failure takes the server with it.
    def _discovery_stats(self):
        try:
            with open(DISCOVERY_STATS) as f:
                return json.load(f)
        except Exception:  # noqa: BLE001
            return dict(launches=0, failed_launches=0, retries=0,
                        recovered=0, gave_up=0, first_seen=None,
                        last_seen=None, recent=[])

    def _discovery_save(self, st):
        try:
            with open(DISCOVERY_STATS, "w") as f:
                json.dump(st, f, indent=1)
        except Exception:  # noqa: BLE001
            pass

    def _discovery_launch_counted(self):
        """One per launch attempt - the denominator of the rate."""
        st = self._discovery_stats()
        st["launches"] = st.get("launches", 0) + 1
        self._discovery_save(st)
        self._discovery_failed_this_run = False

    def _discovery_note(self, ready, want, retry_index):
        """Record a discovery failure and return a one-line rate summary
        suitable for the orchestration log."""
        now = time.strftime("%Y-%m-%d %H:%M:%S")
        st = self._discovery_stats()
        st["retries"] = st.get("retries", 0) + 1
        if not self._discovery_failed_this_run:
            st["failed_launches"] = st.get("failed_launches", 0) + 1
            st["first_seen"] = st.get("first_seen") or now
            st["last_seen"] = now
            st.setdefault("recent", []).append(
                dict(run=self.run_id, when=now, ready=ready, want=want))
            st["recent"] = st["recent"][-40:]
        self._discovery_save(st)
        self._discovery_failed_this_run = True
        n, f = st.get("launches", 0), st.get("failed_launches", 0)
        return ("OPEN-22 rate: %d of %d launches (%.0f%%), %d retries, "
                "%d recovered, %d gave up"
                % (f, n, (100.0 * f / n) if n else 0.0, st.get("retries", 0),
                   st.get("recovered", 0), st.get("gave_up", 0)))

    def _discovery_recovered(self):
        st = self._discovery_stats()
        st["recovered"] = st.get("recovered", 0) + 1
        self._discovery_save(st)

    def _discovery_gave_up(self):
        st = self._discovery_stats()
        st["gave_up"] = st.get("gave_up", 0) + 1
        self._discovery_save(st)

    def _watch_child(self, proc):
        """Make sure this child cannot outlive us.

        Normal teardown reaps every child through _reap_and_confirm, and
        SIGTERM/SIGINT/SIGHUP now route there too - but a SIGKILL or a hard
        crash runs no handler at all, and macOS has no PDEATHSIG, so the
        child simply survives. Every one of ours is harmful as an orphan: a
        stray gz idles at about a full core simulating an empty world, and a
        stray bridge keeps holding UDP 9100/9101, which this project has
        already lost a day to (a fresh controller talking to a bridge with
        no sim behind it, reporting a frozen roll of exactly -3.14159).
        So each child gets a detached watcher that polls OUR pid and kills
        it within ~2 s of us disappearing, however we disappear.
        """
        try:
            # Kill the process GROUP, not just the pid, when the child leads
            # one. The controller is launched as `bash -c "... timeout 900
            # ./mit_ctrl_sim ..."`, so the thing we hold a handle to is two
            # levels above the binary: SIGKILLing it reaped the wrapper and
            # left ./mit_ctrl_sim running (measured - one survivor out of
            # four children). A group kill takes the whole tree. Guarded on
            # the child actually BEING the group leader, because if it is
            # not, its group is OURS and killing it would take the server
            # down with it.
            try:
                leads_group = os.getpgid(proc.pid) == proc.pid
            except OSError:
                leads_group = False
            target = "-%d" % proc.pid if leads_group else "%d" % proc.pid
            # Exit as soon as EITHER we or the child is gone, and only kill
            # the child if WE were the one that died. The first version
            # looped purely on the server's pid, so every child ever spawned
            # left a sleeping `sh` alive for the server's entire lifetime -
            # 215 of them had accumulated over one campaign. Harmless
            # individually, a leak in aggregate, and exactly the kind of
            # process pile-up that makes a healthy rig look sick in `ps`.
            subprocess.Popen(
                ["sh", "-c",
                 "while kill -0 %d 2>/dev/null && kill -0 %d 2>/dev/null; "
                 "do sleep 2; done; "
                 "kill -0 %d 2>/dev/null || kill -9 %s 2>/dev/null"
                 % (os.getpid(), proc.pid, os.getpid(), target)],
                start_new_session=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:  # noqa: BLE001 - never break a launch over this
            self._note("could not arm the orphan watchdog for pid %d: %r"
                        % (proc.pid, e))

    def _bridge_gps(self, i):
        """Latest (lat, lon) the bridge logged for dog i, or None."""
        st = self._gps_tail.setdefault(i, dict(off=0, fix=None, box=None))
        path = os.path.join(RUN_DIR, "bridge_%d.log" % i)
        try:
            with open(path, "r", errors="ignore") as f:
                f.seek(st["off"])
                chunk = f.read()
                st["off"] = f.tell()
        except OSError:
            return st["fix"]
        for m in _GPS_RE.finditer(chunk):
            la, lo = float(m.group(1)), float(m.group(2))
            st["fix"] = (la, lo)
            b = st["box"]
            st["box"] = (la, la, lo, lo) if b is None else (
                min(b[0], la), max(b[1], la), min(b[2], lo), max(b[3], lo))
        return st["fix"]

    def _gps_span(self, i):
        """Bounding-box diagonal, in metres, of every GPS fix the bridge has
        logged for dog i this run - i.e. how much ground the body actually
        covered, measured off a source the gz pose feed cannot corrupt."""
        self._bridge_gps(i)          # advance the tail
        b = (self._gps_tail.get(i) or {}).get("box")
        if not b:
            return None
        dn = (b[1] - b[0]) * 111320.0
        de = (b[3] - b[2]) * 111320.0 * math.cos(math.radians(b[0]))
        return (dn * dn + de * de) ** 0.5

    def _gps_moved_m(self, i, since):
        """Metres the bridge GPS has moved for dog i since `since`
        (a fix tuple), or None when there is nothing to compare."""
        fix = self._bridge_gps(i)
        if fix is None or since is None:
            return None
        dn = (fix[0] - since[0]) * 111320.0
        de = (fix[1] - since[1]) * 111320.0 * math.cos(math.radians(fix[0]))
        return (dn * dn + de * de) ** 0.5

    def _note(self, msg):
        tag = ("run%d " % self.run_id) if self.run_id else ""
        self.log.append("[%s] %s%s" % (time.strftime("%H:%M:%S"), tag, msg))
        self.log = self.log[-200:]
        print(msg, flush=True)

    # ---- LEAK CANARY -----------------------------------------------------
    # Operator, 2026-08-31, on the camera-Node leak: "how can you get better
    # at not leaving your own processes running and corrupting data? seems
    # like the job of whatever is launching processes. I thought we had
    # python doing that."
    #
    # Correct, and the gap is precise. `self.procs` + `_watch_child` +
    # `_reap_and_confirm` DO own every child faithfully - and could never
    # have caught this, because the leak was not a child. It was an
    # in-process `gz.transport13.Node()` holding C++ threads: a resource the
    # supervisor never spawned and therefore could not reap, "released" by
    # setting a list to []. So the rule that actually generalises is not
    # "remember to clean up", it is:
    #
    #   every long-lived resource must be a CHILD PROCESS the supervisor
    #   owns, and teardown must VERIFY rather than hope.
    #
    # OPEN-21 applied that to the pose feed and OPEN-19 now applies it to
    # the cameras, which is why both are subprocesses. This canary is the
    # part that does not depend on my discipline: the server measures its
    # own thread count against the baseline it booted with and says so, out
    # loud, in the panel and in every campaign log. The leak it was written
    # for ran for weeks at +1.2 threads/launch and was only ever found by
    # hand.
    def health(self):
        try:
            threads = threading.active_count()
        except Exception:  # noqa: BLE001
            threads = -1
        base = getattr(self, "_thread_baseline", None)
        live = [p for p in self.procs if p.poll() is None]
        # Each MJPEG viewer is a long-lived handler thread BY DESIGN, so it
        # is subtracted out - otherwise opening the panel in two tabs would
        # look exactly like the leak this is watching for.
        viewers = Handler._mjpeg_clients
        drift = (threads - base - viewers) if base is not None else 0
        # WHAT is accumulating, not just how much. Two rounds of "the leak
        # is fixed" were called on the drift number alone and both were
        # wrong (+1.2 -> +0.56 -> +1.00 threads/run); a canary that says a
        # leak exists without naming it just moves the guessing around. Every
        # thread this server starts is named, so this histogram points
        # straight at the one that is not exiting.
        names = {}
        for t in threading.enumerate():
            key = t.name.split("-")[0] if t.name.startswith("Thread-") else t.name
            names[key] = names.get(key, 0) + 1
        # ONLY MEANINGFUL WHEN SETTLED. A running fleet legitimately holds
        # ~11 extra threads (one _watch_child per child, the pose-feed
        # reader, the cam-feed reader and its mute pusher, the log poller,
        # the chase follower), so comparing mid-run against an IDLE
        # baseline alarms on every healthy run - which this did, within
        # three minutes of shipping, reporting leaking=True on runs
        # 1884-1886 while nothing was wrong. A canary that cries wolf on
        # normal operation is worse than no canary, because it teaches the
        # operator to ignore it. The invariant that actually holds is:
        # once the fleet is torn down, threads return to baseline.
        settled = self.phase not in ("launching", "running")
        return dict(threads=threads, baseline=base, drift=drift,
                    children=len(live), tracked=len(self.procs),
                    mjpeg_viewers=Handler._mjpeg_clients, settled=settled,
                    by_name=names,
                    leaking=bool(base is not None and settled
                                  and drift > THREAD_DRIFT_ALARM))

    def audit_threads(self, where):
        """Called at every launch and teardown. A clean run returns to the
        baseline; anything else is a leak and gets named while it is small,
        which is the whole point - 54 threads and a 4.7 s /api/state was
        found by a human noticing choppy video, six weeks late."""
        h = self.health()
        if h["baseline"] is None:
            return h
        # At "launch" the previous run should already have settled, and at
        # "teardown" this one just did - both are settled moments, which is
        # exactly why these are the two call sites.
        if h["leaking"] or (where == "teardown" and h["drift"] > THREAD_DRIFT_ALARM):
            self._note("THREAD LEAK: %d threads, %d above the %d-thread "
                        "baseline at %s (%d tracked children, %d alive). "
                        "Every long-lived resource is supposed to be a child "
                        "process this server can reap - something is holding "
                        "in-process threads instead. Campaign data taken "
                        "after this point is suspect (a GIL-bound server "
                        "makes /api/state time out and launches report no "
                        "verdict)."
                        % (h["threads"], h["drift"], h["baseline"], where,
                            h["tracked"], h["children"]))
        return h

    def snapshot(self):
        with self.lock:
            return {
                "phase": self.phase,
                "slots": self.slots,
                "status": self.status,
                "log": self.log[-60:],
                "run_id": self.run_id,
                "hard_cap": HARD_SPEED_CAP,
                "fleet_cap": self.fleet_cap,
                "fleet_cap_reason": self.fleet_cap_reason,
                "model_max_speed": MODEL_MAX_SPEED,
                "recipes": RECIPES,
                "gaits": GAITS,
                "planned": self.planned,
                "positions": self.positions,
                "draft_slots": self.draft_slots,
                "draft_cap": self.draft_cap,
                "draft_terrain": self.draft_terrain,
                "discovery": self._discovery_stats(),   # OPEN-22 rate
                # WHAT CAMPAIGN IS RUNNING. The conductor only ever knew
                # about one run; an 80-run sweep therefore looked like 80
                # unrelated launches with quiet gaps, which is why a working
                # rig kept reading as an idle one. Harnesses publish their
                # progress through campaign.py and it surfaces here.
                "campaign": _campaign.read(),
                # How long the current run has been going, so the panel can
                # show a live clock rather than a static phase word (a
                # campaign looked identical to an idle rig without it).
                "elapsed_s": (time.time() - self.started_at)
                              if (self.started_at and
                                  self.phase in ("launching", "running"))
                              else None,
                "terrain": self.run_terrain,
                # UI self-refresh stamp: app.js compares this to the value it
                # booted with and reloads itself when it changes, so a fix to
                # the panel's own JS reaches an already-open tab within one
                # poll tick. Born of a real failure (2026-08-28): a running-
                # view fix was live on disk + verified in a fresh tab while
                # the operator's long-lived tab still ran the old code and
                # showed 3 draft slots for a 1-dog run - "you did NOT fix
                # the stale slot data."
                "ui_rev": self._ui_rev(),
                "terrain_types": terrain.TERRAIN_TYPES,
                # PIXELS NO LONGER TRAVEL HERE (OPEN-19). This is a
                # manifest - {index: {camname: seq}} - so the panel still
                # knows which tiles are live and can see a frozen feed
                # (seq not advancing) without dragging a base64 JPEG
                # through the whole-state poll. The bytes go out of
                # /api/cam/<i>/<name>.mjpg on their own connection.
                "cameras": CAMHUB.manifest(),
                "host_load": self.host_load,
                "health": self.health(),   # leak canary, see audit_threads()
            }

    # ---- draft editing: one method per interactive element ---------------
    # Every one of these is what a click/edit in the browser does; each is
    # also its own REST route, so the panel and a script hit the exact same
    # code path and can never drift apart.
    def draft_add_slot(self):
        with self.lock:
            if len(self.draft_slots) >= self.fleet_cap:
                if self.fleet_cap < 3:
                    return False, ("fleet capped at %d dogs - %s (DELETE "
                                    "/api/fleet_cap to restore 3)"
                                    % (self.fleet_cap, self.fleet_cap_reason))
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

    def _ui_rev(self):
        try:
            st = os.path.join(HERE, "static")
            return max(os.stat(os.path.join(st, f)).st_mtime_ns
                        for f in ("app.js", "index.html"))
        except OSError:
            return 0

    def restore_fleet_cap(self):
        with self.lock:
            self.fleet_cap = 3
            self.fleet_cap_reason = ""
        try:
            os.remove(os.path.join(RUN_DIR, "fleet_cap.txt"))
        except OSError:
            pass
        self._note("fleet cap restored to 3 dogs by operator request")
        return True, 3

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
            # Mirror the launched terrain into the draft (same treatment the
            # camera fields got): the panel's Terrain dropdown reflects
            # draft_terrain, so a body-launch that picked its own kind used
            # to fly with the dropdown silently showing something else -
            # operator: "I'm watching different types go by with no
            # indication of what they are." run_terrain additionally rides
            # /api/state so the fleet panel can label the ACTIVE run.
            self.draft_terrain = terrain_kind
            self.run_terrain = terrain_kind
            # An empty fleet is refused HERE, not by pinning the last draft
            # slot as unremovable - that pin is what left dog 0 with no
            # delete button. "You cannot launch nothing" is a launch-time
            # constraint; "you may not empty the draft" never was one.
            if not slots:
                return False, ("no dogs in the fleet - add at least one slot "
                               "before launching")
            if len(slots) > self.fleet_cap:
                dropped = len(slots) - self.fleet_cap
                slots = slots[:self.fleet_cap]
                self._note("FLEET CAPPED AT %d: dropped %d slot(s) from this "
                            "launch - %s. DELETE /api/fleet_cap to restore 3."
                            % (self.fleet_cap, dropped,
                                self.fleet_cap_reason or "operator policy"))
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
            # start the clock at LAUNCH, not at gait engage - the world
            # build and the discovery wait are part of what the operator is
            # waiting through, and a clock that only starts later reads as
            # a stall during exactly the slowest part.
            self.started_at = time.time()
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
            elif isinstance(recipe.get("speed"), (int, float)) and \
                    abs(speed - clamp_speed(recipe["speed"], speed_cap,
                                             s.get("model", DEFAULT_MODEL))) > 0.05:
                # Mirror of app.js's cap-aware comparison: judge against the
                # recipe speed CLAMPED by this slot's model ceiling (and the
                # fleet cap), not the raw recipe number - a Go1 Air running
                # the oval at its own 2.5 ceiling is at the best legal speed,
                # not off-recipe, and warning about a limit the operator
                # cannot lift from here is pure noise.
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
            # THE ENV IS AUTHORITATIVE, so make the slot agree with it.
            # A harness that passes --extra "WP_CLOSE_LEG=0" (every sweep in
            # this repo does) turns the closing leg off IN THE CONTROLLER
            # while the slot field stays True - so the drawn/measured plan
            # included a return-to-origin leg the dog never flies. That is
            # not cosmetic: plan_len feeds both the flown/planned ratio and
            # the cross-track metric, and it is why corner rows read
            # plan=71.2 m for a 50 m course with an xtrack of 6.9-10.3 m.
            # Same defect shape as the panel recipe whose label described a
            # configuration it was not launching: two sources of truth for
            # one fact, and they drifted.
            if re.search(r"\bWP_CLOSE_LEG\s*=\s*0\b", extra):
                close_leg = False
                s["close_leg"] = False
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
                                # Omitted cam flags FAIL DARK, matching
                                # DEFAULT_CAM_SLOT: a body-launch (automation)
                                # that never mentions cameras must not spawn
                                # nine GPU-loading feeds as a side effect.
                                # Draft slots always carry explicit flags, so
                                # this only bites clients that omit them.
                                cam_front=bool(s.get("cam_front", False)),
                                cam_nadir=bool(s.get("cam_nadir", False)),
                                cam_chase=bool(s.get("cam_chase", False)),
                                chase_distance=float(s.get("chase_distance", 3.0)),
                                chase_height=float(s.get("chase_height", 1.2)),
                                chase_degree=float(s.get("chase_degree", 90.0))))
        with self.lock:
            self.slots = locked
            # Mirror the LIVE camera fields of what actually LAUNCHED into
            # the index-aligned draft slots. The per-frame mute gate and the
            # chase follower deliberately read the DRAFT (that is what makes
            # the checkboxes/sliders live mid-run) - so a body-launch with
            # cameras ON (mission_runner --chase) was muted instantly by the
            # draft's fail-dark defaults, and its tile never showed. Only
            # cam_*/chase_* sync; mission/gait/speed stay untouched, so
            # automation still never pollutes the draft's course config.
            # Operator-ordered (asked three times, now the rule): the slots
            # panel reflects THE CURRENT RUN - never stale or unused dogs.
            # So the draft is REPLACED by what actually launched: same slot
            # count, same config (gait by name, course label context rides
            # in the note). A 1-dog run leaves a 1-slot panel, during AND
            # after the run. The old preserve-the-draft behavior is gone on
            # purpose; the next fleet is built from what last ran.
            self.draft_slots = [dict(
                mission=s["mission"], gait=s["gait_name"], speed=s["speed"],
                dash=s["dash"], close_leg=s["close_leg"],
                model=(self.draft_slots[s["index"]].get("model", "edu")
                        if s["index"] < len(self.draft_slots) else "edu"),
                extra="", cam_front=s["cam_front"], cam_nadir=s["cam_nadir"],
                cam_chase=s["cam_chase"], chase_distance=s["chase_distance"],
                chase_height=s["chase_height"], chase_degree=s["chase_degree"],
            ) for s in locked]
            self.status = [dict(index=s["index"], phase="pending", text="",
                                 t="", waypoints="") for s in locked]
            self.planned = {}
            self.positions = {}
            self._gps_tail = {}
            # Stale camera frames from the PREVIOUS run otherwise persist
            # (only stop() cleared this): the new run's /api/state carries the
            # old run's last frozen JPEGs until a fresh frame overwrites each
            # key - forever, on a cams-off run. Found 2026-08-28 when a
            # wait-for-cameras poll tripped on the prior run's frames.
            self.cameras = {}
            CAMHUB.clear()
            self._chase_stop = threading.Event()

        threading.Thread(target=self._run, args=(locked, terrain_kind),
                          name="run",
                         daemon=True).start()
        return True, "launching %d dog(s) on %s terrain" % (len(locked), terrain_kind)

    def _run(self, locked, terrain_kind="flat"):
        try:
            os.makedirs(RUN_DIR, exist_ok=True)
            tnote = terrain.TERRAIN_TYPES.get(terrain_kind, {}).get("note", "")
            self._note("TERRAIN for this run: %s%s"
                        % (terrain_kind, (" - " + tnote) if tnote else ""))
            self._note("building fleet world: %s (terrain=%s)"
                        % (", ".join("%s [%s]" % (s["mission"], mission_label(s["mission"]))
                                      for s in locked), terrain_kind))
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
                # ANCHOR THE PLAN WHERE THE DOG ACTUALLY STARTS.
                # corner/dash/outback spawn BEHIND wp0 (the dog is at the
                # local origin, wp0 is out ahead), so a plan that begins at
                # wp0 omits the entire approach leg the robot really flies.
                # Consequence, measured: every `corner:` row reported an
                # xtrack of 6.9-10.3 m, identical across flat/rough/rolling,
                # because the metric scored 25 m of approach as deviation.
                # It looked like a terrain result and was pure geometry.
                if _spawns_behind_wp0(spec) and pts and pts[0] != (0.0, 0.0):
                    pts = [(0.0, 0.0)] + pts
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

            # DISCOVERY GUARD (OPEN-22). gz-transport intermittently comes up
            # without ever advertising the sensor topics - the world builds,
            # gz runs, and nothing is ever discoverable. gz.log is EMPTY when
            # it happens, so there is nothing to read; it is silent.
            # Not root-caused, and it does not need to be to be survivable:
            # a fresh gz process discovers fine, so RETRY, say so through the
            # normal log, and COUNT it. The count is the point - it turns
            # "this happens sometimes" into a number that says whether a real
            # fix is worth the effort.
            def _start_gz():
                gz_log = open(os.path.join(RUN_DIR, "gz.log"), "w")
                # INSTRUMENT THE ONE FAILURE WE CANNOT SEE (OPEN-22).
                # When discovery fails, gz.log is EMPTY - there is literally
                # nothing to read, which is why it has never been root-caused.
                # `-v 3` is gz sim's own INFO level: it goes to this per-run
                # log file, is archived with the run, and touches neither the
                # physics step nor the transport hot path (verbosity gates
                # console output, it does not add work to publishing). Level
                # 4 is debug and IS chatty enough to matter on a long run, so
                # it is available but not the default. GZ_VERBOSE does the
                # same for gz-transport's own discovery chatter, which is the
                # layer actually suspected here.
                genv = dict(env)
                lvl = os.environ.get("CONDUCTOR_GZ_VERBOSITY", "3")
                if os.environ.get("CONDUCTOR_GZ_TRANSPORT_VERBOSE", "1") == "1":
                    genv["GZ_VERBOSE"] = "1"
                gp = subprocess.Popen(["gz", "sim", "-s", "-r",
                                       "-v", lvl, world_out],
                                       cwd=GAZEBO_DIR, env=genv, stdout=gz_log,
                                       stderr=subprocess.STDOUT,
                                       start_new_session=True)
                self.procs.append(gp)
                self._watch_child(gp)
                return gp

            def _wait_for_sensors(seconds):
                dl = time.time() + seconds
                seen = set()
                while time.time() < dl and len(seen) < len(locked):
                    try:
                        topics = subprocess.run(
                            ["gz", "topic", "-l"], env=env, capture_output=True,
                            text=True, timeout=5).stdout
                    except subprocess.TimeoutExpired:
                        topics = ""
                    for sl in locked:
                        if "/go1_%d/imu" % sl["index"] in topics:
                            seen.add(sl["index"])
                    if len(seen) < len(locked):
                        time.sleep(1)
                return seen

            self._discovery_launch_counted()
            p = _start_gz()

            self._note("waiting for %d dog(s) to advertise sensors" % len(locked))
            # Both knobs are ordinary config, not test-only hooks: a slower
            # host legitimately needs a longer window, and a campaign that
            # would rather fail fast than retry can say so. They also make
            # the retry path TESTABLE without faking a failure - set the
            # window to a couple of seconds and attempt 1 genuinely times
            # out, exercising kill -> fresh gz -> wait -> recover -> count.
            WAIT_S = float(os.environ.get("CONDUCTOR_DISCOVERY_WAIT_S", "30"))
            ATTEMPTS = int(os.environ.get("CONDUCTOR_DISCOVERY_ATTEMPTS", "4"))
            ready = _wait_for_sensors(WAIT_S)
            for attempt in range(2, ATTEMPTS + 1):
                if len(ready) >= len(locked):
                    break
                # Count it BEFORE retrying, so a run that eventually succeeds
                # still records that discovery failed once.
                stats = self._discovery_note(len(ready), len(locked), attempt - 1)
                self._note("dogs did not advertise sensors (%d/%d) - "
                            "gz-transport discovery failure (OPEN-22, not "
                            "root-caused, gz.log is silent). Restarting gz "
                            "and retrying: attempt %d of %d. %s"
                            % (len(ready), len(locked), attempt, ATTEMPTS,
                               stats))
                for gp in list(self.procs):
                    try:
                        gp.kill()
                    except Exception:  # noqa: BLE001
                        pass
                self.procs = []
                subprocess.run("pkill -9 -f 'gz[ ]sim -s -r' 2>/dev/null",
                               shell=True)
                time.sleep(2)
                p = _start_gz()
                ready = _wait_for_sensors(WAIT_S)
            if len(ready) >= len(locked) and self._discovery_failed_this_run:
                self._note("sensors advertised after the retry - discovery "
                            "recovered, run proceeding normally")
                self._discovery_recovered()
            if len(ready) < len(locked):
                self._discovery_gave_up()
                # ABORT, but only after ATTEMPTS fresh gz processes have each
                # failed to advertise. This used to say "continuing anyway",
                # and that one
                # decision is what cost the operator an evening: the sensors
                # never advertised (the documented gz-transport discovery
                # failure), the run was dead on arrival, the pose
                # subscription that follows had nothing to attach to, and
                # the gz we had just started was left running with nobody
                # to reap it - idling at about a full core simulating an
                # empty world. Downstream, a sweep kept firing cells at the
                # wreckage and recorded nine of them as FAIL without a
                # mission ever running. A launch that cannot work must fail
                # LOUDLY and CLEANLY, leaving the server idle and the host
                # quiet, not half-started.
                self._note("only %d/%d dogs advertised sensors in 30 s - "
                            "ABORTING this launch and tearing the sim back "
                            "down (gz-transport discovery failure; the world "
                            "built fine, nothing subscribed). Re-launch to "
                            "retry." % (len(ready), len(locked)))
                self._teardown_done.clear()
                try:
                    self._reap_and_confirm(list(self.procs))
                finally:
                    self._teardown_done.set()
                with self.lock:
                    self.phase = "error"
                return

            self.audit_threads("launch")
            self._name_to_index = {"go1_%d" % s["index"]: s["index"] for s in locked}
            self._gz_env = env   # kept for the poller's pose-feed self-heal
            self._subscribe_pose(env)
            self._start_cam_feed(locked, env)
            threading.Thread(target=self._follow_chase_cams, args=(locked,),
                              name="chase_follow",
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
                    stdout=blog, stderr=subprocess.STDOUT,
                    start_new_session=True)
                self.procs.append(bp)
                self._watch_child(bp)
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
                # TERRAIN AWARENESS (OPEN-7): the server is the only place
                # that knows which ground it just built, so it is the only
                # place that can tell the pre-planner. A surface kind's own
                # mu goes straight through; the planner caps its lateral
                # budget at mu*g (see BodyLimits::mu_terrain). Kinds without
                # a surface block (flat, and the procedural heightmaps) send
                # nothing at all, so the planner behaves exactly as it did
                # for every validated result. The heightmap kinds' own
                # SPEED ceiling is deliberately absent until the geometry
                # sweep measures one - encoding a guess here is what the
                # whole terrain program exists to avoid.
                tspec = terrain.TERRAIN_TYPES.get(terrain_kind, {})
                terrain_env = ""
                if "surface" in tspec:
                    terrain_env = "WP_TERRAIN_MU=%.3f " % tspec["surface"]["mu"]
                # MEASURED (terrain, gait) SPEED CEILING. v_terrain_max is a
                # per-LAUNCH scalar and the conductor knows both facts here,
                # so the cap can be the one the measurement actually
                # supports: on rough, WALKING is limited (2.5 fails 0/3
                # while flat and rolling pass 4/5) and trotRunning is not
                # (it passed to 4.5). A per-terrain-only cap would slow a
                # gait this ground never troubled. Absent pair = no cap =
                # previous behaviour exactly.
                vmax = terrain.gait_vmax(terrain_kind, s["gait_name"])
                if vmax:
                    terrain_env += "WP_TERRAIN_VMAX=%.3f " % vmax
                    self._note("dog%d terrain cap: %s on %s is measured to "
                                "%.2f m/s - capping cruise there"
                                % (i, s["gait_name"], terrain_kind, vmax))
                # DEM SAMPLING AT WAYPOINT TIME (OPEN-7). Grip is the same
                # everywhere on a surface, so a scalar mu is the right shape
                # for it. SHAPE is not: a plan crossing a ridge and a plan
                # along a valley floor are different problems on the same
                # terrain kind. So for a terrain with real geometry, sample
                # the heightmap we just generated ALONG THIS DOG'S PLANNED
                # PATH and hand the planner the profile.
                prof_path = os.path.join(RUN_DIR, "terrain_profile_%d.csv" % i)
                amp = float(tspec.get("amplitude", 0.0) or 0.0)
                png = os.path.join(RUN_DIR, "terrain_%s.png" % terrain_kind)
                if amp > 0.0 and os.path.exists(png):
                    try:
                        # ANCHOR AT THE TRUE SPAWN, same as the drawn plan.
                        # dash:30 is ONE waypoint, so without this the
                        # sampler got a single point, could not resample a
                        # polyline, and emitted one row - the controller
                        # duly reported "DEM profile: 1 samples over 0.0 m"
                        # and the relief cap had nothing to act on. A
                        # profile that loads successfully and describes
                        # nothing is worse than none: it reads as evidence.
                        _wp = mission_waypoints(
                            s["mission"], close_leg=s.get("close_leg", True))
                        if _spawns_behind_wp0(s["mission"]) and _wp \
                                and _wp[0] != (0.0, 0.0):
                            _wp = [(0.0, 0.0)] + _wp
                        wpts = ";".join("%.3f,%.3f" % (n, e) for (n, e) in _wp)
                        r = subprocess.run(
                            [sys.executable,
                             os.path.join(HERE, "terrain_profile.py"),
                             "--png", png, "--zscale", "%g" % max(amp * 2.0, 0.02),
                             "--waypoints", wpts, "--out", prof_path],
                            capture_output=True, text=True, timeout=60)
                        for ln in (r.stdout or "").strip().splitlines():
                            self._note("dog%d %s" % (i, ln))
                        if os.path.exists(prof_path):
                            terrain_env += "WP_TERRAIN_PROFILE=%s " % prof_path
                    except Exception as e:  # noqa: BLE001 - never block a launch
                        self._note("dog%d terrain profile FAILED: %r" % (i, e))
                cmd = (
                    # SIM_FALL_EXIT=1: THIS IS A SWEEP, and a sweep wants the process
                    # gone so the next cell can start. The controller now
                    # LATCH-LIMPS by default instead (OPEN-13 part 3) -
                    # right for a machine, wrong for a harness that would
                    # then wait out the full timeout on every fall. The
                    # sweep asks for what it needs; hardware inherits the
                    # safe default rather than a harness convenience.
                    "env DYLD_LIBRARY_PATH=. SIM_FALL_EXIT=1 "
                    "SIM_RUN_ID=%d SIM_INSTANCE=%d SIM_GAIT=%d SIM_VX=%s "
                    "SIM_VX_DELAY_S=%d SIM_VX_RAMP_S=8 WP_MISSION=%s WP_PLANNER=1 "
                    "WP_MAX_YAWRATE=1.2 WP_SPAWN_BEARING_DEG=%.4f %s%s timeout 900 ./mit_ctrl_sim 127.0.0.1 "
                    "stm32mp1-defaults.yaml mc-mit-ctrl-user-parameters.yaml"
                    % (self.run_id, i, s["gait"], s["speed"], delay_s, s["mission"],
                       spawn_bearing_deg, terrain_env, s["extra"])
                )
                ctrl_log_path = os.path.join(RUN_DIR, "ctrl_%d.log" % i)
                archive_log(ctrl_log_path, self.run_id - 1)
                clog = open(ctrl_log_path, "w")
                # start_new_session so this bash -> timeout -> mit_ctrl_sim
                # chain is its own process GROUP: killing the wrapper alone
                # left ./mit_ctrl_sim running (measured), and a group kill
                # takes the whole chain.
                cp = subprocess.Popen(["bash", "-c", cmd], cwd=HOST_RUN,
                                       env=cenv, stdout=clog,
                                       stderr=subprocess.STDOUT,
                                       start_new_session=True)
                self.procs.append(cp)
                self._watch_child(cp)
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
                    # own session: the tail reaper holds NO port, so the
                    # launch-time port sweep cannot see it, and a stale one
                    # appends the PREVIOUS run's text into the NEW run's log
                    # (documented). It must die with us like everything else.
                    ["python3", os.path.join(GAZEBO_DIR, "shm_reaper.py"),
                     "--tail-text", str(i), "--append-to", ctrl_log_path,
                     "--poll", "0.2",
                     # ONE RUN NUMBER EVERYWHERE. The reaper follows only
                     # the ring stamped with THIS run id, so a previous
                     # run's segment - which outlives its process, since
                     # ShmTrace only unlinks at startup - can never be
                     # replayed into this run's fresh log. It did exactly
                     # that earlier tonight and produced a false PASS.
                     "--expect-run-id", str(self.run_id)], start_new_session=True)
                self.procs.append(tbp)
                self._watch_child(tbp)
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
        """Start the per-run pose feed and consume it.

        OPEN-21 ROOT FIX. This used to open a `gz.transport13.Node` inside
        the SERVER process and hold it for the life of the fleet. The
        subscription then accumulated transport state across every launch
        the server ever did, and measurably decayed with it: partial trails
        first, then a dead feed, with the in-process self-heal (drop the
        Node, make a fresh one) only ever transient - by ~20-25 launches it
        stayed dead. Measured 2026-08-29: three recycles in ~20 launches of
        one sweep. And a dead feed is not a cosmetic loss, because every
        world-motion instrument reads it - a healthy dog gets accused of
        hallucinating (which is what the bridge-GPS arbiter exists to
        catch).

        A subscription cannot outlive a process that has exited, so it now
        lives in one that does: `pose_feed.py`, started per RUN, killed with
        the run, taking all of its discovery state with it. The server's own
        long-lived process no longer touches gz-transport at all.

        Everything downstream of the callback is UNCHANGED - same trail
        decimation, same speed EMA, same `_pose_last_t` heartbeat - so this
        swaps the SOURCE without re-deriving any of the behaviour that was
        already validated. `CONDUCTOR_POSE_INPROC=1` restores the old
        in-process path for A/B.
        """
        if os.environ.get("CONDUCTOR_POSE_INPROC") == "1":
            return self._subscribe_pose_inproc(env)
        return self._start_pose_feed(env)

    def _apply_pose(self, idx, x, y, z, yaw, now):
        """The one place a world pose becomes panel state: trail decimation,
        the speed EMA and the freshness heartbeat. Both the subprocess feed
        and the legacy in-process subscriber call THIS, so switching source
        cannot silently change behaviour."""
        SEG_MIN = 0.15
        TRAIL_MAX = 20000
        self._pose_last_t = now
        with self.lock:
            speed = 0.0
            cur = self.positions.get(idx)
            prev = self._last_pose.get(idx)
            if prev is not None:
                px, py, pt = prev
                dt_s = now - pt
                if 1e-3 < dt_s < 1.0:   # skip the first fix after a launch/gap
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

    def _start_pose_feed(self, env):
        """Spawn pose_feed.py for THIS run and read its lines."""
        names = ",".join("%s=%d" % (n, i)
                         for n, i in (self._name_to_index or {}).items())
        if not names:
            self._note("pose feed NOT started - no models placed")
            return
        archive_log(os.path.join(RUN_DIR, "pose_feed.log"),
                    self.run_id - 1)
        ferr = open(os.path.join(RUN_DIR, "pose_feed.log"), "a")
        fp = subprocess.Popen(
            [PYBIN, "-u", os.path.join(HERE, "pose_feed.py"),
             "--world", WORLD, "--names", names, "--rate", "20"],
            cwd=HERE, env=env, stdout=subprocess.PIPE, stderr=ferr,
            text=True, bufsize=1, start_new_session=True)
        self.procs.append(fp)
        self._watch_child(fp)
        self._pose_proc = fp

        def reader():
            # The feed EXITS on its own when it is subscribed-but-deaf or
            # goes deaf mid-run (see pose_feed.py). That is the signal to
            # start a fresh one - a silent failure turned into a retried
            # one. Bounded, so a genuinely broken host does not spin.
            try:
                for line in fp.stdout:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                    except ValueError:
                        continue
                    now = time.time()
                    for k, v in (msg.get("p") or {}).items():
                        try:
                            self._apply_pose(int(k), v[0], v[1], v[2], v[3], now)
                        except (ValueError, IndexError, TypeError):
                            continue
            except Exception as e:  # noqa: BLE001 - reader must never kill a run
                self._note("pose feed reader stopped: %r" % e)
                return
            # stdout closed => the feed process ended. If the fleet is still
            # running, that was a deaf/dead subscription reporting itself.
            rc = fp.poll()
            with self.lock:
                still_running = self.phase == "running"
            if still_running and rc not in (0, None):
                n = getattr(self, "_pose_restarts", 0) + 1
                self._pose_restarts = n
                if n <= 3:
                    self._note("pose feed exited rc=%s (subscribed but deaf, "
                                "or went deaf mid-run - OPEN-21/22). "
                                "Restarting it: attempt %d of 3." % (rc, n))
                    try:
                        self._start_pose_feed(getattr(self, "_gz_env", None)
                                               or os.environ.copy())
                    except Exception as e:  # noqa: BLE001
                        self._note("pose feed restart FAILED: %r" % e)
                else:
                    self._note("pose feed has exited %d times this run - "
                                "leaving it down; the run's own verdict will "
                                "be gated NOFEED by the bridge-GPS arbiter, "
                                "which is the correct outcome" % n)

        threading.Thread(target=reader, name="pose_reader", daemon=True).start()
        self._note("pose feed up (per-run subprocess, pid %d - OPEN-21: "
                    "transport state dies with this run)" % fp.pid)

    def _subscribe_pose_inproc(self, env):
        """The pre-OPEN-21 in-process subscriber. Kept for A/B only."""
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
            for p in msg.pose:
                idx = self._name_to_index.get(p.name)
                if idx is None:
                    continue
                o = p.orientation
                yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                                  1.0 - 2.0 * (o.y * o.y + o.z * o.z))
                self._apply_pose(idx, p.position.x, p.position.y,
                                  p.position.z, yaw, now)

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
        # NO gz Node here any more. This used to create one per run and
        # never release it - the exact leak class OPEN-19 fixed for the
        # camera subscriptions, hiding 70 lines below _subscribe_pose's
        # claim that "the server's own long-lived process no longer touches
        # gz-transport at all", which was therefore false. Measured after
        # the camera fix: settled thread drift still climbing +0.56/run
        # across runs 1887-1905, caught by the leak canary. Every line of
        # geometry below stays here (the server owns the poses and the
        # draft slots); only the blocking service call moves into the
        # per-run child, which dies with the run and takes its transport
        # state with it.
        stop_event = self._chase_stop
        indices = [s["index"] for s in locked if s["cam_chase"]]
        if not indices:
            return
        service = "/world/%s/set_pose" % WORLD
        parked = set()   # dogs whose chase cam is currently live-muted (model parked below the world)
        while not stop_event.is_set():
            batch = []
            for i in indices:
                with self.lock:
                    pos = self.positions.get(i)
                    d = self.draft_slots[i] if i < len(self.draft_slots) else {}
                if pos is None:
                    continue
                # Live on/off for the MODEL half of the chase cam. The STREAM
                # half already gates per-frame in _subscribe_cameras, but the
                # free-floating camera model kept flying around the GUI when
                # the box was unchecked (operator-reported, 2026-08-28). Park
                # it 25 m below the world on uncheck - once, not every tick -
                # and let the normal follow path snap it back on re-check.
                # (The gz-side render keeps running either way; the sensor
                # cannot be despawned mid-run, so the GPU cost of a camera is
                # decided at launch, not by this checkbox.)
                if not d.get("cam_chase", True):
                    if i not in parked:
                        parked.add(i)
                        self._cam_send({"setpose": [{
                            "name": "go1_%d_chasecam" % i,
                            "p": [pos["x"], pos["y"], -25.0],
                            "q": [1.0, 0.0, 0.0, 0.0]}]})
                    continue
                parked.discard(i)
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
                # roll=0, pitch, yaw -> quaternion (standard ZYX Euler
                # composition, matches SDF's own <pose> convention)
                batch.append({"name": "go1_%d_chasecam" % i,
                               "p": [wx, wy, wz],
                               "q": [cp * cy, -sp * sy, sp * cy, cp * sy]})
            if batch:
                self._cam_send({"setpose": batch})
            time.sleep(CHASE_FOLLOW_DT)

    def _start_cam_feed(self, locked, env):
        """Every enabled camera for THIS run, in ONE per-run subprocess
        (cam_feed.py) - OPEN-19 root fix, same shape as _start_pose_feed's
        OPEN-21 fix and for the same reason.

        What this replaces: one in-process `gz.transport13.Node()` per
        camera per launch, "released" at teardown by `self._gz_cam_nodes =
        []`. Dropping a Python reference is not a teardown - the C++
        discovery/reception threads never unwound and nothing ever called
        unsubscribe. Measured 2026-08-31: +1.2 threads per launch, a
        12-hour-old conductor at 54 threads answering /api/state in 4.7 s
        against 0.0005 s fresh, ~20% of the main thread's samples in
        take_gil - and campaign c3 losing 18 of 60 launches to "(no
        verdict)" because the harness's own polling timed out against it.
        The video leak was corrupting mission data.

        The subprocess also takes the JPEG encode with it: 3 dogs x 3
        cameras x 10 Hz was 90 PIL encodes/second holding the server's GIL
        against every HTTP request.

        Which cameras exist is still decided at LAUNCH by
        fleet_world.apply_camera_config - a camera unchecked at launch was
        never spawned and has no topic - so this must keep reading the same
        cam_front/cam_nadir/cam_chase slot fields it always did. Mid-run
        muting is still live, now pushed to the child on its stdin so it
        keeps skipping the ENCODE and not merely the display.
        """
        cams = []
        for s in locked:
            i = s["index"]
            for cam in CAMERAS:
                if s.get(CAM_FLAG_KEYS[cam], True):
                    cams.append("%d:%s" % (i, cam))
        if not cams:
            self._note("no cameras enabled - cam feed not started")
            return
        archive_log(os.path.join(RUN_DIR, "cam_feed.log"), self.run_id - 1)
        ferr = open(os.path.join(RUN_DIR, "cam_feed.log"), "a")
        cp = subprocess.Popen(
            [PYBIN, "-u", os.path.join(HERE, "cam_feed.py"),
             "--cams", ",".join(cams),
             "--quality", str(CAM_JPEG_QUALITY),
             "--rate", str(CAM_MAX_FPS),
             # The child owns EVERY gz-transport interaction for this run,
             # subscriptions and service calls alike - see _follow_chase_cams.
             "--set-pose-service", "/world/%s/set_pose" % WORLD,
             "--follow-dt", str(CHASE_FOLLOW_DT)]
            + (["--max-width", str(CAM_MAX_WIDTH)] if CAM_MAX_WIDTH else []),
            cwd=HERE, env=env, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=ferr, start_new_session=True)
        self.procs.append(cp)
        self._watch_child(cp)
        self._cam_proc = cp
        self._cam_stdin_lock = threading.Lock()

        def reader():
            """Header line + exactly n bytes, repeatedly. Framing errors are
            unrecoverable (the stream is a byte offset, not a record set), so
            a bad header ends the reader rather than trying to resync onto
            what would be JPEG payload."""
            out = cp.stdout
            try:
                while True:
                    line = out.readline()
                    if not line:
                        break
                    try:
                        hdr = json.loads(line)
                        n = int(hdr["n"])
                    except (ValueError, KeyError, TypeError):
                        self._note("cam feed framing error - stream abandoned")
                        break
                    payload = out.read(n)
                    if payload is None or len(payload) != n:
                        break
                    CAMHUB.put("%d:%s" % (hdr["i"], hdr["c"]), payload,
                                hdr.get("w"), hdr.get("h"), hdr.get("t"))
            except Exception as e:  # noqa: BLE001 - reader must never kill a run
                self._note("cam feed reader stopped: %r" % e)
                return
            rc = cp.poll()
            with self.lock:
                still_running = self.phase == "running"
            if still_running and rc not in (0, None):
                n = getattr(self, "_cam_restarts", 0) + 1
                self._cam_restarts = n
                if n <= 3:
                    self._note("cam feed exited rc=%s (subscribed but deaf) - "
                                "restarting: attempt %d of 3" % (rc, n))
                    try:
                        self._start_cam_feed(locked, env)
                    except Exception as e:  # noqa: BLE001
                        self._note("cam feed restart FAILED: %r" % e)
                else:
                    self._note("cam feed has exited %d times this run - "
                                "leaving it down; the mission is unaffected, "
                                "only the video tiles" % n)

        threading.Thread(target=reader, name="cam_reader", daemon=True).start()
        threading.Thread(target=self._push_cam_mutes, name="cam_mutes",
                          daemon=True).start()
        self._note("cam feed up (per-run subprocess, pid %d - OPEN-19: "
                    "%d camera(s), encode and transport state both die with "
                    "this run)" % (cp.pid, len(cams)))

    def _cam_send(self, obj):
        """One writer for the cam feed's stdin - the mute pusher and the
        chase follower both use it, and interleaved partial writes would
        desync the child's line-oriented reader."""
        cp = self._cam_proc
        if cp is None or cp.poll() is not None or cp.stdin is None:
            return False
        try:
            with getattr(self, "_cam_stdin_lock", threading.Lock()):
                cp.stdin.write((json.dumps(obj) + "\n").encode())
                cp.stdin.flush()
            return True
        except (BrokenPipeError, ValueError, OSError):
            return False

    def _push_cam_mutes(self):
        """Mirror the draft checkboxes into the cam feed child so an
        unchecked camera stops being ENCODED, not just stops being drawn.
        Only writes on CHANGE - this is a 2 Hz loop, not a per-frame path."""
        cp = self._cam_proc
        last = None
        while cp is not None and cp.poll() is None:
            with self.lock:
                if self.phase != "running":
                    return
                mutes = sorted(
                    "%d:%s" % (idx, cam)
                    for idx, d in enumerate(self.draft_slots)
                    for cam in CAMERAS
                    if not d.get(CAM_FLAG_KEYS[cam], True))
            if mutes != last:
                last = mutes
                if not self._cam_send({"mute": mutes}):
                    return
            time.sleep(0.5)

    # ---- live status, parsed from the controller logs --------------------
    def _start_poller(self, locked):
        def poll():
            done = set()
            seen_bytes = {}   # index -> bytes of ctrl_%d.log already scanned for EVENT_PATTERNS
            # LIVE DESYNC MONITOR (operator-ordered, 2026-08-28: "you must
            # make the monitor aware of if the dog is NOT moving vs the
            # expected track. GPS is your friend... if the dog thinks it is
            # moving you should always be able to spot check the track and
            # see it move under GPS"). Per dog, per ~1s tick: the BELIEVED
            # speed comes from nav's own status line (v=..m/s - the
            # estimator's opinion); the TRUE speed comes from the gz pose
            # feed (self.positions - the same world truth GPS is derived
            # from). Belief cruising while the world stands still for
            # several consecutive ticks = the estimator hallucinating over
            # slipping/blocked feet, flagged IN THE LOG the moment it is
            # happening rather than at the post-run gate. Recovery is
            # logged too, and the alarm re-arms, so an intermittent slip
            # (mud-style struggling) reads as bursts rather than one stale
            # banner.
            desync = {}   # index -> dict(last=(x,y) or None, bad=0, alarmed=False)
            feed_warned = False
            last_resub = 0.0
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
                        # ---- pose-feed heartbeat: a DEAD FEED must not
                        # read as a stalled dog. _pose_last_t is stamped by
                        # every pose callback; silence >5s while a mission
                        # is under way is an INFRASTRUCTURE alarm (restart
                        # the server to rebuild gz-transport state - the
                        # feed measured degrading partial->total across
                        # runs 747-748 while the dog demonstrably flew).
                        _stalled = (time.time() -
                                     getattr(self, "_pose_last_t", time.time()) > 5.0)
                        if _stalled and not feed_warned:
                            feed_warned = True
                            self._note("POSE FEED STALLED >5s - trail/desync/"
                                        "flown metrics unreliable until it "
                                        "recovers; dog state UNKNOWN (check "
                                        "bridge GPS)")
                        elif not _stalled and feed_warned:
                            feed_warned = False
                            self._note("pose feed RECOVERED")
                        if _stalled and time.time() - last_resub > 20.0:
                            # SELF-HEAL (OPEN-21): the feed measured dying
                            # every ~6-15 launches of in-process Node churn;
                            # a fresh Node + subscription usually revives it
                            # without a server restart. Throttled to one
                            # attempt per 20s.
                            last_resub = time.time()
                            try:
                                # With the feed in its own process (OPEN-21
                                # root fix) the heal is a genuinely fresh
                                # PROCESS, not a fresh Node inside a server
                                # that has been accumulating transport state
                                # all session - which is why the old version
                                # of this was only ever transient.
                                fp = getattr(self, "_pose_proc", None)
                                if fp is not None:
                                    try:
                                        fp.kill()
                                    except Exception:  # noqa: BLE001
                                        pass
                                    self._pose_proc = None
                                self._gz_node = None
                                self._subscribe_pose(getattr(self, "_gz_env", None) or {})
                                self._note("pose feed: restarted - watching "
                                            "for recovery")
                            except Exception as e:  # noqa: BLE001
                                self._note("pose feed restart FAILED: %r - "
                                            "server restart needed" % e)
                        # ---- live desync check (see poll() docstring) ----
                        # Only while the nav status line is FRESH (a new one
                        # appeared this tick): after arrival the line goes
                        # stale at its last v= while the dog legitimately
                        # brakes/settles/lies down, which false-fired a
                        # DESYNC at the end of every completed run (flat
                        # dash showed desync=1 with ratio=1.00 - the tell).
                        fresh = "[nav] wp" in new_text
                        ds = desync.setdefault(i, dict(last=None, bad=0,
                                                        alarmed=False,
                                                        gps=None))

                        with self.lock:
                            p = self.positions.get(i)
                            pos = (p["x"], p["y"]) if p else None
                        # THE FEED MUST BE ALIVE BEFORE IT MAY ACCUSE.
                        # This monitor differences two pose samples; if the
                        # feed stopped delivering, both reads are the SAME
                        # stale sample and the difference is zero - which is
                        # indistinguishable from a dog standing still, and
                        # is exactly how run869 got two DESYNC alarms while
                        # its bridge GPS moved 100 m (see _bridge_gps).
                        # _pose_last_t is stamped by the pose callback.
                        pose_age = time.time() - getattr(self, "_pose_last_t",
                                                          0.0)
                        gps_now = self._bridge_gps(i)
                        try:
                            believed_v = float(v)
                        except ValueError:
                            believed_v = 0.0
                        if not fresh or pose_age > 2.0:
                            # stale nav line, or a feed that has not spoken
                            # this tick: no measurement is possible, so make
                            # no claim and re-baseline.
                            ds["last"] = pos
                            ds["bad"] = 0
                        elif pos is not None and ds["last"] is not None:
                            true_v = ((pos[0] - ds["last"][0]) ** 2 +
                                       (pos[1] - ds["last"][1]) ** 2) ** 0.5  # per ~1s tick
                            if believed_v > 0.5 and true_v < 0.2 * believed_v:
                                ds["bad"] += 1
                            else:
                                if ds["alarmed"]:
                                    self._note("dog%d desync CLEARED - world "
                                                "motion matches belief again" % i)
                                    ds["alarmed"] = False
                                ds["bad"] = 0
                            if ds["bad"] >= 5 and not ds["alarmed"]:
                                # ARBITRATE before accusing. Bridge GPS comes
                                # off the sim's NavSat over UDP, nothing to do
                                # with the gz-transport feed this monitor
                                # reads - so if GPS says the body moved while
                                # the pose feed says it did not, the FEED is
                                # the liar and the dog is fine.
                                gps_m = self._gps_moved_m(i, ds["gps"])
                                if gps_m is not None and gps_m > 0.5:
                                    ds["bad"] = 0
                                    self._note("dog%d pose feed is LYING, not "
                                                "the dog: feed shows %.2f m/s "
                                                "but bridge GPS moved %.1f m "
                                                "over the same window - "
                                                "suppressing DESYNC (OPEN-21)"
                                                % (i, true_v, gps_m))
                                else:
                                    ds["alarmed"] = True
                                    st["text"] += " [DESYNC]"
                                    self._note("dog%d DESYNC: belief moving "
                                                "%.2f m/s, gz pose feed shows "
                                                "%.2f m/s and bridge GPS agrees "
                                                "(%s) for %ds - feet slipping "
                                                "or blocked, estimator "
                                                "hallucinating (SHM trace has "
                                                "per-tick detail)"
                                                % (i, believed_v, true_v,
                                                    ("%.1f m" % gps_m) if gps_m
                                                    is not None else "no fix",
                                                    ds["bad"]))
                            elif ds["alarmed"]:
                                st["text"] += " [DESYNC]"
                        ds["last"] = pos
                        if ds["bad"] == 0:
                            ds["gps"] = gps_now   # window start for the arbiter
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
                        # GROUND-TRUTH GATE, panel-side (same rule as
                        # mission_runner's): the judge inside the controller
                        # trusts the ESTIMATOR, so a dog whose belief flew
                        # the course while its body went nowhere prints a
                        # clean PASS - four of those happened on the rough/
                        # rolling terrains before the runner's gate existed,
                        # and the PANEL kept saying COMPLETE even after the
                        # runner demoted them (operator saw exactly that,
                        # run732). Compare the flown trail (gz world truth,
                        # what the canvas draws) against the planned path;
                        # essentially-no-travel becomes INVALID here too.
                        with self.lock:
                            plan = (self.planned or {}).get(i) or (self.planned or {}).get(str(i))
                            trail = ((self.positions or {}).get(i) or {}).get("trail")
                        def _plen(pts):
                            return sum(((pts[k + 1][0] - pts[k][0]) ** 2 +
                                         (pts[k + 1][1] - pts[k][1]) ** 2) ** 0.5
                                       for k in range(len(pts) - 1)) if pts and len(pts) > 1 else 0.0
                        plan_len, flown_len = _plen(plan), _plen(trail)
                        if plan_len > 3.0 and flown_len < 0.3 * plan_len:
                            # SHORT TRAIL IS TWO DIFFERENT FAULTS and they
                            # must not share a verdict: a dog that really did
                            # not move (a robot result), and a pose feed that
                            # stopped delivering samples (OPEN-21, our
                            # infrastructure). Arbitrate with bridge GPS,
                            # which comes off the sim's NavSat over UDP and
                            # is untouched by the gz-transport feed.
                            # Measured the night this went in: run876's
                            # sector was gated INVALID at "flew 43.1m of a
                            # 178.4m plan" while its bridge GPS spanned
                            # 16.7 x 18.6 m - the correct box for a 15 m
                            # flower - with 17/17 waypoints and RESULT: PASS.
                            span = self._gps_span(i)
                            if span is not None and span > 0.5 * _plan_span(plan):
                                st["phase"] = "nofeed"
                                self._note("dog%d NOFEED: trail has %.1fm of a "
                                            "%.1fm plan, but bridge GPS spans "
                                            "%.1fm of a %.1fm course - the POSE "
                                            "FEED failed, not the dog "
                                            "(OPEN-21; re-run, do not read this "
                                            "as a verdict)"
                                            % (i, flown_len, plan_len, span,
                                               _plan_span(plan)))
                            else:
                                st["phase"] = "invalid"
                                self._note("dog%d INVALID: claimed PASS but flew "
                                            "%.1fm of a %.1fm plan and bridge GPS "
                                            "agrees (%s) - belief completed, body "
                                            "did not (estimator hallucination)"
                                            % (i, flown_len, plan_len,
                                               ("%.1fm span" % span) if span
                                               is not None else "no fix"))
                        else:
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
                    # 3-DOG TROUBLE -> PERMANENT DOWNGRADE TO 2 (see
                    # __init__). "Trouble" = any dog in a 3-dog fleet that
                    # did not finish clean: fell, was gated INVALID, or
                    # never reached a verdict. Deliberately does NOT
                    # inspect WHY - the operator's rule is about fleet
                    # size, and the whole point is to stop paying for
                    # 3-dog flakiness run after run.
                    with self.lock:
                        bad = [st for st in self.status
                               if st.get("phase") != "complete"]
                        if len(locked) >= 3 and bad and self.fleet_cap > 2:
                            self.fleet_cap = 2
                            self.fleet_cap_reason = (
                                "run %s: %d of %d dogs did not finish clean (%s)"
                                % (self.run_id, len(bad), len(locked),
                                    ", ".join("dog%d=%s" % (b["index"],
                                                             b.get("phase", "?"))
                                               for b in bad)))
                            try:
                                with open(os.path.join(RUN_DIR, "fleet_cap.txt"),
                                           "w") as _f:
                                    _f.write("2|%s" % self.fleet_cap_reason)
                            except OSError:
                                pass
                            self._note("FLEET DOWNGRADED TO 2 DOGS - %s. "
                                        "3-dog fleets are disabled until you "
                                        "restore them (DELETE /api/fleet_cap)."
                                        % self.fleet_cap_reason)
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
                        self._cam_proc = None
                        self._teardown_done.clear()
                    self.audit_threads("teardown")
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
        threading.Thread(target=poll, name="log_poller", daemon=True).start()

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
            self._pose_proc = None     # per-run feed dies with the run
            self._pose_restarts = 0
            # OPEN-19: the camera subscriptions are a CHILD PROCESS now, so
            # _reap_and_confirm above has already killed them for real -
            # this used to be `self._gz_cam_nodes = []`, which dropped a
            # Python reference and left the C++ transport threads running.
            self._cam_proc = None
            self._cam_restarts = 0
            CAMHUB.clear()
            self.cameras = {}
            if self._chase_stop is not None:
                self._chase_stop.set()   # tells _follow_chase_cams to exit its loop
            self._teardown_done.clear()
        self._reap_and_confirm(procs)
        self._note("fleet stopped")


FLEET = Fleet()


class CamHub:
    """Latest JPEG per camera, plus a sequence number, for the MJPEG
    endpoint - deliberately on its OWN small lock and NEVER the fleet's big
    `self.lock`.

    OPEN-19. The old path base64'd every frame into `/api/state`, so video
    inherited that endpoint's whole-state JSON, its 400 ms client poll, and
    its lock contention: a display ceiling of 2.5 fps that measured ~0.2 fps
    on a server whose /api/state had degraded to 4.7 s. Frames now leave by
    their own door. Slow clients MISS frames rather than backing anything
    up - this is a live view, not a recording, the same reasoning that made
    the old code keep only the latest frame instead of a queue.
    """

    def __init__(self):
        self._cv = threading.Condition()
        self._frames = {}          # "i:cam" -> (seq, jpeg_bytes, w, h, t)
        self._seq = 0

    def put(self, key, payload, w, h, t):
        with self._cv:
            self._seq += 1
            self._frames[key] = (self._seq, payload, w, h, t)
            self._cv.notify_all()

    def get_after(self, key, last_seq, timeout=5.0):
        """Block until `key` has a frame newer than last_seq, or timeout.
        Returns (seq, payload) or (last_seq, None)."""
        deadline = time.time() + timeout
        with self._cv:
            while True:
                f = self._frames.get(key)
                if f and f[0] > last_seq:
                    return f[0], f[1]
                remain = deadline - time.time()
                if remain <= 0:
                    return last_seq, None
                self._cv.wait(remain)

    def manifest(self):
        """{index: {camname: seq}} - what /api/state ships instead of
        pixels, so the panel still knows which tiles are live and can tell
        a stalled feed (seq frozen) from a healthy one."""
        out = {}
        with self._cv:
            for key, (seq, _p, _w, _h, _t) in self._frames.items():
                i, _, cam = key.partition(":")
                try:
                    out.setdefault(int(i), {})[cam] = seq
                except ValueError:
                    continue
        return out

    def stats(self):
        with self._cv:
            return dict(cameras=len(self._frames), frames=self._seq)

    def clear(self):
        with self._cv:
            self._frames = {}
            self._cv.notify_all()


CAMHUB = CamHub()


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
        # The trailing (?:\?.*)? is NOT optional decoration: app.js appends
        # ?r=<run_id> as a cache-buster so a new run gets a NEW connection
        # instead of the browser holding the previous run's stream. Without
        # it every camera tile 404s - caught in the panel's own network log
        # minutes after this shipped, which is exactly why /api/logs already
        # carries the same suffix.
        m = re.match(r"^/api/cam/(\d+)/([a-z_]+)\.mjpg(?:\?.*)?$", self.path)
        if m:
            return self._mjpeg(int(m.group(1)), m.group(2))
        m = re.match(r"^/api/cam/(\d+)/([a-z_]+)\.jpg(?:\?.*)?$", self.path)
        if m:
            return self._still(int(m.group(1)), m.group(2))
        return super().do_GET()

    # ---- video: its own door, off the state poll (OPEN-19) --------------
    MJPEG_MAX_CLIENTS = 12
    _mjpeg_clients = 0
    _mjpeg_lock = threading.Lock()

    def _mjpeg(self, i, cam):
        """multipart/x-mixed-replace - the browser's own decoder, driven by
        an ordinary <img src>. No JS, no polling, no base64 (which cost a
        flat 4/3 on every frame), and the <img> element is never destroyed,
        so frames replace each other in a live decode pipeline instead of
        each one being a fresh element parsed from a data: URL.

        A slow client MISSES frames; it never backs the producer up. That is
        correct for a live view and is the same reasoning the old code used
        to keep only the latest frame rather than a queue.
        """
        key = "%d:%s" % (i, cam)
        with Handler._mjpeg_lock:
            if Handler._mjpeg_clients >= Handler.MJPEG_MAX_CLIENTS:
                return self._json({"ok": False, "error": "too many viewers"}, 503)
            Handler._mjpeg_clients += 1
        boundary = "cheetahframe"
        try:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=%s" % boundary)
            self.end_headers()
            seq = 0
            idle = 0
            while True:
                seq, payload = CAMHUB.get_after(key, seq, timeout=5.0)
                if payload is None:
                    # No frame in 5 s. Keep the connection open for a while
                    # (a muted or not-yet-launched camera is legitimately
                    # silent and the tile should resume by itself), but do
                    # not hold a thread forever on a camera that will never
                    # publish again.
                    idle += 1
                    if idle > 24:      # ~2 minutes
                        return
                    continue
                idle = 0
                self.wfile.write(
                    ("--%s\r\nContent-Type: image/jpeg\r\n"
                     "Content-Length: %d\r\n\r\n" % (boundary, len(payload))
                     ).encode("ascii"))
                self.wfile.write(payload)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            return            # viewer navigated away; entirely normal
        finally:
            with Handler._mjpeg_lock:
                Handler._mjpeg_clients -= 1

    def _still(self, i, cam):
        """One frame, for a test harness or a curl - the MJPEG stream is for
        eyes, this is for assertions."""
        key = "%d:%s" % (i, cam)
        seq, payload = CAMHUB.get_after(key, 0, timeout=2.0)
        if payload is None:
            return self._json({"ok": False, "error": "no frame for %s" % key}, 404)
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

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
        if self.path == "/api/fleet_cap":
            # Deliberate operator action to re-allow 3 dogs after an
            # auto-downgrade ("leave it there" means it does NOT come back
            # on its own - only a person clears it).
            ok, res = FLEET.restore_fleet_cap()
            return self._json({"ok": ok, "fleet_cap": res})
        m = re.match(r"^/api/slots/(\d+)$", self.path)
        if m:
            ok, res = FLEET.draft_remove_slot(int(m.group(1)))
            return self._json({"ok": ok, "slots": res if ok else None,
                                "message": None if ok else res})
        return self._json({"ok": False, "error": "no such route"}, 404)


def _shutdown(signum, _frame):
    """Reap the fleet before dying. Without this a SIGTERM - which is how
    every restart path stops the server - leaves gz/bridge/controller
    children running, and the next server starts blind to them (self.procs
    is empty in a fresh process). The per-gz parent-death watchdog covers
    the SIGKILL/crash case; this covers the polite one, and does it through
    the same _reap_and_confirm every other teardown uses."""
    try:
        print("[conductor] signal %d - reaping the fleet before exit" % signum,
              flush=True)
        FLEET._reap_and_confirm(list(FLEET.procs))
    except Exception as e:  # noqa: BLE001 - dying anyway, say why
        print("[conductor] teardown on exit failed: %r" % e, flush=True)
    os._exit(0)


def main():
    os.makedirs(RUN_DIR, exist_ok=True)
    # REFUSE TO BE THE SECOND SERVER. allow_reuse_address plus a previous
    # instance that has not finished dying is enough to end up with two
    # processes answering on 8420, and then the browser's requests land on
    # whichever the kernel picks - so a wedged server sitting next to a
    # healthy one reads as "the panel is hung" with nothing obviously wrong.
    # Measured exactly that. Checking first turns a confusing hang into a
    # one-line refusal.
    try:
        with urllib.request.urlopen("http://127.0.0.1:%d/api/state" % PORT,
                                     timeout=3) as f:
            f.read(1)
        print("A conductor is ALREADY answering on %d - refusing to start a "
              "second one. Stop that one first (POST /api/stop, then kill "
              "it) or use conductor_ctl.restart_server()." % PORT, flush=True)
        raise SystemExit(1)
    except SystemExit:
        raise
    except Exception:  # noqa: BLE001 - nothing there, which is what we want
        pass
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGHUP, _shutdown)
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    # Leak-canary baseline: every thread this server legitimately runs while
    # idle. Captured here, after __init__ has started its background
    # samplers and before a single launch, so any later drift is real.
    FLEET._thread_baseline = threading.active_count()
    print("[health] thread baseline = %d" % FLEET._thread_baseline, flush=True)
    with socketserver.ThreadingTCPServer(("127.0.0.1", PORT), Handler) as httpd:
        print("Conductor on http://127.0.0.1:%d" % PORT, flush=True)
        httpd.serve_forever()


if __name__ == "__main__":
    main()
