# TERRAIN.md — ground-only terrain features for the Go1 SITL, and where to get them

Scope, per the operator's request (2026-08-28): **terrain features alone** —
grass, sand, dirt, mud, concrete, rock rubble, uneven stone. NOT stairs,
obstacles, or structures. Every source below was verified to exist (fetched
2026-08-28) unless explicitly marked otherwise; per the records rule, re-check
a source at the moment of use.

## The two rules that govern all of it

1. **A texture is pixels; behavior comes from `<surface>` params.** A
   grass-textured plane walks identically to the speedway unless mu/kp/kd
   change with it. "Matching reality" is therefore two separate jobs: get the
   LOOK+SHAPE from the sources below, and set the PHYSICS from the cheat-sheet
   at the bottom. This project already measured how much this matters: foot
   mu=2.0 (shipped) vs the URDF's 0.6 is the single most flattering number in
   the sim (CLAUDE.md, sim-fidelity table).
2. **Heightmap collision must be probe-verified before anything walks on it.**
   Modern gz-sim has a documented history of objects passing THROUGH heightmap
   collisions (gazebosim/gz-sim issue #1714, "Object passes through heightmap
   in Gazebo Garden"). We already own the validation method: drop probe
   spheres, measure settle (used on the farm mesh; flat to ~1 mm over ±6 m).
   Run it on every new ground before trusting a single run on it.

## List 1 — terrain features, best source, and the physics that makes them real

| feature | best verified source(s) | arrives as | physics to set (beyond visuals) |
|---|---|---|---|
| **grass (mown lawn)** | PX4-gazebo-models `lawn.sdf` (native gz) | SDF world | mu 0.55–0.7 |
| **grass (wild park + dirt mix)** | PX4-gazebo-models `baylands.sdf`; Fuel `baylands_terrain` (DAE); classic dirt/grass/fungus height-blend | mesh / blended heightmap | mu 0.5–0.7 |
| **dirt (packed path)** | cpr_orchard ("rows of small trees separated by dirt paths"); cpr_race_modules dirt segments; classic `dirt_diffusespecular.png` | mesh + texture | mu 0.6–0.8 |
| **mud** | **no ready asset exists** — recipe: dirt visuals + soft contact (kp lowered, `min_depth` raised) + mu 0.3–0.45 | physics recipe, not a download | the slip IS the mud |
| **sand** | Mars Gale Crater Patch 1/2, Apollo 15 heightmap, Moon 60S–30S (Fuel — regolith reads as sand); or flat plane + sand texture | DEM heightmap | mu 0.4–0.6, soft contact, torsional friction (point feet pivot in sand); true deformation not simulable in rigid-body gz |
| **gravel / rock rubble** | Fuel "Harmonic Terrain" + "Harmonic Terrain Objects" (OpenRobotics, named for our gz version); "FRC 2016 Rough Terrain"; SubT cave/tunnel tiles | mesh models | mu 0.5–0.7 — the GEOMETRY does most of the work |
| **uneven stone / rock** | PX4 `ridge.sdf`; Fuel "Lunar Tranquillitatis Pit"; classic `yosemite.world` DEM; classic `heightmap_bowl.png` / `heightmap_valley.png` | heightmap / mesh | mu 0.7–0.9, rigid contact |
| **concrete / asphalt** | cpr_race_modules ("concrete and dirt road segments"); classic `sonoma_raceway.world`; or our own flat plane with mu set honestly | mesh | mu 0.8–1.0 (rubber-on-concrete, per our own sim-fidelity table) |
| **forest floor** | PX4-gazebo-models `forest.sdf` | SDF world | mu 0.5–0.7 |

## List 2 — the sources

### Tier A — native modern-gz (drop-in for our Harmonic stack)

1. **PX4/PX4-gazebo-models** — https://github.com/PX4/PX4-gazebo-models —
   VERIFIED: `worlds/` contains `baylands.sdf`, `lawn.sdf`, `forest.sdf`,
   `ridge.sdf`, `default.sdf` (14 worlds total); targets gz-garden/harmonic;
   BSD-3-Clause. **The single best first stop** — native format, permissive
   license, exactly the grass/park/forest/ridge set.
2. **Gazebo Fuel** — https://app.gazebosim.org (REST:
   `https://fuel.gazebosim.org/1.0/models?q=terrain`) — VERIFIED models:
   "Harmonic Terrain" + "Harmonic Terrain Objects" (OpenRobotics),
   "Apollo15 Landing Site Heightmap 1000x1000 meters", "Lunar Tranquillitatis
   Pit", "Moon 60S 30S" + "Mars Gale Crater Patch 1/2" (jasmeetsingh, LOLA/
   HiRISE elevation data), "Heightmap Bowl" (chapulina), "VRC Driving
   Terrain", "FRC 2016 Rough Terrain", "baylands_terrain" DAE (bdilman).
   Fuel models include-by-URI directly in Harmonic SDF.
3. **osrf/subt** — https://github.com/osrf/subt — VERIFIED (prior session):
   the official DARPA SubT repo; cave/tunnel/urban circuit tiles also on Fuel
   ("DARPA SubT Tech Repo" collection). Underground only: rubble, rock,
   mud-textured floors — no grass anywhere in it. ign-era SDF, mostly
   compatible.

### Tier B — classic-gazebo assets that port (meshes/textures/heightmap PNGs carry over; `.world` files and OGRE material scripts need conversion)

4. **clearpathrobotics/cpr_gazebo** — https://github.com/clearpathrobotics/cpr_gazebo —
   VERIFIED packages: `cpr_agriculture_gazebo` ("flat, outdoor world with a
   barn, fences, and a medium-sized solar farm" — almost certainly the
   ancestor of our in-tree `agriculture_world`), `cpr_inspection_gazebo`
   ("hilly, outdoor world... bridge, cave/mine, water"), `cpr_orchard_gazebo`
   (flat + dirt paths), `cpr_race_modules` (concrete + dirt road segments).
   ROS Noetic / classic Gazebo.
5. **PX4/PX4-SITL_gazebo-classic** — https://github.com/PX4/PX4-SITL_gazebo-classic —
   VERIFIED files: `worlds/baylands.world`, `worlds/yosemite.world`,
   `worlds/sonoma_raceway.world` (plus ksql_airport, mcmillan_airfield per
   the PX4 v1.12 docs world list). Terrain character (baylands=park
   photogrammetry, yosemite=mountain DEM, sonoma=asphalt+grass hills) is
   from knowledge, not re-verified — eyeball on import.
6. **gazebosim/gazebo-classic `media/materials/textures`** —
   https://github.com/gazebosim/gazebo-classic/tree/gazebo11/media/materials/textures —
   VERIFIED filenames: `dirt_diffusespecular.png`, `grass.jpg`,
   `grass_diffusespecular.png`, `fungus_diffusespecular.png`, `terrain.png`,
   `terrain_detail.jpg`, `heightmap_bowl.png`, `heightmap_valley.png` (+
   normal maps). The classic three-texture height-blended ground set.
7. **arpg/Gazebo** — https://github.com/arpg/Gazebo —
   VERIFIED: `worlds/heightmap.world` and `worlds/heightmap_dem.world` —
   working examples of the dirt/grass/fungus multi-texture blend over a
   heightmap, ready to crib the SDF blocks from.

### Tier C — generators ("match reality" literally)

8. **saiaravind19/gazebo_terrain_generator** —
   https://github.com/saiaravind19/gazebo_terrain_generator — VERIFIED:
   draw a polygon on a map → ready `.world` with a textured heightmap built
   from real satellite imagery + DEM of that exact place. Could generate the
   operator's actual yard.
9. **MatthewVerbryke/gazebo_terrain** —
   https://github.com/MatthewVerbryke/gazebo_terrain — VERIFIED: grayscale
   PNG → Gazebo terrain model (the minimal path for hand-drawn test bumps).
10. **Sarath18/terrain_generator** —
    https://github.com/Sarath18/terrain_generator — VERIFIED: heightmap
    wizard with textures and lighting.
11. **LTU-RAI/darpa_subt_worlds** —
    https://github.com/LTU-RAI/darpa_subt_worlds — operator-found, VERIFIED:
    third-party ROS-friendly packaging of the SubT worlds (the original is
    #3).

### Avoid

- **aws-robotics/aws-robomaker-\*** — VERIFIED archived 2026-07-21;
  RoboMaker itself EOL 2025-09-10; README says "do not use this code in
  production" (unpatched deps). Nothing there is worth the port cost now.

## Physics cheat-sheet (the half a download can't give you)

- `<surface><friction><ode><mu>/<mu2>` — the primary knob. Anchors this
  project has already measured/recorded: URDF foot 0.6; shipped sim 2.0
  (deliberately flattering); rubber-on-concrete reality 0.8–1.0.
- `<surface><contact><ode><kp>/<kd>` + `<min_depth>` — ground compliance.
  Lower kp with a few mm of min_depth ≈ soft soil; this is the ONLY lever for
  mud/sand feel, since rigid-body gz cannot deform terrain.
- `<surface><friction><torsional>` — resistance to a foot pivoting in place;
  matters on sand/gravel where real feet auger in.
- Heightmap collision: probe-sphere-validate every new ground (rule 2 above).

## Phase 1 EXECUTED (2026-08-28): the surface matrix, ground-truth verified

24 cells, solo sequential, every PASS re-verified against gz-truth flown
path (mission_runner's flown-vs-planned gate; the full-course GPS ranges
were additionally confirmed per run after a false-alarm scare - see
ISSUES CLOSED (was OPEN-20)).

| tier | config | result |
|---|---|---|
| T1: walking 1.5, dash:30 | flat + 9 surfaces | **10/10 PASS**, nine at 20.3-20.4 s; **ice 20.9 s** (micro-slip at walking's tiny ~0.04 mu demand) |
| T2: trotting 2.5, octagon 45-deg | flat + 9 surfaces | **9/10 PASS** at 25.9-26.3 s (mu >= 0.35 all clear trot's ~0.25 demand - friction never binds, so no time cost); **ice (mu 0.15) FELL** - the demand boundary, found exactly where sqrt(a_lat/g) physics puts it |
| T3: walking 1.5 on GEOMETRY kinds | rolling + rough, dash + octagon | **0/4 - all INVALID**: the dog's belief completes the course while the body goes nowhere (estimator hallucinating over terrain the gait cannot traverse). This is OPEN-7, now gate-verified instead of anecdotal |

Read: the surface physics genuinely reaches the engine (dose-response at
both ends - ice slower at T1, ice down at T2, everything above the
demand line clean), and mu differences below a gait's demand cost
NOTHING in time on flat ground. The next discriminating experiments are
higher-demand configs (trotRunning sprints, tighter corners) where the
demand line climbs into the sand/mud/gravel band - and OPEN-7's terrain
following for the geometry kinds.

## Phase 1b EXECUTED (2026-08-28 evening): the full friction characterization

Per operator order ("systematically perfect each texture... keep track of
the friction vibe in each world and how much it makes the runs deviate...
a dash on each terrain... I suspect oval would handle with similar
interest"). 20 cells, solo, every row ground-truth verified (flown-vs-
planned gate + live desync monitor + bridge-GPS cross-checks); raw data
in `unittests/terrain_friction.csv`.

### The deviation ladder (xtrack_max = worst distance off the planned line)

| terrain | mu | dash:30 walking 1.5 | oval trotRunning 3.5 (VSUS 2.4 curves) |
|---|---|---|---|
| flat | (default) | 20.3 s, 0.08 m | 43.0 s, 0.11 m |
| concrete | 0.90 | 20.4 s, 0.09 m | 42.9 s, 0.14 m |
| asphalt | 0.85 | 20.4 s, 0.09 m | 43.1 s, 0.14 m |
| rock | 0.80 | 20.3 s, 0.13 m | 43.1 s, 0.13 m |
| dirt | 0.70 | 20.3 s, 0.11 m | 43.0 s, 0.13 m |
| grass | 0.60 | 20.3 s, 0.15 m | 43.0 s, 0.11 m |
| gravel | 0.55 | 20.3 s, 0.08 m | 43.0 s, 0.09 m |
| sand | 0.45 | 20.3 s, 0.14 m | 43.0 s, 0.15 m |
| mud | 0.35 | 20.4 s, 0.20 m | 42.9 s, 0.20 m |
| **ice** | **0.15** | **20.9 s, 0.47 m** | **FELL @49 s** |

### What the numbers say

- **mu above a gait's demand line costs NOTHING in time** (all passing
  cells within 0.2 s of flat on both courses) - friction that does not
  bind is free.
- **Deviation scales with contact SOFTNESS, not mu**: the rigid set holds
  0.08-0.14 m on both courses while mud (kp 4e4) wanders 0.20 m
  everywhere - the dog visibly "works" on soft ground without slowing.
- **Ice is the demand boundary in all three regimes**: walking dash PASS
  with 5x wander and +0.6 s (demand ~0.04); trot octagon FELL (demand
  ~0.25); oval sustained curves FELL @49 s (lateral ~0.13 + gait shear
  vs 0.15). The boundary lands exactly where sqrt-friction physics puts
  it, at every tier.
- Everything above was collected THROUGH the monitoring stack built the
  same evening: live belief-vs-world DESYNC alarms, the NOFEED verdict
  (a dead pose feed is infrastructure, not a robot verdict - one mud
  oval row shows the catch, its clean retry beside it), the pose-feed
  heartbeat + fresh-node self-heal (OPEN-21), and bridge GPS as the
  independent arbiter.

## How this feeds the test plan

- **Phase 0** (no new assets): validate the panel's existing `rolling`/
  `rough` terrain kinds + the farm-walk RETEST (ISSUES OPEN-7), probe spheres
  first.
- **Phase 1** (no new assets): mu/kp sweep on the flat speedway — concrete
  (0.9) → dirt (0.7) → grass (0.55) → mud (0.35) as pure physics changes,
  per-gait pass/fail. Cheapest realism win available; also closes the
  "mu=2.0 is flattering" gap on our own terms.
- **Phase 2**: heightmaps — classic `heightmap_bowl/valley` PNGs, then a
  generator (#8) patch of real ground.
- **Phase 3**: native meshes — PX4 `lawn`/`ridge`/`baylands`, Fuel "Harmonic
  Terrain".
- **Phase 4**: SubT tiles (rubble/rock/mud floors) for the underground set.

## Phase 2 EXECUTED (2026-08-28 night): the planner now KNOWS the ground

Operator instruction: "add the rest to the pre-planner for each terrain
type and gait on said terrain." This is the first half of that — the
friction axis, which is the one Phase 1b already measured well enough to
encode as physics rather than as a tuned table.

**The rule, and why it is a rule and not a lookup.** A body turning at
speed v on a path of curvature kappa needs lateral acceleration
`a_lat = v^2 * kappa`, and the ground can only supply `mu * g` before the
feet slide. So the planner's lateral budget is not a free parameter on a
surface with known friction:

    a_lat_max <= safety * mu * g        (safety = 0.9, $WP_TERRAIN_SAFETY)

`BodyLimits::mu_terrain` (`common/include/Planning/BodyPathPlanner.h`)
carries it; `plan()` applies it BEFORE `buildPath`/`computeGeometry`/
`computeSpeedProfile`, so every downstream number — corner speeds, the
backward braking pass, the analyzer's sustained-segment caps — is
computed against what this ground can actually deliver, not against the
2.5 default and then trimmed afterwards. Unset (`-1`) is stock behaviour
bit-for-bit, which is what keeps every validated flat result valid.

**The conductor is the only thing that knows which ground it built**, so
it is the only thing that can tell the planner: `server.py`'s controller
launch line now carries `WP_TERRAIN_MU=<mu>` taken straight from
`terrain.py`'s own `TERRAIN_TYPES[kind]["surface"]["mu"]` — the same
number that goes into the SDF's `<surface>` block and into the foot
collisions. One source, both sides of the contact pair, and the planner.

**Verified live, not argued** (`dash:20`, walking @1.5, terrain `ice`):

    [plan] terrain mu=0.15 caps lateral budget 2.50 -> 1.32 m/s^2
    [plan] 201 pts, 20.0 m, tightest R=0.00 m -> 1.50 m/s (cruise 1.50,
           a_lat 1.32, corridor 1.00)          <- PASS, ratio 1.03

0.9 x 0.15 x 9.81 = 1.324, which is what it printed. Flat and the
geometry kinds send nothing at all, so they plan exactly as before.

**A defect this exposed and fixed in passing**: the `[plan] ... a_lat`
summary read the CALLER's `lim` copy, not the planner's post-cap `_lim`,
so it printed `a_lat 2.50` one line under a `terrain mu` line saying the
budget was now 1.32. A summary that contradicts the line above it is
worse than no summary; it now prints `planner.limits()`.

**What is deliberately NOT encoded yet**: `v_terrain_max` (the per-kind
SPEED ceiling) exists in `BodyLimits` and is wired to `$WP_TERRAIN_VMAX`,
and it is left UNSET on every kind. Phase 1b measured that mu above a
gait's demand line costs nothing in time, so there is no measured speed
ceiling to encode for the surface kinds; the geometry kinds (rolling,
rough) plausibly have one, and `unittests/terrain_envelope.py` is the
sweep that would measure it. Encoding a guess there is precisely what
this whole terrain program exists to avoid.

### Predicted from the rule, for the record (not yet measured)

At the 2.5 default budget the cap binds only below mu ~= 0.283, so:

| kind | mu | capped a_lat | binds? |
|---|---|---|---|
| concrete .90 / asphalt .85 / rock .80 / dirt .70 | — | — | no |
| grass .60 / gravel .55 / sand .45 / mud .35 | — | — | no |
| **ice** | 0.15 | **1.32** | **yes** |

Which reproduces Phase 1b's own measured outcome — ice was the only
surface that failed anything, and it failed at trot+oval, the highest
lateral demand in the matrix. The rule and the data agree without the
rule having been fitted to the data.
