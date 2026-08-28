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
