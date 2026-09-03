# Cheetah ↔ Go1 Gazebo SITL

> **Moved 2026-08-31**: this tree used to live at `stm32mp1/gazebo/`. It is a
> host-side SITL harness and has nothing to do with the STM32MP1 board port
> beyond history — `stm32mp1/` is now just the board port (toolchain, deploy,
> `lcm_shim`, `robot_main.cpp`). Every reference was rewritten; the relative
> `#include "../../gazebo/ShmTrace.h"` depths are unchanged because only a
> path segment was removed, not a directory level.

## Start here

```bash
./start.sh              # bring the Conductor up (refuses to be a second server)
./start.sh --open       # ...and open the panel
./start.sh --restart    # stop.sh first, then start
./status.sh             # is the rig actually working? (also: leak canary, strays)
./stop.sh               # SAFE teardown - see below
./stop.sh --hard        # also -9 any strays the reap left behind
```

`stop.sh` calls **`/api/stop` first**, then kills the server. That order is
not cosmetic: `/api/stop` routes through the conductor's own
`_reap_and_confirm()`, which kills every child it spawned and confirms each
one dead. Killing the server first leaves it unable to reap what it owned —
an orphaned `gz sim` then survives into the next run and lands its output in
that run's log. That has already cost this project a debugging session in
which three gaits were wrongly recorded as failing.

`conductor/conductor.sh` is the older launcher and does **not** do this — it
`kill -9`s first. Prefer `start.sh`.


Run the Cheetah controller (on the STM32MP1) against a simulated Unitree **Go1** in
**Gazebo Sim 8 (gz-harmonic)** on the Mac, connected over UDP — the same pattern as
the OpenPilot `gazebo_bridge`. Simulated **IMU, baro, and GPS** feed the controller;
the controller's joint impedance commands drive the Go1 through a motor-PD in the bridge.

```
Gazebo (Go1)  ── /imu /air_pressure /navsat /joint_state ──►  cheetah_gazebo_bridge.py
   (Mac)       ◄── /model/go1/joint/*/cmd_force ──────────────  (Mac, motor PD)
                                                                       │ UDP
                                                                       ▼
                                              jpos_ctrl_sim / stand_sim  (STM32MP1)
```

## Layout

- `start.sh` / `stop.sh` / `status.sh` — the entry points above.
- `conductor/` — the browser control panel on `:8420` (`server.py`), the
  mission harness (`mission_runner.py`), and the per-run subprocesses that
  own all gz-transport for a run (`pose_feed.py`, `cam_feed.py`). Nothing
  long-lived in the server touches gz-transport; see ISSUES.md OPEN-19/21 for
  why that rule exists and what it cost to learn.
- `worlds/`, `models/` — generated worlds and visual assets.
- `ShmTrace.h` — per-tick SHM tracing, included from the controller tree by
  relative path (`../../gazebo/ShmTrace.h` and friends). It lives here rather
  than in `robot/include/` because it is a SITL diagnostic.
- `*Controller.{cpp,hpp}`, `*_main.cpp`, `WaypointNav.*` — the sim-side
  controllers, built by `robot/CMakeLists.txt` and
  `user/MIT_Controller/CMakeLists.txt`.

## Historical layout notes
- `unitree_ros/robots/go1_description/` — the Go1 model (sparse checkout of unitree_ros).
- `make_world.py` — converts `go1.urdf` → a gz-harmonic world: strips ROS/classic
  plugins, adds IMU + baro (air_pressure) + GPS (navsat) sensors, a joint-state
  publisher, and 12 per-joint force inputs. Regenerate: `python3 make_world.py /tmp/go1_raw.sdf`
  (first: `gz sdf -p unitree_ros/robots/go1_description/urdf/go1.urdf > /tmp/go1_raw.sdf`).
- `worlds/go1.sdf` — the generated world.
- `cheetah_gazebo_bridge.py` — gz-transport ↔ UDP bridge (runs the Unitree motor PD).
- `run_gazebo_sim.sh` — launches Gazebo (headless) + the bridge on the Mac.
- `sim_main.cpp` / `StandController.*` / `stand_main.cpp` — the MP1-side controllers
  (built as `jpos_ctrl_sim` and `stand_sim`; see robot/CMakeLists STM32MP1 branch).

## Run
On the Mac:
```bash
gazebo/run_gazebo_sim.sh          # headless  (--gui for the viewer)
```
On the STM32MP1 (from the deployed package):
```bash
cd /usr/local/cheetah-mp1
./stand_sim   <mac-lan-ip>                 # holds a Go1 stance (clean demo)
./jpos_ctrl_sim <mac-lan-ip>               # JPos sine sweep
```
UDP: controller → bridge on :9100 (impedance command), bridge → controller on :9101
(IMU/joint/baro/GPS). The bridge learns the controller's IP from its first packet.

## Notes / next
- Joint map is identity (Go1 `{hip,thigh,calf}` = Cheetah `{abad,hip,knee}`); the Go1
  stands correctly with it. Tune `SIGN`/`OFFSET` in the bridge if a controller needs
  a different convention (e.g. for locomotion).
- Baro/GPS are plumbed through to the controller (`gazebo_get_aux()`), unused by the
  control law today — reserved for waypoint navigation.
- Sim sensors sidestep the real CAN-IMU/DroneCAN dependency, so no board sensors are
  needed for controller development.
