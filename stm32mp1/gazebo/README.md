# Cheetah ↔ Go1 Gazebo SITL

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
stm32mp1/gazebo/run_gazebo_sim.sh          # headless  (--gui for the viewer)
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
