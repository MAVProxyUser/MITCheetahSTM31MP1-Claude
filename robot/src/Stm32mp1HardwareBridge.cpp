/*!
 * @file Stm32mp1HardwareBridge.cpp
 * @brief Headless hardware bridge for the Octavo OSD32MP1 port. See the header.
 */
// portable: BSD sockets + pthreads (Linux-only pieces are guarded inline)

#include "Stm32mp1HardwareBridge.h"

#ifdef __linux__
#include <sched.h>
#include <sys/mman.h>
#endif
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "rt/rt_rc_interface.h"
#include "Math/orientation_tools.h"

// The SpineBoard structs and the LCM POD structs must be layout-identical for the
// memcpy bridge below to be valid.
static_assert(sizeof(SpiCommand) == sizeof(spi_command_t), "SpiCommand layout mismatch");
static_assert(sizeof(SpiData)    == sizeof(spi_data_t),    "SpiData layout mismatch");

/*!
 * Headless port has no RC. RobotRunner calls this every step; we report "no RC"
 * (all zero -> mode 0). This is ignored because use_rc is 0 in the port config.
 * Defined here so the SBUS/serial chain (which needs <stropts.h>) is not compiled.
 */
void get_rc_control_settings(void* settings) {
  memset(settings, 0, sizeof(rc_control_settings));
}

// The RC chain (rt_rc_interface.cpp) normally defines this global; the MIT
// FSM references it via `extern rc_control_settings rc_control`. Headless port
// has no RC, so provide a zeroed instance.
rc_control_settings rc_control;

Stm32mp1HardwareBridge::Stm32mp1HardwareBridge(RobotController* controller,
                                               std::string robot_yaml,
                                               std::string user_yaml,
                                               Backend backend)
    : _controller(controller), _robotYaml(robot_yaml), _userYaml(user_yaml),
      _backend(backend) {
  _userControlParameters = controller->getUserControlParameters();
#ifdef __linux__
  _unitreeCfg = unitree_default_config();   // A1, 4 buses; override before run() as needed
#endif
  // _canImuCfg defaults: can0, node 124, msgs 20500/20501
  memset(&_spiData, 0, sizeof(_spiData));
  memset(&_spiCommand, 0, sizeof(_spiCommand));
  // Zero the operator command channel. It was left uninitialised, so the
  // waypoint driver's "wait until the sequencer has ramped the stick to SIM_VX"
  // gate saw garbage that already exceeded the target and took the stick at
  // t=0 - before the robot had even stood up, which spun it 170 degrees off
  // heading. An uninitialised stick is worse than a bug in sim: on hardware it
  // is a velocity command nobody asked for.
  memset(&_gamepadCommand, 0, sizeof(_gamepadCommand));
  memset(&_utCmd, 0, sizeof(_utCmd));
  memset(&_utData, 0, sizeof(_utData));
}

void Stm32mp1HardwareBridge::setupScheduler() {
  // Best-effort real-time. On the board run as root for this to take effect;
  // during bring-up we warn and continue rather than abort.
#ifdef __linux__
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    printf("[stm32mp1] mlockall failed (run as root for RT determinism); continuing\n");
  struct sched_param params;
  params.sched_priority = 49;
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1)
    printf("[stm32mp1] SCHED_FIFO failed (run as root for RT determinism); continuing\n");
#else
  printf("[stm32mp1] host build: default scheduling (no mlockall/SCHED_FIFO)\n");
#endif
}

void Stm32mp1HardwareBridge::initHardware() {
  if (_backend == Backend::GAZEBO) {
    printf("[stm32mp1] backend: GAZEBO (UDP to %s cmd:%d sensor:%d)\n",
           _gazeboCfg.peer_addr, _gazeboCfg.cmd_port, _gazeboCfg.sensor_port);
    if (init_gazebo(_gazeboCfg, &_vectorNavData) != 0)
      printf("[stm32mp1] Gazebo UDP init failed\n");
    return;
  }
  printf("[stm32mp1] init CAN IMU (%s node %d)\n", _canImuCfg.interface, _canImuCfg.imu_node_id);
#ifdef __linux__
  if (init_can_imu(_canImuCfg, &_vectorNavData) != 0)
#else
  printf("[stm32mp1] HARDWARE backend (CAN IMU) is Linux-only\n"); exit(1);
#endif
    printf("[stm32mp1] CAN IMU init failed; estimator will run on a stale/identity IMU\n");

  printf("[stm32mp1] init Unitree RS485 (%d bus(es))\n", _unitreeCfg.num_buses);
  if (init_unitree(_unitreeCfg) != 0)
    printf("[stm32mp1] Unitree RS485 init failed; motors will not respond\n");
}

void Stm32mp1HardwareBridge::runMotors() {
  // RobotRunner has written _spiCommand; ship it and read back _spiData.
  // SpiCommand/spi_command_t (and SpiData/spi_data_t) are layout-identical (asserted above).
  memcpy(&_utCmd, &_spiCommand, sizeof(_utCmd));
  if (_backend == Backend::GAZEBO) {
    gazebo_send_receive(&_utCmd, &_utData);
    // GROUND TRUTH, FOR INSTRUMENTATION ONLY. `cheater_mode` is now hard-zero
    // (see the note in init) and no estimator consumes this struct, so nothing
    // here reaches the control loop. It exists so [ESTERR] can report how far
    // the LinearKF is from reality - the estimator gap is this port's measured
    // speed wall, and you cannot close a gap you refuse to measure.
    // MIT quat order is (w,x,y,z); vBody = rBody * vWorld with rBody = world->body.
    {
      SimTruth t;
      gazebo_get_truth(&t);
      Quat<double> q;
      q << t.quat[3], t.quat[0], t.quat[1], t.quat[2];
      _cheaterState.orientation = q;
      _cheaterState.position << t.pos[0], t.pos[1], t.pos[2];
      Vec3<double> vWorld(t.vworld[0], t.vworld[1], t.vworld[2]);
      _cheaterState.vBody = ori::quaternionToRotationMatrix(q) * vWorld;
      _cheaterState.omegaBody << _vectorNavData.gyro[0], _vectorNavData.gyro[1],
          _vectorNavData.gyro[2];
      _cheaterState.acceleration << _vectorNavData.accelerometer[0],
          _vectorNavData.accelerometer[1], _vectorNavData.accelerometer[2];
    }
    // ---- ABSOLUTE POSITION AIDING (baro / GPS) ---------------------------
    // MIT's KF fuses IMU with LEG ODOMETRY, which is weighted per foot by a
    // contact trust that goes to zero in swing - so an all-swing window leaves
    // position with no measurement at all and the covariance diverges. Baro and
    // GPS are absolute and contact-independent, so they bound that drift. Same
    // sensors the real dog gets over CAN; here they come over UDP.
    // Opt-in ($SIM_ABS_AIDING=1) so existing measurements are unaffected.
    {
      static const bool aidOn = getenv("SIM_ABS_AIDING") &&
                                atoi(getenv("SIM_ABS_AIDING")) != 0;
      if (aidOn) {
        SimAuxSensors aux;
        gazebo_get_aux(&aux);
        static bool  originSet = false;
        static double lat0 = 0, lon0 = 0, mPerDegLon = 0;
        static float  baro0 = 0;
        static float  estOffX = 0, estOffY = 0, estOffZ = 0.08f;
        if (!originSet && aux.gps_lat != 0.0) {
          lat0 = aux.gps_lat; lon0 = aux.gps_lon; baro0 = aux.baro_alt;
          mPerDegLon = 111320.0 * std::cos(lat0 * M_PI / 180.0);
          // FRAME ALIGNMENT. The estimator's position origin is the robot's
          // pose at spawn; the GPS origin is wherever it happens to be at the
          // FIRST FIX, which is later - after stand-up has already moved it. If
          // the two are not aligned, the aiding corrects toward a systematically
          // wrong reference, and the gentler the gain the more relentlessly it
          // drags the estimate into that bias. Measured exactly that: aiding off
          // 20.68 m, low-gain 5.78 m, gentlest (tau=10 s) 0.20 m - harm growing
          // as the correction got softer, which is the signature of a bias
          // rather than noise. Record where the estimator thinks it is at the
          // moment the GPS origin is captured, and carry that as the offset.
          if (_robotRunner) {
            const auto& est = _robotRunner->getStateEstimate();
            estOffX = est.position[0];
            estOffY = est.position[1];
            estOffZ = est.position[2];
          }
          originSet = true;
          printf("[stm32mp1] abs aiding: origin lat=%.7f lon=%.7f baro=%.2f m\n",
                 lat0, lon0, baro0);
          fflush(stdout);
        }
        if (originSet) {
          // Equirectangular projection about the origin, same as WaypointNav.
          // Gazebo world is ENU: x = East, y = North, z = up.
          const float north = (float)((aux.gps_lat - lat0) * 111320.0);
          const float east  = (float)((aux.gps_lon - lon0) * mPerDegLon);
          // Baro is referenced to its own value at spawn, plus the height the
          // robot actually starts at (belly on the deck).
          const float z = (aux.baro_alt - baro0) + estOffZ;
          _absAiding.position << east + estOffX, north + estOffY, z;
          _absAiding.haveXY = true;
          _absAiding.haveZ  = true;
          // 1-sigma. Baro is ~0.08 m here, far coarser than the 2-5 cm a gait
          // moves the body - it bounds drift, it does not track the bounce.
          const float gps_sig  = getenv("SIM_GPS_SIGMA")
                               ? atof(getenv("SIM_GPS_SIGMA")) : 0.5f;
          const float baro_sig = getenv("SIM_BARO_SIGMA")
                               ? atof(getenv("SIM_BARO_SIGMA")) : 0.10f;
          _absAiding.sigma << gps_sig, gps_sig, baro_sig;
        }
      }
    }
  } else {
#ifdef __linux__
    unitree_send_receive(&_utCmd, &_utData);
#else
    printf("[stm32mp1] HARDWARE backend is Linux-only\n"); exit(1);
#endif
  }
  memcpy(&_spiData, &_utData, sizeof(_spiData));
}

void Stm32mp1HardwareBridge::run() {
  setupScheduler();
  initHardware();

  printf("[stm32mp1] loading robot parameters from %s\n", _robotYaml.c_str());
  try {
    _robotParams.initializeFromYamlFile(_robotYaml);
  } catch (std::exception& e) {
    printf("[stm32mp1] robot yaml failed: %s\n", e.what());
    exit(1);
  }
  if (!_robotParams.isFullyInitialized()) {
    printf("[stm32mp1] robot parameters incomplete in %s\n", _robotYaml.c_str());
    exit(1);
  }

  if (_userControlParameters) {
    printf("[stm32mp1] loading user parameters from %s\n", _userYaml.c_str());
    try {
      _userControlParameters->initializeFromYamlFile(_userYaml);
    } catch (std::exception& e) {
      printf("[stm32mp1] user yaml failed: %s\n", e.what());
      exit(1);
    }
    if (!_userControlParameters->isFullyInitialized()) {
      printf("[stm32mp1] user parameters incomplete in %s\n", _userYaml.c_str());
      exit(1);
    }
  }

  printf("[stm32mp1] parameters loaded; starting control\n");

  _robotRunner = new RobotRunner(_controller, &_taskManager,
                                 _robotParams.controller_dt, "robot-control");
  _robotRunner->driverCommand = &_gamepadCommand;
  _robotRunner->spiData = &_spiData;
  _robotRunner->spiCommand = &_spiCommand;
  _robotRunner->robotType = RobotType::MINI_CHEETAH;   // Unitree legs mapped onto the MC model for now
  _robotRunner->vectorNavData = &_vectorNavData;
  _robotRunner->cheaterState = &_cheaterState;
  if (getenv("SIM_ABS_AIDING") && atoi(getenv("SIM_ABS_AIDING")) != 0)
  {  _robotRunner->absAiding = &_absAiding;
     printf("[stm32mp1] absAiding wired: %p\n", (void*)&_absAiding); fflush(stdout); }
  // CHEATER MODE IS DELETED. There is no environment variable, no yaml key and
  // no code path that can feed sim ground truth into the state estimator. It is
  // gone deliberately, because merely FIXING it twice did not stop it being used:
  //   1. getenv("SIM_CHEATER") is non-null for "0", so SIM_CHEATER=0 still
  //      switched it ON, and every "real estimator" number ever recorded for
  //      this port was in fact a ground-truth run;
  //   2. after that was found and retracted in CLAUDE.md, the sweep harnesses
  //      went on hardcoding SIM_CHEATER=1 in their env block, and a whole 100 m
  //      dash table was measured and reported off the back of it.
  // A capability that is only safe when everyone remembers a rule is not safe.
  // Ground truth is still read for INSTRUMENTATION ONLY (see gazebo_get_truth /
  // SIM_ESTERR below) so estimator error can be quantified against it. Logging
  // truth is measurement; feeding truth to the controller is fiction.
  _robotParams.cheater_mode = 0;
  if (_backend == Backend::GAZEBO)
    printf("[stm32mp1] REAL ESTIMATOR: VectorNav orientation + LinearKF"
           " (cheater mode does not exist)\n");
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->visualizationData = &_visualizationData;
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;

  // RS485 motor exchange (~500 Hz) and the control loop as separate periodic tasks,
  // sharing _spiData/_spiCommand (same lock-free pattern as the stock spiTask).
  PeriodicMemberFunction<Stm32mp1HardwareBridge> motorTask(
      &_taskManager, 0.002, "unitree-rs485",
      &Stm32mp1HardwareBridge::runMotors, this);
  motorTask.start();

  _robotRunner->start();

  // GAZEBO SITL has no operator/RC: auto-sequence the FSM control_mode so an FSM
  // controller (MIT_Controller) goes PASSIVE -> STAND_UP -> LOCOMOTION on its own.
  // Controllers that ignore control_mode (JPos/Stand) are unaffected. Override with
  // $SIM_MODE (final control_mode) / $SIM_STAND_S / $SIM_LOCO_S.
  if (_backend == Backend::GAZEBO) {
    std::thread([this]() {
      const char* m = getenv("SIM_MODE");
      int final_mode = m ? atoi(m) : 4;               // default: LOCOMOTION
      int t_stand = getenv("SIM_STAND_S") ? atoi(getenv("SIM_STAND_S")) : 4;
      int t_bal   = getenv("SIM_BAL_S")   ? atoi(getenv("SIM_BAL_S"))   : 8;
      int t_loco  = getenv("SIM_LOCO_S")  ? atoi(getenv("SIM_LOCO_S"))  : 14;
      usleep(t_stand * 1000000);
      _robotParams.control_mode = 1;                  // K_STAND_UP
      printf("[sim] control_mode -> STAND_UP\n"); fflush(stdout);
      if (final_mode != 1) {
        // Go THROUGH BALANCE_STAND (like MIT's operator flow): it lifts the body
        // from the ~0.20 m stand-up crouch to the 0.29 m locomotion height. Jumping
        // 1 -> 4 directly makes the MPC see a 9 cm height step and command ~2x
        // bodyweight stance forces (a leap) right as the gait starts.
        if (final_mode == 4 && !getenv("SIM_SKIP_BAL")) {
          usleep((t_bal - t_stand) * 1000000);
          _robotParams.control_mode = 3;              // K_BALANCE_STAND
          printf("[sim] control_mode -> BALANCE_STAND\n"); fflush(stdout);
          usleep((t_loco - t_bal) * 1000000);
        } else {
          usleep((t_loco - t_stand) * 1000000);
        }
        _robotParams.control_mode = final_mode;       // K_LOCOMOTION (4) etc.
        printf("[sim] control_mode -> %d\n", final_mode); fflush(stdout);
        // After the trot stabilizes, RAMP in the forward velocity (a 0 -> vx step
        // through the ~6-10 ms UDP loop knocked the trot over; ramping does not).
        // $SIM_VX target speed, $SIM_VX_RAMP_S ramp duration (default 3 s).
        // $SIM_WZ adds a YAW RATE command on the same schedule, so the robot can
        // be asked to turn while moving (cornering) or to spin in place with
        // zero forward velocity (a pirouette). Both channels ramp together: a
        // stepped yaw command knocks the gait over for the same reason a stepped
        // velocity command does.
        //
        // SIGN, and why this is NEGATED - the full chain, measured end to end:
        //   SIM_WZ -> rightStickAnalog[0] -> DesiredStateCommand.cpp does
        //   `joystickRight[0] *= -1` (UPSTREAM MIT, and correct: pushing a
        //   gamepad's right stick RIGHT should turn the robot RIGHT) ->
        //   rightAnalogStick[0] -> _yaw_turn_rate, which is CCW-POSITIVE.
        // So a raw stick value and the yaw rate it produces have OPPOSITE signs.
        // Passing SIM_WZ straight through meant `SIM_WZ=+1.0` printed as
        // `wzCmd=-1.000` in the logs and turned the robot clockwise, which cost
        // a full investigation chasing a "sign bug" that did not exist.
        // SIM_WZ is therefore defined as a YAW RATE in rad/s, CCW-positive -
        // the same convention as _yaw_turn_rate and as the [YAW]/[YAWREF] logs -
        // and it is negated HERE, once, to become a stick deflection.
        // VERIFIED not a controller bug: with SIM_WZ=1.0 the reference and the
        // measurement track each other with a CONSTANT 0.21 rad offset through
        // a full revolution and across the +-pi wrap - a healthy rate loop with
        // a steady-state offset, not a divergence.
        const bool haveVx = getenv("SIM_VX") && atof(getenv("SIM_VX")) != 0.0;
        const bool haveWz = getenv("SIM_WZ") && atof(getenv("SIM_WZ")) != 0.0;
        if (final_mode == 4 && (haveVx || haveWz)) {
          // Let the gait ENGAGE before asking it to go anywhere. The MPC port
          // holds MIT's standing gait for a short window after LOCOMOTION
          // entry (first async solution + settle), so a velocity ramp that
          // starts at mode-4 is already at 0.2-0.3 m/s when the trot actually
          // begins - and the first strides lunge (measured: 21 cm surge,
          // +23 deg pitch, roll-over). Real operators do the same thing with
          // the stick: stand, trot in place, then push forward.
          float delay_s = getenv("SIM_VX_DELAY_S") ? atof(getenv("SIM_VX_DELAY_S")) : 3.f;
          printf("[sim] holding velocity at 0 for %.1f s while the gait engages\n", delay_s);
          fflush(stdout);
          usleep((useconds_t)(delay_s * 1e6f));
          usleep(3000000);
          float vx = haveVx ? atof(getenv("SIM_VX")) : 0.f;
          float wz = haveWz ? atof(getenv("SIM_WZ")) : 0.f;
          float ramp_s = getenv("SIM_VX_RAMP_S") ? atof(getenv("SIM_VX_RAMP_S")) : 3.f;
          printf("[sim] ramping vx -> %.2f m/s, wz -> %.2f rad/s over %.1f s\n",
                 vx, wz, ramp_s);
          fflush(stdout);
          const int steps = (int)(ramp_s * 10.f);
          for (int s = 1; s <= steps; ++s) {
            _gamepadCommand.leftStickAnalog[1]  =  vx * s / steps;
            _gamepadCommand.rightStickAnalog[0] = -wz * s / steps;   // see note
            usleep(100000);
          }
          _gamepadCommand.leftStickAnalog[1]  =  vx;
          _gamepadCommand.rightStickAnalog[0] = -wz;
          printf("[sim] command -> vx=%.2f m/s wz=%.2f rad/s\n", vx, wz); fflush(stdout);

          // $SIM_PROFILE: drive SPEED and GAIT changes DURING a run, to test
          // that the parameter scheduler transitions seamlessly rather than
          // stepping a running gait. Format "t:vx:gait;t:vx:gait;..." with t in
          // seconds from the end of the ramp; gait <0 leaves the gait alone.
          // Speed is ramped between steps (a stepped velocity knocks the gait
          // over, which would test the step and not the scheduler); the gait
          // switch itself is instantaneous because that is what an operator or
          // a planner actually does.
          // NOTE: do NOT set SIM_GAIT for profile runs - it is read once into a
          // static and would pin the gait, masking every switch.
          if (const char* prof = getenv("SIM_PROFILE")) {
            std::string s(prof);
            size_t pos = 0;
            float tPrev = 0.f, vPrev = vx;
            while (pos < s.size()) {
              size_t end = s.find(';', pos);
              if (end == std::string::npos) end = s.size();
              float tt = 0, vv = 0; int gg = -1;
              if (sscanf(s.substr(pos, end - pos).c_str(), "%f:%f:%d", &tt, &vv, &gg) >= 2) {
                float wait = tt - tPrev;
                if (wait > 0) usleep((useconds_t)(wait * 1e6f));
                if (gg >= 0 && _userControlParameters) {
                  ControlParameterValue cv; cv.d = (double)gg;
                  _userControlParameters->collection.lookup("cmpc_gait")
                      .set(cv, ControlParameterValueKind::DOUBLE);
                  printf("[profile] t=%.1fs GAIT -> %d\n", tt, gg);
                }
                // ramp the speed over 2 s so the scheduler, not a step, is what
                // is under test
                const int st = 20;
                for (int k = 1; k <= st; ++k) {
                  _gamepadCommand.leftStickAnalog[1] = vPrev + (vv - vPrev) * k / st;
                  usleep(100000);
                }
                _gamepadCommand.leftStickAnalog[1] = vv;
                printf("[profile] t=%.1fs vx -> %.2f\n", tt, vv); fflush(stdout);
                vPrev = vv; tPrev = tt + 2.f;
              }
              pos = end + 1;
            }
          }
        }
      }
    }).detach();
  }

  for (;;) {
    usleep(1000000);
    if (_robotRunner) {
      printf("[stm32mp1] ctrl loop: maxRuntime=%.2f ms  maxPeriod=%.2f ms  (period target %.1f ms)\n",
             _robotRunner->getMaxRuntime() * 1000.f,
             _robotRunner->getMaxPeriod() * 1000.f,
             _robotParams.controller_dt * 1000.f);
      fflush(stdout);
      _robotRunner->clearMax();
    }
  }
}

  // linux
