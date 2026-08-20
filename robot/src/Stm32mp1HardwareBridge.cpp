/*!
 * @file Stm32mp1HardwareBridge.cpp
 * @brief Headless hardware bridge for the Octavo OSD32MP1 port. See the header.
 */
#ifdef linux

#include "Stm32mp1HardwareBridge.h"

#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <thread>

#include "rt/rt_rc_interface.h"

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
  _unitreeCfg = unitree_default_config();   // A1, 4 buses; override before run() as needed
  // _canImuCfg defaults: can0, node 124, msgs 20500/20501
  memset(&_spiData, 0, sizeof(_spiData));
  memset(&_spiCommand, 0, sizeof(_spiCommand));
  memset(&_utCmd, 0, sizeof(_utCmd));
  memset(&_utData, 0, sizeof(_utData));
}

void Stm32mp1HardwareBridge::setupScheduler() {
  // Best-effort real-time. On the board run as root for this to take effect;
  // during bring-up we warn and continue rather than abort.
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    printf("[stm32mp1] mlockall failed (run as root for RT determinism); continuing\n");
  struct sched_param params;
  params.sched_priority = 49;
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1)
    printf("[stm32mp1] SCHED_FIFO failed (run as root for RT determinism); continuing\n");
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
  if (init_can_imu(_canImuCfg, &_vectorNavData) != 0)
    printf("[stm32mp1] CAN IMU init failed; estimator will run on a stale/identity IMU\n");

  printf("[stm32mp1] init Unitree RS485 (%d bus(es))\n", _unitreeCfg.num_buses);
  if (init_unitree(_unitreeCfg) != 0)
    printf("[stm32mp1] Unitree RS485 init failed; motors will not respond\n");
}

void Stm32mp1HardwareBridge::runMotors() {
  // RobotRunner has written _spiCommand; ship it and read back _spiData.
  // SpiCommand/spi_command_t (and SpiData/spi_data_t) are layout-identical (asserted above).
  memcpy(&_utCmd, &_spiCommand, sizeof(_utCmd));
  if (_backend == Backend::GAZEBO)
    gazebo_send_receive(&_utCmd, &_utData);
  else
    unitree_send_receive(&_utCmd, &_utData);
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
      int t_loco  = getenv("SIM_LOCO_S")  ? atoi(getenv("SIM_LOCO_S"))  : 9;
      usleep(t_stand * 1000000);
      _robotParams.control_mode = 1;                  // K_STAND_UP
      printf("[sim] control_mode -> STAND_UP\n"); fflush(stdout);
      if (final_mode != 1) {
        usleep((t_loco - t_stand) * 1000000);
        _robotParams.control_mode = final_mode;       // K_LOCOMOTION (4) etc.
        printf("[sim] control_mode -> %d\n", final_mode); fflush(stdout);
        // After the trot stabilizes, command a forward velocity (open-loop test).
        // $SIM_VX sets forward speed on the gamepad's left stick (0 = walk in place).
        if (final_mode == 4 && getenv("SIM_VX")) {
          usleep(3000000);
          float vx = atof(getenv("SIM_VX"));
          _gamepadCommand.leftStickAnalog[1] = vx;
          printf("[sim] forward velocity command -> %.2f\n", vx); fflush(stdout);
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

#endif  // linux
