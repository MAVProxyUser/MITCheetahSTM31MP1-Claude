/*!
 * @file Stm32mp1HardwareBridge.h
 * @brief Headless hardware bridge for the Octavo OSD32MP1 port.
 *
 * Self-contained replacement for MiniCheetahHardwareBridge: no SPI spine, no
 * EtherCAT, no VectorNav, no operator LCM/GUI. It wires the Unitree RS485 driver
 * (actuators) and the CAN IMU driver (state) into the stock RobotRunner control
 * loop, loads parameters from YAML, and runs headless.
 */
#ifndef PROJECT_STM32MP1_HARDWAREBRIDGE_H
#define PROJECT_STM32MP1_HARDWAREBRIDGE_H

// Available on every platform: the GAZEBO backend is pure UDP and is exactly
// what the Mac-first workflow runs natively. Linux-only pieces (RS485 motors,
// CAN IMU, mlockall/SCHED_FIFO) are guarded inside the .cpp.
#include <string>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/VisualizationData.h"
#include "SimUtilities/SpineBoard.h"       // SpiData / SpiCommand
#include "SimUtilities/IMUTypes.h"         // VectorNavData
#include "ControlParameters/RobotParameters.h"
#include "rt/rt_unitree.h"
#include "rt/rt_can_imu.h"
#include "rt/rt_gazebo.h"

class Stm32mp1HardwareBridge {
 public:
  //! I/O backend: real hardware (CAN IMU + Unitree RS485) or a Gazebo SITL over UDP.
  enum class Backend { HARDWARE, GAZEBO };

  Stm32mp1HardwareBridge(RobotController* controller,
                         std::string robot_yaml,
                         std::string user_yaml,
                         Backend backend = Backend::HARDWARE);
  void run();

  //! Configure the Gazebo UDP peer (workstation running the bridge) for GAZEBO backend.
  void setGazebo(const GazeboUdpConfig& cfg) { _gazeboCfg = cfg; }
  //! The robot's TRUE WORLD yaw (Gazebo/SDF convention) at spawn - needed to
  //! rotate world-frame GPS velocity into the KF's own frame before feeding
  //! it to $SIM_VEL_AIDING. See CLAUDE.md "GPS VELOCITY AIDING" for why this
  //! rotation is required: VectorNavOrientationEstimator zeroes yaw to the
  //! robot's OWN spawn heading on the first tick, so the KF's internal
  //! x/y axes are NOT world east/north except when spawn yaw happens to be
  //! zero. mit_sim_main.cpp knows this value (mission_spawn_yaw_rad, the
  //! SAME value used to actually build the SDF spawn pose); the hardware
  //! bridge does not, by design (it is mission-agnostic), so it must be
  //! told once at startup.
  void setSpawnYawRad(float yaw) { _spawnYawRad = yaw; }
  //! Drive the FSM from outside the sequencer - used by the mission's
  //! end-of-run lie-down. K_BALANCE_STAND / K_STAND_UP / K_PASSIVE.
  void setControlMode(int m) { _robotParams.control_mode = m; }
  int  getControlMode() const { return (int)_robotParams.control_mode; }
  //! User parameters, so a mission can retune the controller at runtime
  //! (the gait decider changes cmpc_gait through this).
  ControlParameters* userParams() { return _userControlParameters; }

  //! periodic actuator exchange (RS485 motors or Gazebo UDP), public for PeriodicMemberFunction
  void runMotors();

  /*! Operator command channel - the same two sticks a gamepad or the sequencer
   *  drives. Exposed so a SITL main can close a waypoint loop around the
   *  controller without reaching into the FSM (see mit_sim_main.cpp). */
  GamepadCommand& driverCommand() { return _gamepadCommand; }
  /*! May be null until run() has built it. */
  RobotRunner* robotRunner() const { return _robotRunner; }

 private:
  void setupScheduler();
  void initHardware();

  RobotController*    _controller = nullptr;
  ControlParameters*  _userControlParameters = nullptr;
  std::string         _robotYaml, _userYaml;

  PeriodicTaskManager _taskManager;
  GamepadCommand      _gamepadCommand;
  VisualizationData   _visualizationData;
  CheetahVisualization _mainCheetahVisualization;

  SpiData             _spiData;
  SpiCommand          _spiCommand;
  VectorNavData       _vectorNavData;
  CheaterState<double> _cheaterState;   //!< sim ground truth, INSTRUMENTATION ONLY - no
                                        //!< estimator consumes it; cheater mode is deleted
  //! Absolute position aiding for the KF (baro/GPS). On the real Go1 these
  //! arrive over CAN; in SITL over UDP. Opt-in via $SIM_ABS_AIDING.
  AbsolutePositionAiding<float> _absAiding;
  // Initialized from $WP_SPAWN_BEARING_DEG in the constructor body (the
  // same env var server.py already passes for every shifted-spawn mission)
  // rather than trusting this pi/2 default until navThread gets around to
  // setSpawnYawRad(): with velocity aiding DEFAULT-ON, corrections fire
  // from tick 1, and the whole boot/stand/engage sequence used to run them
  // rotated for a NORTH spawn regardless of the real heading. Harmless at
  // bearing 0; at the star's 162 deg spawn it was a 126-deg-wrong velocity
  // injection during the most fragile transition, and the dog tipped and
  // ESTOPped at engagement (measured, first fast-suite run after the
  // aiding default flipped). setSpawnYawRad() remains the authoritative
  // late set; with the env read it is a no-op when they agree.
  float _spawnYawRad = 1.5707963f;  // pi/2 - default matches every world file's own
                                    // universal spawn convention (yaw=+90deg, body-x
                                    // north) when nobody calls the setter
  RobotControlParameters _robotParams;
  RobotRunner*        _robotRunner = nullptr;

  Backend             _backend;
  UnitreeConfig       _unitreeCfg;
  CanImuConfig        _canImuCfg;
  GazeboUdpConfig     _gazeboCfg;

  // driver-side scratch (identical layout to SpiCommand/SpiData)
  spi_command_t       _utCmd;
  spi_data_t          _utData;
};

#endif  // PROJECT_STM32MP1_HARDWAREBRIDGE_H
