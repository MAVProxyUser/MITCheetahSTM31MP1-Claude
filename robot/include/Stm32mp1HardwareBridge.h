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

#ifdef linux

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

  //! periodic actuator exchange (RS485 motors or Gazebo UDP), public for PeriodicMemberFunction
  void runMotors();

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
  CheaterState<double> _cheaterState;   //!< sim ground truth (SIM_CHEATER=1, GAZEBO backend)
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

#endif  // linux
#endif  // PROJECT_STM32MP1_HARDWAREBRIDGE_H
