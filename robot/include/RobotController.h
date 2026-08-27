/*!
 * @file RobotController.h
 * @brief Parent class of user robot controllers.
 * This is an interface between the control code and the common hardware code
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Controllers/LegController.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/DesiredStateCommand.h"
#include "SimUtilities/VisualizationData.h"
#include "SimUtilities/GamepadCommand.h"

/*!
 * Parent class of user robot controllers
 */
class RobotController{
  friend class RobotRunner;
public:
  RobotController(){}
  virtual ~RobotController(){}

  virtual void initializeController() = 0;
/**
 * Called one time every control loop 
 */
  virtual void runController() = 0;
  virtual void updateVisualization() = 0;
  virtual ControlParameters* getUserControlParameters() = 0;
  virtual void Estop() {}
  //! True if the underlying FSM (if any) is latched in its own ESTOP mode -
  //! MIT's stock ControlFSM has NO path back to NORMAL from there on its
  //! own (safetyPreCheck() only ever SETS operatingMode to ESTOP, never
  //! clears it - a deliberate real-hardware fail-safe: no self-reset
  //! without something explicitly asking for one). Default false for any
  //! controller with no such concept. See MIT_Controller::isEstopped() and
  //! mit_sim_main.cpp's ESTOP-recovery sequence, which polls this to decide
  //! when to call Estop() (which, for MIT_Controller, re-initializes the
  //! FSM back to PASSIVE/NORMAL - already existed, just never had a reason
  //! to be called mid-mission before).
  virtual bool isEstopped() { return false; }

protected:
  Quadruped<float>* _quadruped = nullptr;
  FloatingBaseModel<float>* _model = nullptr;
  LegController<float>* _legController = nullptr;
  StateEstimatorContainer<float>* _stateEstimator = nullptr;
  StateEstimate<float>* _stateEstimate = nullptr;
  GamepadCommand* _driverCommand = nullptr;
  RobotControlParameters* _controlParameters = nullptr;
  DesiredStateCommand<float>* _desiredStateCommand = nullptr;

  VisualizationData* _visualizationData = nullptr;
  RobotType _robotType;
};

#endif
