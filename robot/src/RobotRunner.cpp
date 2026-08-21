/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#ifdef USE_GO1_MODEL
#include "Dynamics/Go1.h"
#endif
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"

RobotRunner::RobotRunner(RobotController* robot_ctrl, 
    PeriodicTaskManager* manager, 
    float period, std::string name):
  PeriodicTask(manager, period, name),
  _lcm(getLcmUrl(255)) {

    _robot_ctrl = robot_ctrl;
  }

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init() {
  printf("[RobotRunner] initialize\n");

  // Build the appropriate Quadruped object
  if (robotType == RobotType::MINI_CHEETAH) {
#ifdef USE_GO1_MODEL
    _quadruped = buildGo1<float>();       // Unitree Go1 kinematics/inertials
#else
    _quadruped = buildMiniCheetah<float>();
#endif
  } else {
    _quadruped = buildCheetah3<float>();
  }

  // Initialize the model and robot data
  _model = _quadruped.buildModel();
  _jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(
      cheaterState, vectorNavData, _legController->datas,
      &_stateEstimate, controlParameters);
  initializeStateEstimator(false);

  memset(&rc_control, 0, sizeof(rc_control_settings));
  // Initialize the DesiredStateCommand object
  _desiredStateCommand =
    new DesiredStateCommand<float>(driverCommand,
        &rc_control,
        controlParameters,
        &_stateEstimate,
        controlParameters->controller_dt);

  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  _robot_ctrl->_visualizationData= visualizationData;
  _robot_ctrl->_robotType = robotType;
  _robot_ctrl->_driverCommand = driverCommand;
  _robot_ctrl->_controlParameters = controlParameters;
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand;

  _robot_ctrl->initializeController();

}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run() {
  // Run the state estimator step
  //_stateEstimator->run(cheetahMainVisualization);
  _stateEstimator->run();
  //cheetahMainVisualization->p = _stateEstimate.position;
  visualizationData->clear();

  // NaN GUARD ON THE STATE ESTIMATE.
  // LinearKFPositionVelocityEstimator carries a covariance that it propagates
  // every tick; when the gait gives it long all-swing windows (galloping and
  // pronking both have them) the position update is unobservable, the
  // covariance blows up and the whole estimate goes non-finite. From there the
  // NaN travels straight into the MPC - measured `p=nan nan nan v=nan nan nan`
  // going into update_problem_data_floats - and out again as NaN joint torques,
  // which Gazebo rejects and which are undefined behaviour on real hardware.
  // Reinitialising the estimator is far better than propagating NaN.
  {
    const auto& r = _stateEstimate;
    bool bad = false;
    for (int i = 0; i < 3 && !bad; ++i)
      if (!std::isfinite(r.position[i]) || !std::isfinite(r.vWorld[i]) ||
          !std::isfinite(r.vBody[i])    || !std::isfinite(r.rpy[i])) bad = true;
    for (int i = 0; i < 4 && !bad; ++i)
      if (!std::isfinite(r.orientation[i])) bad = true;
    if (bad) {
      static int nanCount = 0;
      ++nanCount;
      printf("[stm32mp1] STATE ESTIMATE WENT NON-FINITE (%d) - reinitialising\n", nanCount);
      fflush(stdout);
      initializeStateEstimator(_cheaterModeEnabled);
      _stateEstimator->run();
    }
  }

  // FALL DETECTOR. MIT's ControlFSM already E-stops on attitude
  // (safetyPreCheck -> 0.5 rad -> PASSIVE), but the PROCESS keeps running, so a
  // run that ended on its side still burns the whole timeout before the harness
  // moves on. Attitude is the right discriminator - a legged robot takes a real
  // acceleration spike on every footfall, so acceleration cannot separate
  // "trotting hard" from "fallen over", whereas a working quadruped is never on
  // its side or its face. Threshold is deliberately well beyond MIT's E-stop so
  // this only fires on a genuine fall, never on a hard corner.
  // $SIM_FALL_DEG (default 50), $SIM_FALL_HOLD_S (0.5), $SIM_FALL_EXIT=0 to disable.
  {
    static const bool fall_exit =
        !getenv("SIM_FALL_EXIT") || atoi(getenv("SIM_FALL_EXIT")) != 0;
    static const float fall_rad =
        (getenv("SIM_FALL_DEG") ? atof(getenv("SIM_FALL_DEG")) : 50.f) * float(M_PI) / 180.f;
    static const float fall_hold =
        getenv("SIM_FALL_HOLD_S") ? atof(getenv("SIM_FALL_HOLD_S")) : 0.5f;
    static float fallen_for = 0.f;
    if (fall_exit) {
      // Attitude catches a TIP-OVER. It does not catch a COLLAPSE: when a gait
      // fails by splaying its legs the body sinks to the deck while staying
      // roughly level, so roll and pitch never trip and the run burns its whole
      // timeout on a robot that is already down (measured: trot@1.0 and both
      // trotRunning runs "fell" at 24 s and still ran to 42 s). So also test
      // sustained low body height - armed only after the robot has actually
      // stood up once, since it starts belly-down on the deck by design.
      static const float fall_z =
          getenv("SIM_FALL_Z") ? atof(getenv("SIM_FALL_Z")) : 0.15f;
      static bool stood = false;
      const float bodyZ = _stateEstimate.position[2];
      if (bodyZ > 0.22f) stood = true;

      const float roll  = std::fabs(_stateEstimate.rpy[0]);
      const float pitch = std::fabs(_stateEstimate.rpy[1]);
      const bool tipped    = (roll > fall_rad || pitch > fall_rad);
      const bool collapsed = (stood && std::isfinite(bodyZ) && bodyZ < fall_z);
      if (tipped || collapsed)
        fallen_for += controlParameters->controller_dt;
      else
        fallen_for = 0.f;
      if (fallen_for >= fall_hold) {
        printf("[FALL] %s: roll=%.0f deg pitch=%.0f deg z=%.3f m held %.2f s - "
               "robot is down, stopping (legs go limp via the bridge watchdog)\n",
               tipped ? "tipped over" : "collapsed",
               _stateEstimate.rpy[0] * 57.2958f, _stateEstimate.rpy[1] * 57.2958f,
               bodyZ, fallen_for);
        fflush(stdout);
        for (int leg = 0; leg < 4; leg++) _legController->commands[leg].zero();
        finalizeStep();
        // _exit, not exit: this runs on the control thread while the MPC worker
        // and the UDP threads are still live, and running static destructors
        // underneath them aborts (SIGABRT) instead of exiting cleanly. Flush
        // first, since _exit does not.
        fflush(nullptr);
        _exit(0);
      }
    }
  }

  // STM32MP1 SITL debug: throttled estimator dump so we can compare the
  // estimated body state to Gazebo ground truth (set STM32MP1_EST_DBG=1).
  if (getenv("STM32MP1_EST_DBG")) {
    static int _estdbg = 0;
    if ((++_estdbg % 25) == 0) {   // 20 Hz at 500 Hz
      printf("[EST] rpy=%.3f %.3f %.3f pos=%.3f %.3f %.3f vB=%.3f %.3f %.3f wB=%.3f %.3f %.3f\n",
             _stateEstimate.rpy[0], _stateEstimate.rpy[1], _stateEstimate.rpy[2],
             _stateEstimate.position[0], _stateEstimate.position[1], _stateEstimate.position[2],
             _stateEstimate.vBody[0], _stateEstimate.vBody[1], _stateEstimate.vBody[2],
             _stateEstimate.omegaBody[0], _stateEstimate.omegaBody[1], _stateEstimate.omegaBody[2]);
      fflush(stdout);
    }
  }

  // Update the data from the robot
  setupStep();

  static int count_ini(0);
  ++count_ini;
  if (count_ini < 10) {
    _legController->setEnabled(false);
  } else if (20 < count_ini && count_ini < 30) {
    _legController->setEnabled(false);
  } else if (40 < count_ini && count_ini < 50) {
    _legController->setEnabled(false);
  } else {
    _legController->setEnabled(true);

    if( (rc_control.mode == 0) && controlParameters->use_rc ) {
      if(count_ini%1000 ==0)   printf("ESTOP!\n");
      for (int leg = 0; leg < 4; leg++) {
        _legController->commands[leg].zero();
      }
      _robot_ctrl->Estop();
    }else {
      // Controller
      if (!_jpos_initializer->IsInitialized(_legController)) {
        Mat3<float> kpMat;
        Mat3<float> kdMat;
        // Update the jpos feedback gains
        if (robotType == RobotType::MINI_CHEETAH) {
          kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
          kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
        } else if (robotType == RobotType::CHEETAH_3) {
          kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
          kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        } else {
          assert(false);
        } 

        for (int leg = 0; leg < 4; leg++) {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
      } else {
        // Run Control
        _robot_ctrl->runController();

        // STM32MP1 SITL debug: dump what the controller commands the front
        // legs (WBC output), to separate "kinWBC folds the legs" from
        // "WBIC/QP starves force" from "joint gains never set".
        if (getenv("STM32MP1_EST_DBG")) {
          static int _legdbg = 0;
          if ((++_legdbg % 25) == 0) {
            for (int leg = 0; leg < 4; ++leg) {
              auto& c = _legController->commands[leg];
              auto& d = _legController->datas[leg];
              printf("[LEG%d] q=%.2f %.2f %.2f qd=%.1f %.1f %.1f qdDes=%.1f %.1f %.1f "
                     "fff=%.1f %.1f %.1f tff=%.2f %.2f %.2f\n",
                     leg, d.q[0], d.q[1], d.q[2], d.qd[0], d.qd[1], d.qd[2],
                     c.qdDes[0], c.qdDes[1], c.qdDes[2],
                     c.forceFeedForward[0], c.forceFeedForward[1], c.forceFeedForward[2],
                     c.tauFeedForward[0], c.tauFeedForward[1], c.tauFeedForward[2]);
            }
            fflush(stdout);
          }
        }
        cheetahMainVisualization->p = _stateEstimate.position;

        // Update Visualization
        _robot_ctrl->updateVisualization();
        cheetahMainVisualization->p = _stateEstimate.position;
      }
    }

  }



  // Visualization (will make this into a separate function later)
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      cheetahMainVisualization->q[leg * 3 + joint] =
        _legController->datas[leg].q[joint];
    }
  }
  cheetahMainVisualization->p.setZero();
  cheetahMainVisualization->p = _stateEstimate.position;
  cheetahMainVisualization->quat = _stateEstimate.orientation;

  // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep() {
  // Update the leg data
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateData(spiData);
  } else if (robotType == RobotType::CHEETAH_3) {
    _legController->updateData(tiBoardData);
  } else {
    assert(false);
  }

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);

  // state estimator
  // check transition to cheater mode:
  if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);
    // todo any configuration
    _cheaterModeEnabled = true;
  }

  // check transition from cheater mode:
  if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);
    // todo any configuration
    _cheaterModeEnabled = false;
  }

  get_rc_control_settings(&rc_control);

  // todo safety checks, sanity checks, etc...
}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
void RobotRunner::finalizeStep() {
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateCommand(spiCommand);
  } else if (robotType == RobotType::CHEETAH_3) {
    _legController->updateCommand(tiBoardCommand);
  } else {
    assert(false);
  }
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
  _stateEstimate.setLcm(state_estimator_lcm);
  _lcm.publish("leg_control_command", &leg_control_command_lcm);
  _lcm.publish("leg_control_data", &leg_control_data_lcm);
  _lcm.publish("state_estimator", &state_estimator_lcm);
  _iterations++;
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);
  if (cheaterMode) {
    _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
    _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
  } else {
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
  }
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
  delete _jpos_initializer;
}

void RobotRunner::cleanup() {}
