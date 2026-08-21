/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Locomotion.h"
#include <cstdlib>
#include <Utilities/Timer.h>
#include <algorithm>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
//#include <rt/rt_interface_lcm.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  if(_controlFSMData->_quadruped->_robotType == RobotType::MINI_CHEETAH){
    double mpc_ms = 27;
#ifdef USE_GO1_MODEL
    // Go1 SITL: $SIM_MPC_MS stretches the gait segment clock (ms). The stock
    // 27 ms segments give a 260 ms trot cycle tuned for MIT's ~2 ms loop; the
    // UDP SITL loop is ~8 ms, so a slower cycle (e.g. 40 -> 400 ms) buys the
    // swing/touchdown dynamics real phase margin.
    if (const char* e = getenv("SIM_MPC_MS")) {
      double v = atof(e);
      if (v >= 10 && v <= 80) mpc_ms = v;
    }
#endif
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
        //30 / (1000. * _controlFSMData->controlParameters->controller_dt),
        //22 / (1000. * _controlFSMData->controlParameters->controller_dt),
        mpc_ms / (1000. * _controlFSMData->controlParameters->controller_dt),
        _controlFSMData->userParameters);

  }else if(_controlFSMData->_quadruped->_robotType == RobotType::CHEETAH_3){
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
        33 / (1000. * _controlFSMData->controlParameters->controller_dt),
        _controlFSMData->userParameters);

  }else{
    assert(false);
  }


  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();
}

template <typename T>
void FSM_State_Locomotion<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
  cMPCOld->initialize();
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
  printf("[FSM LOCOMOTION] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Locomotion<T>::run() {
  // Call the locomotion control logic for this iteration
  LocomotionControlStep();
}

extern rc_control_settings rc_control;

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition() {
  // Get the next state
  iter++;

  // Switch FSM control mode
  if(locomotionSafe()) {
    switch ((int)this->_data->controlParameters->control_mode) {
      case K_LOCOMOTION:
        break;

      case K_BALANCE_STAND:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::BALANCE_STAND;

        // Transition time is immediate
        this->transitionDuration = 0.0;

        break;

      case K_PASSIVE:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::PASSIVE;

        // Transition time is immediate
        this->transitionDuration = 0.0;

        break;

      case K_STAND_UP:
        this->nextStateName = FSM_StateName::STAND_UP;
        this->transitionDuration = 0.;
        break;

      case K_RECOVERY_STAND:
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        this->transitionDuration = 0.;
        break;

      case K_VISION:
        this->nextStateName = FSM_StateName::VISION;
        this->transitionDuration = 0.;
        break;

      default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_LOCOMOTION << " to "
                  << this->_data->controlParameters->control_mode << std::endl;
    }
  } else {
    this->nextStateName = FSM_StateName::RECOVERY_STAND;
    this->transitionDuration = 0.;
    rc_control.mode = RC_mode::RECOVERY_STAND;
  }


  // Return the next state name to the FSM
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
    case FSM_StateName::BALANCE_STAND:
      LocomotionControlStep();

      iter++;
      if (iter >= this->transitionDuration * 1000) {
        this->transitionData.done = true;
      } else {
        this->transitionData.done = false;
      }

      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();

      this->transitionData.done = true;

      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;


    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

template<typename T>
bool FSM_State_Locomotion<T>::locomotionSafe() {
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if(std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for(int leg = 0; leg < 4; leg++) {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if(p_leg[2] > 0) {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    // Lateral foot limit. TWO fixes here:
    //
    // 1) GEOMETRY. 0.18 m is mini-cheetah's (abad link 0.062 m). The Go1's abad
    //    link is 0.08 m, so its feet legitimately stand ~30% wider and a Go1
    //    trot trips this on the rear legs within ~1 s of gait entry. Tripping
    //    it is not benign: locomotionSafe() failing sends the FSM to
    //    RECOVERY_STAND, which FOLDS ALL FOUR LEGS. Every "the MPC tumbles at
    //    gait start" report in this port was this check firing, not a dynamics
    //    problem.
    // 2) UPSTREAM TYPO. `std::fabs(p_leg[1] > 0.18)` takes fabs of a *bool* -
    //    it is 0 or 1, so the test reduces to `p_leg[1] > 0.18` and the
    //    negative side is never checked at all. Parenthesis moved.
#ifdef USE_GO1_MODEL
    const T max_pleg_y = 0.24;   // 0.18 * (Go1 abad 0.08 / mini-cheetah 0.062)
#else
    const T max_pleg_y = 0.18;
#endif
    if(std::fabs(p_leg[1]) > max_pleg_y) {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m, max %.3f)\n",
             leg, p_leg[1], max_pleg_y);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if(std::fabs(v_leg) > 9.) {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;

}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Locomotion<T>::onExit() {
  // Nothing to clean up when exiting
  iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep() {
  // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Contact state logic
  // estimateContact();

  cMPCOld->run<T>(*this->_data);
  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  for(int leg(0); leg<4; ++leg){
    pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;
  }

  if(this->_data->userParameters->use_wbc > 0.9){
    _wbc_data->pBody_des = cMPCOld->pBody_des;
    _wbc_data->vBody_des = cMPCOld->vBody_des;
    _wbc_data->aBody_des = cMPCOld->aBody_des;

    _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;
    _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;
    
    for(size_t i(0); i<4; ++i){
      _wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];
      _wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];
      _wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];
      _wbc_data->Fr_des[i] = cMPCOld->Fr_des[i]; 
    }
    _wbc_data->contact_state = cMPCOld->contact_state;

    // WBC DECIMATION.
    // The factory Go1 runs this every tick, but it has 4x Cortex-A72 at
    // 1.5 GHz on a PREEMPT_RT kernel - roughly 11x this board's 2x Cortex-A7 at
    // 650 MHz. Here WBIC costs ~50 ms against a 2 ms control period, so running
    // it every tick is impossible and running it NOT AT ALL loses BodyPosTask /
    // BodyOriTask, which are the only thing servoing body pose once
    // Kp_stance = 0 (measured: body parks at 0.204 m against a 0.30 m
    // reference). So run it every Nth tick and hold its joint commands in
    // between - the WBC writes qDes/qdDes/kp/kd/tau, all of which the leg
    // controller keeps applying on the ticks it is skipped.
    // $SIM_WBC_DECIM (default 1 = stock every-tick behaviour).
    static const int wbc_decim = getenv("SIM_WBC_DECIM")
                               ? std::max(1, atoi(getenv("SIM_WBC_DECIM"))) : 1;
    static int wbc_tick = 0;
    // CACHE AND RE-APPLY between WBC runs. The original decimation assumed the
    // leg controller "keeps applying" the WBC's commands on skipped ticks - it
    // does not: RobotRunner::setupStep() calls zeroCommand() EVERY tick, so a
    // skipped tick sent all-zero gains and torques to the legs. At decim 2
    // that is a 250 Hz full-command/zero-command chatter (visible in the
    // bridge dump as alternating healthy/zero packets), i.e. half the average
    // stiffness and force - and the robot folded at every gait engage while
    // the logs showed nothing wrong. Now the WBC's outputs are cached when it
    // runs and rewritten into the freshly-zeroed commands on the ticks it is
    // skipped, which is what the decimation was always meant to mean.
    static Vec3<T> c_qDes[4], c_qdDes[4], c_tau[4];
    static Mat3<T> c_kp[4], c_kd[4];
    static bool c_valid = false;
    if ((wbc_tick++ % wbc_decim) == 0) {
      _wbc_ctrl->run(_wbc_data, *this->_data);
      for (int leg = 0; leg < 4; ++leg) {
        auto& cmd = this->_data->_legController->commands[leg];
        c_qDes[leg] = cmd.qDes;   c_qdDes[leg] = cmd.qdDes;
        c_tau[leg]  = cmd.tauFeedForward;
        c_kp[leg]   = cmd.kpJoint; c_kd[leg] = cmd.kdJoint;
      }
      c_valid = true;
    } else if (c_valid) {
      for (int leg = 0; leg < 4; ++leg) {
        auto& cmd = this->_data->_legController->commands[leg];
        cmd.qDes = c_qDes[leg];   cmd.qdDes = c_qdDes[leg];
        cmd.tauFeedForward = c_tau[leg];
        cmd.kpJoint = c_kp[leg];  cmd.kdJoint = c_kd[leg];
      }
    }
  }
  for(int leg(0); leg<4; ++leg){
    //this->_data->_legController->commands[leg].pDes = pDes_backup[leg];
    this->_data->_legController->commands[leg].vDes = vDes_backup[leg];
    //this->_data->_legController->commands[leg].kpCartesian = Kp_backup[leg];
    this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
  }

}

/**
 * Stance leg logic for impedance control. Prevent leg slipping and
 * bouncing, as well as tracking the foot velocity during high speeds.
 */
template <typename T>
void FSM_State_Locomotion<T>::StanceLegImpedanceControl(int leg) {
  // Impedance control for the stance leg
  this->cartesianImpedanceControl(
      leg, this->footstepLocations.col(leg), Vec3<T>::Zero(),
      this->_data->controlParameters->stand_kp_cartesian,
      this->_data->controlParameters->stand_kd_cartesian);
}

// template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<float>;
