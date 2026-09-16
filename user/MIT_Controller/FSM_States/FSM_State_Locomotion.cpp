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
#include "../../../gazebo/ShmTrace.h"   // per-tick/text SHM tracing - see that file's own header
#include "Utilities/CtrlTuning.h"
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
    // GAIT SEGMENT (ms). DEFAULT IS 22, NOT MIT's 27, and this is measured:
    // stance duration x commanded speed is the distance a stance foot must
    // sweep, against ~0.318 m of horizontal reach, so a faster gait needs a
    // SHORTER cycle. At 3.0 m/s, 22 ms crosses 100 m in 33.2 s while the old
    // 26-27 ms fails at 67 m; 18 ms fails too, so the optimum is interior.
    // 22 also works across 2.0-3.1 m/s, which makes it the right single
    // default. $CTRL_MPC_MS overrides for experiments.
    //
    // The optimum is also GAIT-dependent, so it is selected per gait from
    // measurement rather than left for a human to remember: trotRunning covers
    // 100 m in 24.8 s at 26 ms and FAILS at 32 m on 22 ms, while trotting is
    // the other way round. Keyed off the gait that will actually run ($SIM_GAIT
    // if set, else the yaml's cmpc_gait) so no environment variable is needed
    // to get a gait's own best segment.
    {
      int g = _controlFSMData->userParameters->cmpc_gait;
      if (const char* ge = getenv("SIM_GAIT")) { int v = atoi(ge); if (v >= 0) g = v; }
      switch (g) {
        case 5:  mpc_ms = 26; break;   // trotRunning - 40% duty, flight phase
        default: mpc_ms = 22; break;   // trotting / walking and the rest
      }
    }
    // STILL A GAP: it is SPEED-dependent too (trotting wants 26 below ~2.75 m/s
    // and 22 above), and nothing schedules it against the commanded velocity.
    // 22 crosses at every trotting speed measured from 2.0 to 3.1, so it is a
    // safe single value - but a controller that varies its own speed widely
    // should compute this rather than inherit it.
    if (const char* e = ctrl_tuning::raw("CTRL_MPC_MS")) {
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
  shmtrace::logf(0.0, "[FSM LOCOMOTION] On Enter");
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
        shmtrace::logf(0.0, "[CONTROL FSM] Bad Request: Cannot transition from %d to %d",
                       (int)K_LOCOMOTION, (int)this->_data->controlParameters->control_mode);
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
      shmtrace::logf(0.0, "[CONTROL FSM] Something went wrong in transition");
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
    shmtrace::logf(0.0, "Unsafe locomotion: roll is %.3f degrees (max %.3f)", (double)ori::rad2deg(seResult.rpy[0]), (double)max_roll);
    return false;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    shmtrace::logf(0.0, "Unsafe locomotion: pitch is %.3f degrees (max %.3f)", (double)ori::rad2deg(seResult.rpy[1]), (double)max_pitch);
    return false;
  }

  for(int leg = 0; leg < 4; leg++) {
    // ALL THREE PER-LEG CHECKS BELOW ARE DEBOUNCED (ISSUES OPEN-39, 2026-09-15
    // 21:50). They read INSTANTANEOUS kinematics (FK position, J*qd) and answer
    // with RECOVERY_STAND, which re-commands all four legs to a stand pose
    // mid-stride - a fall every time at cruise. Run 8315 is the case that forced
    // this: the leg-speed debounce (below) correctly absorbed a four-tick
    // 12.4 m/s spike inside a 464-samples/s second, and the run fell anyway
    // through THIS branch's un-debounced sibling, `leg 2's y-position is bad
    // (-0.240 m, max 0.240)` - 0 mm past the limit, repeating into
    // RECOVERY_STAND from a body still at 0.299 m. Every y-position line in the
    // archive grazes the limit (-0.240..-0.251 m against 0.240), which is what a
    // boundary graze looks like, not a leg swung out of the envelope. A genuine
    // violation persists; a graze or a one-tick sensor artefact does not. Knobs:
    // CTRL_LEGY_TRIP_TICKS / CTRL_HIP_TRIP_TICKS / CTRL_LEGV_TRIP_TICKS (each
    // default 5 ticks = 10 ms; 1 restores upstream's one-tick trip for an A/B).
    // In-tree precedent: MIT's orientation E-stop is debounced here too
    // (CTRL_ORIENT_HOLD_MS, 60 ms), for exactly this reason.
    static const int  hip_ticks  = (int)ctrl_tuning::num("CTRL_HIP_TRIP_TICKS", 5.0);
    static const int  legy_ticks = (int)ctrl_tuning::num("CTRL_LEGY_TRIP_TICKS", 5.0);
    static int        hip_over[4]  = {0, 0, 0, 0};
    static int        legy_over[4] = {0, 0, 0, 0};
    static int        kin_spikes_logged = 0;

    auto p_leg = this->_data->_legController->datas[leg].p;

    // A HELD SAMPLE PERSISTS PERFECTLY - so a tick count alone is not enough
    // (ISSUES OPEN-39, 2026-09-15 22:25). Run 8407 (the tier's star at 3.5 m/s)
    // fell in a second the stream delivered 404 samples/s: eight consecutive
    // ticks each reported leg 0 at EXACTLY 9.749 m/s and the foot at exactly
    // 0.000 m above hip - one frozen sample re-read, not a leg moving. The
    // debounce counted it as persistence and tripped at tick 5. Contrast run
    // 8315's genuine transient, absorbed correctly the same day: 10.064 ->
    // 10.978 -> 12.133 -> 12.389 m/s, a different value every tick. So the
    // discriminator is CHANGE, not duration: a tick whose leg state is
    // bit-for-bit identical to the previous tick carries no new evidence and
    // must not advance any of these counters. (Bit-for-bit is the right test
    // for exactly the reason the GPS staleness gate uses it - a zero-order-held
    // value IS bit-identical, while real float dynamics never repeat.) A
    // genuine runaway that held a constant speed to the last bit for 10 ms
    // would be missed; the attitude checks above and the debounced orientation
    // E-stop still catch a robot that is actually going over.
    // CTRL_LEG_HELD_GATE=0 disables this gate for an A/B.
    static const bool  held_gate = ctrl_tuning::flag("CTRL_LEG_HELD_GATE", true);
    static float       last_py[4] = {0, 0, 0, 0}, last_pz[4] = {0, 0, 0, 0}, last_v[4] = {0, 0, 0, 0};
    static bool        last_valid[4] = {false, false, false, false};
    static int         held_logged = 0;
    const float py_now = (float)p_leg[1], pz_now = (float)p_leg[2];
    const float v_now  = (float)this->_data->_legController->datas[leg].v.norm();
    const bool  held   = held_gate && last_valid[leg] && py_now == last_py[leg] && pz_now == last_pz[leg] && v_now == last_v[leg];
    last_py[leg] = py_now; last_pz[leg] = pz_now; last_v[leg] = v_now; last_valid[leg] = true;
    if(held) {
      if(held_logged < 50) {
        held_logged++;
        shmtrace::logf(0.0, "[leghold] leg %d state unchanged (y %.3f z %.3f |v| %.3f) - a held sample, not counted toward any trip", leg, (double)py_now, (double)pz_now, (double)v_now);
      }
      continue;   // no new evidence about this leg this tick
    }
    if(p_leg[2] > 0) {
      hip_over[leg]++;
      if(hip_over[leg] >= hip_ticks) {
        shmtrace::logf(0.0, "Unsafe locomotion: leg %d is above hip (%.3f m, %d ticks)", leg, (double)p_leg[2], hip_over[leg]);
        return false;
      }
      if(kin_spikes_logged < 50) {
        kin_spikes_logged++;
        shmtrace::logf(0.0, "[legkin] leg %d above hip (%.3f m) for %d tick(s) - not a trip until %d", leg, (double)p_leg[2], hip_over[leg], hip_ticks);
      }
    } else {
      hip_over[leg] = 0;
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
      legy_over[leg]++;
      if(legy_over[leg] >= legy_ticks) {
        shmtrace::logf(0.0, "Unsafe locomotion: leg %d's y-position is bad (%.3f m, max %.3f, %d ticks)",
               leg, (double)p_leg[1], (double)max_pleg_y, legy_over[leg]);
        return false;
      }
      if(kin_spikes_logged < 50) {
        kin_spikes_logged++;
        shmtrace::logf(0.0, "[legkin] leg %d y-position %.3f m (max %.3f) for %d tick(s) - not a trip until %d",
               leg, (double)p_leg[1], (double)max_pleg_y, legy_over[leg], legy_ticks);
      }
    } else {
      legy_over[leg] = 0;
    }

    // LEG-SPEED TRIP, DEBOUNCED (ISSUES OPEN-39 / OPEN-28, 2026-09-15 03:50).
    // Upstream trips on ONE tick over 9 m/s. datas[leg].v is J * qd, and qd
    // here is the sim's joint velocity, which spikes for a tick or two on a
    // touchdown impact and on a sensor-stream gap (a held sample, then a
    // jump). The response - RECOVERY_STAND - re-commands all four legs to a
    // stand pose in the middle of a stride, which at cruise is a fall every
    // time: of 189 runs archived on the night of 2026-09-14/15, 8 logged this
    // line and 8 fell (two with a clean stream, run 7680 at 1.9 m/s on the
    // hairpin's closing leg and run 7763 at 3.6 m/s on the star's dash, the
    // rest inside OPEN-39 sample-deficit seconds). A genuine runaway leg stays
    // over the limit; a spike does not. CTRL_LEGV_TRIP_TICKS (default 5, i.e.
    // 10 ms) consecutive ticks over CTRL_LEGV_TRIP_MPS (default 9, upstream)
    // are required; 1 restores the stock one-tick trip for an A/B. Ignored
    // spikes are logged (throttled) so the record can count them.
    static const int  legv_ticks = (int)ctrl_tuning::num("CTRL_LEGV_TRIP_TICKS", 5.0);
    static const T    legv_mps   = (T)ctrl_tuning::num("CTRL_LEGV_TRIP_MPS", 9.0);
    static int        legv_over[4] = {0, 0, 0, 0};
    static int        legv_spikes_logged = 0;
    const auto v_leg = v_now;   // computed once above for the held-sample gate
    if(std::fabs(v_leg) > legv_mps) {
      legv_over[leg]++;
      if(legv_over[leg] >= legv_ticks) {
        shmtrace::logf(0.0, "Unsafe locomotion: leg %d is moving too quickly (%.3f m/s, %d ticks over %.1f)", leg, (double)v_leg, legv_over[leg], (double)legv_mps);
        return false;
      }
      if(legv_spikes_logged < 50) {
        legv_spikes_logged++;
        shmtrace::logf(0.0, "[legv] leg %d over %.1f m/s for %d tick(s) (%.3f m/s) - not a trip until %d", leg, (double)legv_mps, legv_over[leg], (double)v_leg, legv_ticks);
      }
    } else {
      legv_over[leg] = 0;
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
    // $CTRL_WBC_DECIM (default 1 = stock every-tick behaviour).
    static const int wbc_decim = getenv("CTRL_WBC_DECIM")
                               ? std::max(1, atoi(getenv("CTRL_WBC_DECIM"))) : 1;
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
