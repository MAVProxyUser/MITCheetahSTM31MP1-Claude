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
#include <atomic>

// ISSUES OPEN-40 (2026-09-16): locomotionSafe() answering with RECOVERY_STAND is
// not a recovery - it is a 500 Hz LIMIT CYCLE, because
// FSM_State_RecoveryStand::checkTransition() reads only control_mode, which nav
// leaves at K_LOCOMOTION for the whole mission. So the FSM goes
// LOCOMOTION -> (trip) -> RECOVERY_STAND -> (next tick) -> LOCOMOTION -> (trip),
// giving the recovery state exactly ONE tick per cycle - never enough to stand -
// while the body sinks. Measured on the only two runs in ~400 that enter it, both
// on leg 2's y-position branch with the value changing every tick (a genuine
// sustained violation, correctly debounced, not a held sample):
//   run 8507 wkc_finals: 54 trips, entered at z=0.306 m, bled to 0.257 -> PASS
//   run 8892 wkc_weave:  45 trips, entered at z=0.219 m, bled to 0.183 -> FELL
// The bleed rate is ~0.36 m/s (8892: 0.2186 -> 0.1832 in 49 ticks / 98 ms), and
// RECOVERY_STAND::onEnter() re-decides fold-vs-stand EVERY entry against
// `0.2 < body_height < 0.45`. So the cycle drives the body across its own 0.20 m
// fold threshold and then RECOVERY_STAND FOLDS FOUR LEGS under a body still at
// cruise. Survival is decided by height headroom above 0.20 m when the cycle
// starts: 8507 had 106 mm and lived, 8892 had 19 mm and died.
// CTRL_LOCO_UNSAFE_HOLD_MS holds the robot IN RecoveryStand for that long after a
// trip so the stand-up actually runs. 0 = stock (bounce back next tick).
std::atomic<long> g_locoUnsafeHold{0};
std::atomic<long> g_locoEntrySeq{0};   // bumped on every LOCOMOTION onEnter (OPEN-40 option b)
std::atomic<long> g_locoUnsafeAdvisory{0};  // trips suppressed by OPEN-40 option (d)
std::atomic<long> g_locoTick{0};       // monotonic across LOCOMOTION re-entries, unlike `iter`,
                                       // which onEnter() resets and so restarts every cycle
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
  g_locoEntrySeq.fetch_add(1);
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
  g_locoTick.fetch_add(1);

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
    // OPEN-40 option (d), CTRL_LOCO_UNSAFE_ADVISORY_VMAX (default -1 = stock/off).
    // THIS DISABLES A SAFETY TRANSITION ABOVE A SPEED, and it is written because
    // the measurements say the transition is what kills:
    //   * the trip hands the FSM to RecoveryStand, which reads only control_mode
    //     and is handed straight back, so a sustained violation becomes a 500 Hz
    //     cycle - 296 trips and 296 entries in one measured run;
    //   * holding it there instead (option a) collapsed the cycle 60x exactly as
    //     designed AND tipped 4 of 4 runs completely over (roll 131-180 deg),
    //     because RecoveryStand's _StandUp interpolates from a captured pose and
    //     assumes a STATIONARY body;
    //   * suppressing the fold (option e) is a measured null - 57 folds
    //     suppressed, outcome unchanged.
    // So no variation on "reach RecoveryStand better" works, and what is left is
    // not routing a MOVING robot into a state built for a fallen one. Above the
    // knob the trip is advisory: it is counted and logged and locomotion
    // continues. Below it, stock behaviour is untouched - a genuinely fallen or
    // slow robot still recovers.
    // THE RISK, stated plainly: this removes the guard in the regime where it
    // fires. Every y-position line in the archive grazes the limit by 0-11 mm of
    // 240, which is the argument that there is nothing to guard against there;
    // that argument is not proof, the knob defaults OFF, and shipping it is the
    // operator's call (ISSUES decision #4).
    static const double advisory_vmax = ctrl_tuning::num("CTRL_LOCO_UNSAFE_ADVISORY_VMAX", -1.0);
    const T v_body = this->_data->_stateEstimator->getResult().vBody.template head<2>().norm();

    // OPEN-40 option (f), the CYCLE CAP - added 2026-09-17 and DEFAULT OFF.
    // ((e) is already the fold gate, CTRL_RECOVER_FOLD_VMAX.)
    // Measured at the SHIPPED trigger (241 runs, ctrl logs read individually):
    // the hazard is not the trip, it is the SUSTAINED cycle. Runs with <= 2
    // trip/recover cycles survived 6 of 6; runs with >= 31 cycles fell 7 of 11
    // while bleeding a median 155 mm of body height, and entry height did NOT
    // discriminate (FAIL median 0.295 m vs PASS 0.293 m). So keep the guard's
    // ACTION for a genuine one-off trip and only fall back to advisory once the
    // pattern has proved itself a cycle. That is strictly more conservative than
    // option (d), which never transitions at all above a speed.
    //
    // The gap test is what separates a cycle from two unrelated trips: inside the
    // limit cycle successive trips are 2-3 ticks apart (trip -> RECOVERY_STAND for
    // one tick -> back to LOCOMOTION -> trip), so any quiet gap longer than
    // CTRL_LOCO_UNSAFE_CYCLE_GAP_TICKS ends the run of cycles and resets the count.
    static const long cycle_cap = (long)ctrl_tuning::num("CTRL_LOCO_UNSAFE_CYCLE_CAP", 0.0);
    static const long cycle_gap = (long)ctrl_tuning::num("CTRL_LOCO_UNSAFE_CYCLE_GAP_TICKS", 250.0);
    static long cycle_count      = 0;
    static long last_unsafe_tick = -1000000;
    const long now_tick = g_locoTick.load();
    if(now_tick - last_unsafe_tick > cycle_gap) cycle_count = 0;
    last_unsafe_tick = now_tick;
    ++cycle_count;
    const bool cycle_capped = (cycle_cap > 0 && cycle_count > cycle_cap);
    if(cycle_capped) {
      static int cap_logged = 0;
      if(cap_logged < 20) {
        cap_logged++;
        shmtrace::logf(0.0, "[lococap] unsafe trip %ld in this run of cycles (> cap %ld) at %.2f m/s - ADVISORY from here, staying in LOCOMOTION",
                       cycle_count, cycle_cap, (double)v_body);
      }
    }

    if(cycle_capped || (advisory_vmax > 0.0 && (double)v_body > advisory_vmax)) {
      ++g_locoUnsafeAdvisory;
      static int adv_logged = 0;
      if(adv_logged < 20) {
        adv_logged++;
        shmtrace::logf(0.0, "[locoadv] unsafe trip at %.2f m/s (> %.2f) - ADVISORY, staying in LOCOMOTION (%ld so far)",
                       (double)v_body, advisory_vmax, g_locoUnsafeAdvisory.load());
      }
      // nextStateName is left as the current state: no transition, no cycle.
    } else {
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = 0.;
      rc_control.mode = RC_mode::RECOVERY_STAND;
      // Arm the dwell so RecoveryStand is not handed back one tick later (OPEN-40).
      static const long hold_ticks = (long)(ctrl_tuning::num("CTRL_LOCO_UNSAFE_HOLD_MS", 0.0) / 2.0);
      if(hold_ticks > 0) g_locoUnsafeHold.store(hold_ticks);
    }
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
    // CORRECTED at 23:10 the same evening, by its own first measurement. The
    // first cut SKIPPED unchanged ticks, and the very first probe run logged 50
    // held samples - the throttle cap - so holds are ROUTINE at matched rates
    // (the controller ticks at 500 Hz and the sensor stream delivers ~500/s, so
    // two control ticks landing on one packet is ordinary, not a fault). Skipping
    // those ticks would have made every genuine trip slower by however often the
    // stream happens to hold, an input nobody chose. The rule that needs no such
    // bargain: count EVERY over-limit tick as before, but require that at least
    // two DISTINCT values were seen while over the limit before tripping. A
    // frozen sample reports one value however long it lasts and can never trip;
    // a real transient reports a different value every tick and trips on
    // schedule. Checked against both runs on record: 8407's eight ticks at an
    // identical 9.749 m/s never trip, 8315's 10.064 -> 12.389 would trip on time.
    // CTRL_LEG_HELD_GATE=0 restores the plain tick count for an A/B.
    static const bool  held_gate = ctrl_tuning::flag("CTRL_LEG_HELD_GATE", true);
    static float       last_py[4] = {0, 0, 0, 0}, last_pz[4] = {0, 0, 0, 0}, last_v[4] = {0, 0, 0, 0};
    static bool        last_valid[4] = {false, false, false, false};
    static long        held_run[4] = {0, 0, 0, 0};
    const float py_now = (float)p_leg[1], pz_now = (float)p_leg[2];
    const float v_now  = (float)this->_data->_legController->datas[leg].v.norm();
    const bool  held   = last_valid[leg] && py_now == last_py[leg] && pz_now == last_pz[leg] && v_now == last_v[leg];
    last_py[leg] = py_now; last_pz[leg] = pz_now; last_v[leg] = v_now; last_valid[leg] = true;
    {   // the lateral excursion itself, whatever the limit is set to
      const long ymm = (long)(std::fabs((double)py_now) * 1000.0 + 0.5);
      if(ymm > g_legYMaxMm.load()) g_legYMaxMm.store(ymm);
    }
    if(held) {
      ++g_legHeldTicks;
      if(++held_run[leg] > g_legHeldMaxRun.load()) g_legHeldMaxRun.store(held_run[leg]);
    } else {
      held_run[leg] = 0;
    }
    // "has this over-limit streak shown more than one value?", per leg per check
    static float legy_first[4] = {0, 0, 0, 0}, hip_first[4] = {0, 0, 0, 0}, legv_first[4] = {0, 0, 0, 0};
    static bool  legy_moved[4] = {false, false, false, false}, hip_moved[4] = {false, false, false, false}, legv_moved[4] = {false, false, false, false};
    if(p_leg[2] > 0) {
      if(hip_over[leg]++ == 0) { hip_first[leg] = pz_now; hip_moved[leg] = false; }
      else if(pz_now != hip_first[leg]) hip_moved[leg] = true;
      if(hip_over[leg] >= hip_ticks && (hip_moved[leg] || !held_gate)) {
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
    // A KNOB SINCE 2026-09-16 01:45, default unchanged (ISSUES OPEN-39). This
    // limit is a STANCE-ENVELOPE policy, not a mechanical guard, and the number
    // is inherited: 0.18 * (0.08/0.062) = 0.232 rounded up to 0.24, mini-cheetah's
    // ratio, never checked against this robot's gait. The FK says
    // p(1) = (l1+l4)*cos(q0) + R*sin(q0), so with the Go1's l1 = 0.08, l4 = 0 and
    // R up to _maxLegLength = 0.430 the MECHANICAL ceiling at Unitree's own abad
    // stop (49.5 deg) is 0.379 m - 0.24 is 63 % of it. The shipped trot reaches
    // 0.240-0.247 m on wkc_finals and hp_gap20 (an abad angle of 44.9 deg at a
    // 0.27 m stance, well inside the stop), i.e. it EXCEEDS the policy by 0-7 mm,
    // which is why this branch fires at all: run 8507 tripped it 54 times and
    // survived because the body was standing, run 8315 tripped it at cruise and
    // fell. 0.30 would sit 21 % above the gait's envelope and 21 % below the
    // mechanical bound (reaching 0.30 needs 56 deg of abad at stance, past the
    // stop, so only an extended leg genuinely out of the envelope gets there).
    // The default here stays 0.24 - which value SHIPS is the operator's call;
    // this makes the A/B one env var instead of a rebuild.
#ifdef USE_GO1_MODEL
    static const T max_pleg_y = (T)ctrl_tuning::num("CTRL_MAX_PLEG_Y", 0.24);
#else
    static const T max_pleg_y = (T)ctrl_tuning::num("CTRL_MAX_PLEG_Y", 0.18);
#endif
    if(std::fabs(p_leg[1]) > max_pleg_y) {
      if(legy_over[leg]++ == 0) { legy_first[leg] = py_now; legy_moved[leg] = false; }
      else if(py_now != legy_first[leg]) legy_moved[leg] = true;
      if(legy_over[leg] >= legy_ticks && (legy_moved[leg] || !held_gate)) {
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
    // OPEN-40 option (b), CTRL_LEG_TRIP_RESET_ON_ENTRY (default OFF = stock).
    // These counters are function-statics, so they survive the FSM leaving and
    // re-entering LOCOMOTION. That is what makes the limit cycle tight: once a
    // counter is past its trip threshold it re-trips on the FIRST tick of every
    // re-entry, so the debounce protects only the very first trip and the cycle
    // runs at 500 Hz. Zeroing them on a fresh entry restores the 5-tick budget
    // per entry, which divides the cycle's frequency - and so its height bleed,
    // the measured discriminator between the 61 runs that fell and the 18 that
    // survived - by legy_ticks. It does NOT stop the cycle; it is the cheap,
    // partial half of the fix and is compatible with the dwell above.
    static const bool reset_on_entry = ctrl_tuning::flag("CTRL_LEG_TRIP_RESET_ON_ENTRY", false);
    static long       last_entry_seq = -1;
    if(reset_on_entry) {
      const long seq = g_locoEntrySeq.load();
      if(seq != last_entry_seq) {      // first leg of the first tick after an entry
        last_entry_seq = seq;
        for(int i = 0; i < 4; i++) { hip_over[i] = 0; legy_over[i] = 0; legv_over[i] = 0; }
      }
    }
    const auto v_leg = v_now;   // computed once above for the held-sample gate
    if(std::fabs(v_leg) > legv_mps) {
      if(legv_over[leg]++ == 0) { legv_first[leg] = v_now; legv_moved[leg] = false; }
      else if(v_now != legv_first[leg]) legv_moved[leg] = true;
      if(legv_over[leg] >= legv_ticks && (legv_moved[leg] || !held_gate)) {
        shmtrace::logf(0.0, "Unsafe locomotion: leg %d is moving too quickly (%.3f m/s, %d ticks over %.1f)", leg, (double)v_leg, legv_over[leg], (double)legv_mps);
        return false;
      }
      if(legv_spikes_logged < 50) {
        legv_spikes_logged++;
        shmtrace::logf(0.0, "[legv] leg %d over %.1f m/s for %d tick(s) (%.3f m/s, %s) - not a trip until %d", leg, (double)legv_mps, legv_over[leg], (double)v_leg, legv_moved[leg] ? "changing" : "one value - held sample", legv_ticks);
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
