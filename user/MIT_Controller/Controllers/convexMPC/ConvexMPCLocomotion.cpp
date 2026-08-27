#include <cstdlib>
#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <pthread.h>
#include <sched.h>
#include <cmath>
#include "ConvexMPCLocomotion.h"
#include <Controllers/SprawlGuard.h>
#include "../../../../stm32mp1/gazebo/ShmTrace.h"   // per-tick/text SHM tracing - see that file's own header

// Per-foot force cap handed to the convex MPC. Mini-cheetah's 120 N suits a
// 9 kg / 88 N robot; the Go1 is 13.1 kg / 128 N and its knee (35.55 Nm over a
// ~0.10-0.15 m stance moment arm) is good for 240-355 N at the foot. At 120 N
// the solver cannot plan the 2-3x bodyweight peaks bounding and galloping need.
#ifdef USE_GO1_MODEL
// MIT uses 120 N for the 9 kg / 88 N mini-cheetah - 1.36x bodyweight per foot.
// Holding that ratio for the 13.1 kg / 128 N Go1 gives 175 N, which is well
// inside what its knee can actually deliver (35.55 Nm over a 0.10-0.15 m stance
// moment arm is 240-355 N at the foot). 250 N was tried and made the stand sit
// lower, so this keeps MIT's tuning rather than inventing a new one.
#define MPC_F_MAX 175
#else
#define MPC_F_MAX 120
#endif
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"

#include "Gait.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH


////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
  iterationsBetweenMPC(_iterations_between_mpc),
  // Horizon is env-tunable ($CTRL_MPC_HORIZON) because dense-QP solve cost grows
  // steeply with it, and this board is 30x slower than the x86 UP board MIT
  // shipped on: the 10-step horizon measures 62-68 ms per solve on the A7
  // against a 2 ms control period.
  horizonLength(getenv("CTRL_MPC_HORIZON") ? atoi(getenv("CTRL_MPC_HORIZON")) : 10),
  dt(_dt),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
  //bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  jumping(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  //trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
  trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  shmtrace::logf(0.0, "[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f", dt, iterationsBetweenMPC, dtMPC);
  // $CTRL_F_MAX overrides the per-foot force cap. The 175 N default holds MIT's
  // 1.36x-bodyweight ratio, and 250 N was rejected earlier because it made the
  // STAND sit lower - but standing is not what the cap binds on. With two feet
  // down (bounding) a 2-3x bodyweight peak is 128-192 N per foot, and with one
  // (galloping) it is 250-350 N, so 175 N is at or past the limit for exactly
  // the gaits that fail. The knee is good for 240-355 N at the foot.
  {
    static const float fmax = getenv("CTRL_F_MAX") ? atof(getenv("CTRL_F_MAX"))
                                                  : (float)MPC_F_MAX;
    setup_problem(dtMPC, horizonLength, 0.4, fmax);
  }
  //setup_problem(dtMPC, horizonLength, 0.4, 650); // DH

  // Start the MPC worker (see the header for why the solve cannot run inline on
  // this board). CTRL_MPC_ASYNC=0 restores the stock inline behaviour for A/B.
  // DEFAULT IS INLINE (async OFF). Every measured result in this port - the
  // whole 100 m dash table, the yaw envelope, the star mission - was taken
  // inline, and the reduced solve fits inside the gait segment, so MIT's own
  // synchronous semantics hold. The async worker exists for a machine too slow
  // to solve in-segment; defaulting to it meant the shipped default was the
  // configuration NOTHING was measured against. $CTRL_MPC_ASYNC=1 re-enables it.
  _mpcAsync = (getenv("CTRL_MPC_ASYNC") && atoi(getenv("CTRL_MPC_ASYNC")) != 0);
  if (_mpcAsync) {
    _mpcThread = std::thread(&ConvexMPCLocomotion::_mpcWorker, this);
    shmtrace::logf(0.0, "[Convex MPC] solve runs ASYNC on a worker thread");
  } else {
    shmtrace::logf(0.0, "[Convex MPC] solve runs INLINE (stock)");
  }

  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  initSparseMPC();

   pBody_des.setZero();
   vBody_des.setZero();
   aBody_des.setZero();
}

ConvexMPCLocomotion::~ConvexMPCLocomotion() {
  _mpcQuit.store(true);
  { std::lock_guard<std::mutex> lk(_mpcMtx); _mpcRequest = true; }
  _mpcCv.notify_all();
  if (_mpcThread.joinable()) _mpcThread.join();
}

void ConvexMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
  _height_blend = 0.f;      // restart the entry height ramp (see _SetupCommand)
  _locoEntryMs = nowMs();
  // A solution from a previous LOCOMOTION episode is stale by definition
  // (different pose, different gait phase); drop it so the bootstrap and the
  // standing-entry window below re-engage cleanly on re-entry.
  _mpcHaveSolution.store(false);
}

/*!
 * Situational height bias from the PLANNER, metres, added to the height
 * governor's biological target. Positive = ride higher.
 *
 * The reactive half of body-height control lives in HeightGovernor and works
 * off the robot's own state. This is the predictive half's only channel into
 * it: the planner knows a corner is coming and can pre-load margin before the
 * lateral demand shows up, which is what an animal reading the ground ahead
 * does. Same free-function hook pattern as setEdamp()/setStandUpHeight().
 */
static std::atomic<double> g_heightBias{0.0};
void setHeightBias(double b) { g_heightBias.store(b); }

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
  if(data._quadruped->_robotType == RobotType::MINI_CHEETAH){
#ifdef USE_GO1_MODEL
    _body_height = 0.30;   // Go1 nominal stance (legs 0.426 m vs MC's 0.404)
#else
    _body_height = 0.29;
#endif
#ifdef USE_GO1_MODEL
    // Go1 SITL: allow a crouched gait ($CTRL_BODY_H, m) - lower CoM = much
    // larger roll/pitch margins through the UDP loop's extra latency.
    {
      static float h_env = -1.f;
      if (h_env < 0.f) {
        const char* e = getenv("CTRL_BODY_H");
        h_env = e ? atof(e) : 0.f;
      }
      if (h_env > 0.05f) _body_height = h_env;
    }
#endif
  }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
    _body_height = 0.45;
  }else{
    assert(false);
  }

  // ---- ENTRY HEIGHT RAMP ----
  // BALANCE_STAND settles the Go1 around 0.21-0.26 m, but locomotion's nominal
  // _body_height is 0.30. Handing the MPC that 9 cm step on the first tick of
  // LOCOMOTION makes it solve for a large upward force and LAUNCH the robot:
  // measured z 0.211 -> 0.342 in ~0.2 s at 0.32 m/s vertical, after which it
  // comes down badly and rolls out (wB peaked at -3.2 rad/s, roll -49 deg,
  // then the FSM bailed to RECOVERY_STAND for the rest of the run). MIT never
  // sees this because a mini-cheetah's balance stand already sits at its
  // locomotion height.
  // So start from the height the robot is ACTUALLY at and ramp to nominal.
  {
    float z_now = data._stateEstimator->getResult().position[2];
    if (_height_blend <= 0.f && z_now > 0.05f) _entry_height = z_now;
    if (_height_blend < 1.f) {
      _height_blend += dt / _height_ramp_s;
      if (_height_blend > 1.f) _height_blend = 1.f;
      _body_height = _entry_height + (_body_height - _entry_height) * _height_blend;
    }

    /*
     * REACTIVE STANCE HEIGHT. Everything above sets an OPEN-LOOP reference and
     * lets the robot settle wherever the force balance drops it - 0.21-0.24 m
     * when healthy, 0.19 when it is about to go down. Since achieved height is
     * the only quantity that separates this port's passes from its falls, close
     * the loop on it: trim the reference until the MEASURED height sits in the
     * band a cheetah runs at (0.55-0.57 x leg length, Zhang et al. 2022), and
     * if the trim saturates while the body is still sinking, buy vertical force
     * with forward speed. Full rationale and citations in HeightGovernor.h.
     *
     * Armed only once the entry ramp has finished - during the ramp the
     * reference is deliberately not where the robot should be.
     */
    _hgov.bias = (float)g_heightBias.load();
    _hgov.update(z_now, _body_height, dt, _height_blend >= 1.f);
    _body_height = _hgov.reference();
  }

  float x_vel_cmd, y_vel_cmd;
  // Command smoothing. MIT hard-codes 0.1, i.e. a ~20 ms time constant at
  // 500 Hz - quite fast. Unitree's trajPlanner (0xe6a70) instead carries three
  // complementary first-order pairs, 0.998/0.002, 0.994/0.006 and 0.992/0.008,
  // which are 250 ms - 1 s constants: 12-50x heavier smoothing of the reference
  // it hands downstream. Worth testing here because the gaits still failing
  // (pronking, galloping) die on the transient at gait engagement, not in
  // steady state. $CTRL_CMD_FILTER overrides; default stays MIT's 0.1.
  static const float kCmdFilter =
      getenv("CTRL_CMD_FILTER") ? atof(getenv("CTRL_CMD_FILTER")) : 0.1f;
  float filter(kCmdFilter);
  if(data.controlParameters->use_rc){
    const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;
    data.userParameters->cmpc_gait = rc_cmd->variable[0];
    _yaw_turn_rate = -rc_cmd->omega_des[2];
    x_vel_cmd = rc_cmd->v_des[0];
    y_vel_cmd = rc_cmd->v_des[1] * 0.5;
    _body_height += rc_cmd->height_variation * 0.08;
  }else{
    _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
    x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
    y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];
  }
  /*
   * The height governor's second lever. Applied to the COMMAND rather than to
   * the filtered state so the derate arrives through the same first-order lag
   * as any other command change - scaling _x_vel_des directly would compound
   * every tick and brake far harder than asked.
   */
  // Two independent reasons to give up forward speed, and they compose: the
  // height governor's departure derate, and the sprawl guard asking for less
  // momentum to catch while it is arresting a roll.
  x_vel_cmd *= _hgov.speedScale() * sprawlSpeedScaleRef().load();
  _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter;
  _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;

  // STM32MP1 SITL: prove the velocity command actually reaches the MPC.
  if (getenv("STM32MP1_EST_DBG")) {
    static int _vdbg = 0; ++_vdbg;
    if ((_vdbg % 25) == 0) {
      shmtrace::logf(0.0, "[MPC] gait=%d pad=%.3f stick=%.3f xcmd=%.3f -> xdes=%.3f  yawrate=%.3f  bodyH=%.3f",
             gaitNumber,
             data._desiredStateCommand->gamepadCommand
               ? data._desiredStateCommand->gamepadCommand->leftStickAnalog[1] : -9.f,
             data._desiredStateCommand->leftAnalogStick[1],
             x_vel_cmd, _x_vel_des, _yaw_turn_rate, _body_height);
    }
  }

  // HEADING REFERENCE - see the integration in run(). Upstream sets
  //   _yaw_des = rpy[2] + dt*_yaw_turn_rate
  // here, i.e. it re-slaves the yaw REFERENCE to the yaw MEASUREMENT every
  // tick, so the heading error handed to the MPC is only ever one timestep of
  // commanded turn (exactly zero when walking straight) and the robot has NO
  // heading regulation at all - it keeps whatever direction it drifts into.
  // Measured on the board: an 11.4 m straight-line trot ended 5 m off course.
  // Note MIT integrates world_position_desired properly (run(), below), so
  // position gets a real reference and heading does not; this makes the two
  // consistent. Kept switchable for A/B against stock.
  /*
   * BANK INTO THE TURN, and CROUCH while doing it - what every running animal
   * does and what this controller has never done.
   *
   * A dog, horse or cheetah entering a turn drops its centre of mass and leans
   * INTO the corner. That is not style: a turn needs lateral acceleration
   * a = v*omega, and a body held level must generate it as pure SHEAR at the
   * feet, which is what this port pays for in roll disturbance and what trips
   * the SafetyChecker. Leaning puts the ground reaction along the body axis
   * instead, exactly as a banked track does for a car - the bank angle that
   * makes the force axial is
   *       theta = atan(a_lat / g)
   * so at the 2.5 m/s^2 budget the correct lean is 14.3 deg. Crouching lowers
   * the moment arm the same disturbance acts through.
   *
   * Both are computed from the controller's OWN commands - it already knows v
   * and omega - so no plumbing from the planner is needed.
   * $CTRL_BANK scales the lean (0 = off, 1 = full geometric bank).
   * $CTRL_CORNER_CROUCH is metres of crouch at full lateral load.
   */
  {
    static const float bank_k = getenv("CTRL_BANK") ? atof(getenv("CTRL_BANK")) : 0.f;
    static const float crouch = getenv("CTRL_CORNER_CROUCH")
                                ? atof(getenv("CTRL_CORNER_CROUCH")) : 0.f;
    const float a_lat = _x_vel_des * _yaw_turn_rate;      // signed, body frame
    if (bank_k > 1e-6f) {
      // Lean INTO the turn: a left turn (+omega, CCW) banks to the left, which
      // is NEGATIVE roll in this frame.
      _roll_des = -bank_k * std::atan2(a_lat, 9.81f);
      const float lim = 0.35f;                             // ~20 deg ceiling
      if (_roll_des >  lim) _roll_des =  lim;
      if (_roll_des < -lim) _roll_des = -lim;
    } else {
      _roll_des = 0.;
    }
    if (crouch > 1e-6f) {
      // Drop the CoM in proportion to how hard the turn is, capped so the
      // stance never gets so low the gait cannot sustain itself.
      const float f = std::min(1.f, std::fabs(a_lat) / 2.5f);
      _body_height -= crouch * f;
      if (_body_height < 0.20f) _body_height = 0.20f;
    }
  }
  /*
   * Fore/hind differential. The cheetah does not run level: hindlegs sit at
   * 0.57 x leg length and forelegs at 0.55, i.e. nose-down by ~1.3 deg over
   * the Go1's hip separation (Zhang et al. 2022 section 4.3, and section 5.3 -
   * "designing different control methods for the fore and hindlegs ... are
   * essential", forelegs carrying weight and cushioning while hindlegs drive).
   * Off unless $CTRL_HGOV_PITCH is set: it is a small effect and gets its own
   * A/B rather than a free ride on the height loop's result.
   */
  _pitch_des = _hgov.pitchBias();

}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data) {
  bool omniMode = false;

  // Command Setup
  _SetupCommand(data);
  // $SIM_GAIT overrides the yaml's initial cmpc_gait so a gait matrix can be
  // swept without editing (and re-deploying) a config file per run. Applied
  // ONCE, into cmpc_gait itself, on firstRun - NOT re-applied every tick.
  // The original form of this override (`if (gait_env >= 0) gaitNumber =
  // gait_env;`, unconditional, every tick) silently discarded every runtime
  // cmpc_gait.set() write - exactly the mechanism the mission analyzer's
  // mid-course gait changes use (WP_ANALYZER=1): the analyzer's own
  // "[mission] gait X -> Y" print fired correctly (the write happened), but
  // the change never reached the robot because this override put gaitNumber
  // right back every single tick afterward, forever. Note firstRun is reset
  // by initialize() on every LOCOMOTION entry (not just mission start), so a
  // dash-finish interlude that re-enters LOCOMOTION after a mid-course
  // analyzer switch will re-seed cmpc_gait back to SIM_GAIT's value - treated
  // as correct here (the sprint should run the recipe's base gait), not as a
  // bug to chase.
  {
    static const int gait_env = getenv("SIM_GAIT") ? atoi(getenv("SIM_GAIT")) : -1;
    if (gait_env >= 0 && firstRun) data.userParameters->cmpc_gait = gait_env;
  }
  gaitNumber = data.userParameters->cmpc_gait;
  // 20+ are this port's additions (walking / walking2 / galloping); they must
  // bypass MIT's omni rewrite, which would otherwise turn 20/21/22 into
  // 10/11/12 and then into 0/1/2.
  if(gaitNumber >= 20) {
    // keep as-is
  } else if(gaitNumber >= 10) {
    gaitNumber -= 10;
    omniMode = true;
  }

  // ---- ENTER THROUGH MIT'S OWN STANDING GAIT ----
  // The sequencer switches control_mode straight into a dynamic gait, so a
  // diagonal pair lifts on the very first gait tick - while the only support
  // is the constant-force bootstrap, which has no attitude feedback. On MIT's
  // hardware the first solve is synchronous, so model-based forces exist from
  // tick one; here the first solve lands 30-240 ms later, and the robot is
  // already tipping about the support diagonal when it does (measured: falls
  // 0.1-0.3 s after entry, repeatably). MIT's own flow drives LOCOMOTION at
  // gait 4 (standing) first and lets the operator pick a dynamic gait, so do
  // the same: hold `standing` until the async pipeline has produced its first
  // solution AND a settle window has passed ($CTRL_GAIT_WAIT_MS, default 600).
  if (_mpcAsync) {
    static const int64_t waitMs = getenv("CTRL_GAIT_WAIT_MS")
                                ? atoll(getenv("CTRL_GAIT_WAIT_MS")) : 600;
    bool hold = (!_mpcHaveSolution.load() || (nowMs() - _locoEntryMs) < waitMs);
    if (getenv("STM32MP1_MPC_IN")) {
      static int _wc = 0;
      if ((++_wc % 500) == 1)
        shmtrace::logf(0.0, "[WIN] hold=%d gait=%d have=%d age_ms=%lld wait=%lld",
               (int)hold, gaitNumber, (int)_mpcHaveSolution.load(),
               (long long)(nowMs() - _locoEntryMs), (long long)waitMs);
    }
    if (hold)
      gaitNumber = 4;
  }

  // Zero-velocity hold. NOTE the window above only applies when _mpcAsync is
  // set, and this port runs the solve INLINE by default - so until now nothing
  // stopped a dynamic gait from engaging while the sequencer still held the
  // velocity command at zero, which is exactly where every flight-phase gait
  // dies. Applies in both modes.
  if (zeroVelHold())
    gaitNumber = 4;      // MIT's standing gait

  auto& seResult = data._stateEstimator->getResult();

  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &pronking;
  else if(gaitNumber == 3)
    gait = &random;
  else if(gaitNumber == 4)
    gait = &standing;
  else if(gaitNumber == 5)
    gait = &trotRunning;
  else if(gaitNumber == 6)
    gait = &random2;
  else if(gaitNumber == 7)
    gait = &random2;
  else if(gaitNumber == 8)
    gait = &pacing;
  // MIT defines `walking` and `walking2` but the stock selector stops at 8, so
  // neither is reachable and anything else silently falls through to `trotting`.
  // Both matter here:
  // NUMBERING: MIT reserves >=10 for "same gait, omniMode" - run() does
  //   if(gaitNumber >= 10) { gaitNumber -= 10; omniMode = true; }
  // so 10/11/12 are rewritten to 0/1/2 and silently select trotting/bounding/
  // pronking. The new gaits therefore live at 20+, which survives that
  // subtraction as 10/11/12 only if omni is wanted - so they are matched
  // BEFORE the subtraction instead (see run()).
  //   20 = walking  - 4-beat, offsets (0,3,5,8), one foot down at a time
  //   21 = walking2 - diagonal pairs like a trot but 7/10 duty, i.e. a 40%
  //                   DOUBLE-SUPPORT overlap. A 50%-duty trot is on exactly two
  //                   feet at every instant and free to roll about that
  //                   diagonal the whole time; the overlap is what removes that
  //                   window, and it is the single thing that made this port's
  //                   own hand-rolled trot stable.
  else if(gaitNumber == 20)
    gait = &walking;
  else if(gaitNumber == 21)
    gait = &walking2;
  //   22 = galloping - offsets (0,2,7,9), the only gait here with a real
  //        flight phase, and the one the published 2.5-4.7 m/s figures rely on.
  else if(gaitNumber == 22)
    gait = &galloping;
  current_gait = gaitNumber;

  // Look up the segment/clearance this gait wants at this speed and apply it
  // where it cannot cause a discontinuity. Done BEFORE setIterations so a
  // boundary change takes effect on the cycle that is just starting.
  applySchedule(gaitNumber, _x_vel_des, gait);

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);


  jumping.setIterations(27/2, iterationCounter);

  //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
  // check jump trigger
  jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
      data._desiredStateCommand->trigger_pressed);


  // bool too_high = seResult.position[2] > 0.29;
  // check jump action
  if(jump_state.should_jump(jumping.getCurrentGaitPhase())) {
    gait = &jumping;
    recompute_timing(27/2);
    _body_height = _body_height_jumping;
    currently_jumping = true;

  } else {
    recompute_timing(default_iterations_between_mpc);
    currently_jumping = false;
  }

  if(_body_height < 0.02) {
#ifdef USE_GO1_MODEL
    _body_height = 0.30;   // Go1 nominal stance (legs 0.426 m vs MC's 0.404)
#else
    _body_height = 0.29;
#endif
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = 
    omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");

  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
  }

  // HEADING HOLD IS UNCONDITIONAL. It was an env toggle only because it started
  // as an A/B experiment, and an experiment scaffold has no business shipping:
  // there is no situation - sim or hardware - in which a legged robot is better
  // off with NO heading regulation. Upstream MIT has none (it re-slaves the yaw
  // reference to the measurement every tick, so the error it sees is always
  // zero); without this, walking travels 9.3 m before the yaw runs away, and
  // with it, 93.6 m. Turning it off is not a configuration, it is a defect.
  static const bool heading_hold = true;

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
    if (heading_hold) {
      _yaw_des += dt * _yaw_turn_rate;   // a REFERENCE, not the measurement
      // [YAWREF] ($SIM_YAWREF=1): is the heading REFERENCE moving the same way
      // the robot actually is? Measured, a positive stick command turns the
      // robot CLOCKWISE while this line winds _yaw_des COUNTER-clockwise, so
      // reference and reality would diverge without bound - except the error
      // clamp below hides it at 0.40 rad. Print both to settle it.
      {
        static const bool yref = getenv("SIM_YAWREF") && atoi(getenv("SIM_YAWREF")) != 0;
        if (yref) {
          static int nyr = 0;
          if ((nyr++ % 250) == 0)
            shmtrace::logf(0.0, "[YAWREF] wzCmd=%+.3f yawDes=%+.4f yawAct=%+.4f err=%+.4f",
                   _yaw_turn_rate, _yaw_des, seResult.rpy[2],
                   _yaw_des - seResult.rpy[2]);
        }
      }
      // Keep the reference on the same branch as the measurement (rpy wraps at
      // +-pi; an unwrapped reference would hand the MPC a 2*pi error and spin
      // the robot), and saturate how much correction it may ask for so a big
      // disturbance cannot wind up into a violent turn.
      float e = _yaw_des - seResult.rpy[2];
      while (e >  (float)M_PI) { _yaw_des -= 2.f*(float)M_PI; e -= 2.f*(float)M_PI; }
      while (e < -(float)M_PI) { _yaw_des += 2.f*(float)M_PI; e += 2.f*(float)M_PI; }
      // How far the heading reference may lead the measurement. This is also
      // the yaw error the MPC sees during a sustained turn, and it COMPETES
      // WITH HEIGHT: with Kp_stance = 0 every newton of support comes from the
      // MPC's force solution, so a permanently-saturated yaw error can be paid
      // for out of body height. Measured: the walk cleared a star corner and
      // then collapsed level at z=0.091 with no orientation trip.
      static const float YAW_ERR_MAX =
          getenv("CTRL_YAW_ERR_MAX") ? atof(getenv("CTRL_YAW_ERR_MAX")) : 0.40f;
      if (e >  YAW_ERR_MAX) _yaw_des = seResult.rpy[2] + YAW_ERR_MAX;
      if (e < -YAW_ERR_MAX) _yaw_des = seResult.rpy[2] - YAW_ERR_MAX;

      // PROPORTIONAL HEADING FEEDBACK ON THE YAW-RATE CHANNEL.
      // The angle reference above is tracked by the MPC at Q[2]=10, but the yaw
      // RATE reference (vBody_Ori_des[2]) is just the commanded turn rate - zero
      // when walking straight - so nothing actively drives yaw BACK. Measured:
      // the 4-beat `walking` gait lifts one leg at a time, each step kicks yaw,
      // and with only a saturating angle error to fight it the heading ran away
      // to 0.51 rad (past the 0.40 clamp) and the robot went down at ~17 m every
      // run. `walking2` (diagonal pairs) is yaw-balanced and never showed this.
      // Feeding a corrective rate -kp*e into the rate channel (tracked at
      // Q[8]=0.3) adds authority WITHOUT needing a big angle error, so the drift
      // is unwound as it forms. Active only while holding heading (no turn
      // commanded); disabled by CTRL_YAW_RATE_KP=0.
      static const float yaw_rate_kp =
          getenv("CTRL_YAW_RATE_KP") ? atof(getenv("CTRL_YAW_RATE_KP")) : 1.5f;
      if (std::fabs(_yaw_turn_rate) < 0.05f) {
        _yaw_rate_ff = -yaw_rate_kp * e;
        const float ff_max = 0.8f;
        if (_yaw_rate_ff >  ff_max) _yaw_rate_ff =  ff_max;
        if (_yaw_rate_ff < -ff_max) _yaw_rate_ff = -ff_max;
      } else {
        _yaw_rate_ff = 0.f;
      }
    } else {
      _yaw_des = seResult.rpy[2] + dt * _yaw_turn_rate;   // stock MIT
      _yaw_rate_ff = 0.f;
    }
  } else {
    // Standing: track the measurement so entering locomotion starts from the
    // heading the robot is actually at (mirrors the stand_traj reset below).
    _yaw_des = seResult.rpy[2] + dt * _yaw_turn_rate;
  }

  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];
    _yaw_des = seResult.rpy[2];      // lock the heading reference on entry
    _yaw_rate_ff = 0.f;

    for(int i = 0; i < 4; i++)
    {

#ifdef USE_GO1_MODEL
      footSwingTrajectories[i].setHeight(0.055);
#else
      footSwingTrajectories[i].setHeight(0.05);
#endif
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    // Make the FIRST control tick land on an MPC solve boundary.
    // iterationCounter++ runs before updateMPCIfNeeded, so with a fresh
    // counter the first solve would otherwise wait iterationsBetweenMPC
    // ticks (~26 ms) - during which f_ff/Fr_des are ZERO and the stance
    // legs have only the soft joint PD: the body free-falls at gait entry,
    // then the catch-up force spike tumbles the robot.
    iterationCounter = iterationsBetweenMPC - 1;
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  //float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {
    //footSwingTrajectories[i].setHeight(.05);
#ifdef USE_GO1_MODEL
    // Swing clearance, env-tunable: the 2.5 m walk died on a foot scuff (z dip
    // mid-swing), and clearance is the direct lever on scuffs.
    // DEFAULT 0.11, not 0.07: every 100 m dash result was measured at 0.11, and
    // clearance is the direct lever on the foot scuffs that killed the early
    // walks. Raising it further is HARMFUL at speed (at 3.5 m/s: 0.11 -> 27.7 m,
    // 0.14 -> 22.1 m, 0.17 -> 13.8 m) because a higher swing must be covered in
    // the same shrinking swing window.
    // LATCHED PER LEG AT SWING START. The trajectory is a Bezier evaluated over
    // the whole swing, so changing its height mid-flight moves the target out
    // from under a foot that is already travelling. Upstream MIT calls
    // setHeight() every tick (its own `if(firstSwing[i])` guard is commented
    // out), which is harmless with a constant but not with a scheduler.
    if (firstSwing[i]) _swingHLatched[i] = scheduleFor(gaitNumber, _x_vel_des).swingH;
    footSwingTrajectories[i].setHeight(_swingHLatched[i]);
#else
    footSwingTrajectories[i].setHeight(.06);
#endif
#ifdef USE_GO1_MODEL
    // Place swing feet at the robot's own lateral hip offset (abad link). The
    // stock .065 is mini-cheetah's abad link; Go1's is .08, so .065 narrows the
    // stance 1.5 cm every step and erodes roll stability.
    Vec3<float> offset(0, side_sign[i] * data._quadruped->_abadLinkLength, 0);
#else
    Vec3<float> offset(0, side_sign[i] * .065, 0);
#endif

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected = 
      coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;


    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
          + des_vel * swingTimeRemaining[i]);

    //+ seResult.vWorld * swingTimeRemaining[i];

    //float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);

    // NOTE the missing `* dtMPC`. Upstream reads
    //     vWorld[1] * .5 * stance_time * dtMPC
    // while the x term one line up is
    //     vWorld[0] * (.5 + bonus) * stance_time
    // - no dtMPC. Both are the same capture-point quantity (v * T_stance / 2),
    // so the extra factor makes LATERAL foot placement ~22x too weak at
    // dtMPC = 0.045 s. Sideways velocity then goes essentially uncorrected:
    // measured vB_y climbing 0.013 -> 0.466 m/s over ~1 s at gait entry while
    // roll ran away to -0.75 rad and the robot fell over every single time.
    // A mini-cheetah tolerates it; the Go1 does not.
    float pfy_rel = seResult.vWorld[1] * .5 * stance_time +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    Pf[2] = -0.003;
    //Pf[2] = 0.0;
    footSwingTrajectories[i].setFinalPosition(Pf);

  }

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
     0, 700, 0,
     0, 0, 150;
  // MIT runs stance with ZERO Cartesian stiffness - the MPC is supposed to do
  // all of it. That works when the MPC solves at 30-40 Hz; here it manages
  // ~11 Hz, and with force == bodyweight the body sits in neutral equilibrium
  // at whatever height it happens to be (measured: parked at 0.204 m against a
  // 0.30 m reference and never rising). A small stance stiffness gives the
  // height something to servo against between MPC updates without taking the
  // force authority away from the MPC. $CTRL_KP_STANCE, 0 restores stock.
  {
    static const float kps = getenv("CTRL_KP_STANCE") ? atof(getenv("CTRL_KP_STANCE")) : 0.f;
    Kp_stance = kps * Kp;
  }


  Kd << 7, 0, 0,
     0, 7, 0,
     0, 0, 7;
  Kd_stance = Kd;
  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();

  // ---- PIPELINED MPC (replaces the kSeg latency indexing) ----
  // MIT's design assumption is a solve that completes within one MPC segment,
  // synchronous with the gait clock. This board cannot do that inline, and the
  // previous scheme (solve whenever idle, index the solution trajectory by
  // elapsed time) recreated MIT's timing only approximately - and every edge
  // case of it (stale solutions across contact flips, busy-flag leaks, engage
  // bootstraps) has cost hours. So do what a slow synchronous MPC would do,
  // explicitly, as a two-stage pipeline:
  //   at segment boundary k:  APPLY the solution computed during segment k-1
  //                           (solved FOR this segment's contact table), and
  //                           DISPATCH a solve for segment k+1's table.
  // Latency is one segment, constant, by construction - the same zero-order
  // hold MIT already has between solves - and a gait-table change (standing ->
  // trot at engage) has its forces ready BEFORE its first tick executes.
  if (_mpcAsync && (iterationCounter % iterationsBetweenMPC) == 0) {
    // stage 1: promote last segment's solve result to the applied slot
    std::lock_guard<std::mutex> lk(_mpcMtx);
    if (_mpcHaveSolution.load()) {
      for (int leg = 0; leg < 4; ++leg) _fApplied[leg] = _frTraj[0][leg];
      _fAppliedValid = true;
    }
    // stage 2 happens in updateMPCIfNeeded below (dispatch with mpcTableNext)
  }
  // prefetch: the contact table one segment ahead, for the solve we dispatch now
  {
    gait->setIterations(iterationsBetweenMPC, iterationCounter + iterationsBetweenMPC);
    int* nxt = gait->getMpcTable();
    for (int i = 0; i < 4 * horizonLength && i < (int)(sizeof(_mpcTableNext)/sizeof(int)); ++i)
      _mpcTableNext[i] = nxt[i];
    gait->setIterations(iterationsBetweenMPC, iterationCounter);   // restore
  }
  updateMPCIfNeeded(_mpcAsync ? _mpcTableNext : mpcTable, data, omniMode);

  // ---- latency-compensated application of the async MPC solution ----
  // Index the force trajectory by how long ago its snapshot was taken, in MPC
  // segments, and rotate with the CURRENT attitude. A solve that took 1-2
  // segments then lands on the segment it was computed FOR instead of being
  // applied late - stale step-0 forces across a contact switch were the
  // sharpest remaining destabiliser once the solver itself was correct.
  if (_mpcAsync && _fAppliedValid) {
    std::lock_guard<std::mutex> lk(_mpcMtx);
    for (int leg = 0; leg < 4; ++leg) {
      f_ff[leg]   = -seResult.rBody * _fApplied[leg];   // current attitude
      Fr_des[leg] = _fApplied[leg];
    }
  }

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0,0,0,0);

#ifdef DRAW_DEBUG_PATH
  auto* trajectoryDebug = data.visualizationData->addPath();
  if(trajectoryDebug) {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for(int i = 0; i < 10; i++) {
      trajectoryDebug->position[i][0] = trajAll[12*i + 3];
      trajectoryDebug->position[i][1] = trajAll[12*i + 4];
      trajectoryDebug->position[i][2] = trajAll[12*i + 5];
      auto* ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

#ifdef DRAW_DEBUG_SWINGS
      auto* debugPath = data.visualizationData->addPath();
      if(debugPath) {
        debugPath->num_points = 100;
        debugPath->color = {0.2,1,0.2,0.5};
        float step = (1.f - swingState) / 100.f;
        for(int i = 0; i < 100; i++) {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
          debugPath->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }
      auto* finalSphere = data.visualizationData->addSphere();
      if(finalSphere) {
        finalSphere->position = footSwingTrajectories[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      auto* actualSphere = data.visualizationData->addSphere();
      auto* goalSphere = data.visualizationData->addSphere();
      goalSphere->position = footSwingTrajectories[foot].getPosition();
      actualSphere->position = pFoot[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);


      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      
      if(!data.userParameters->use_wbc){
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

#ifdef DRAW_DEBUG_SWINGS
      auto* actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pFoot[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }else{ // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0.*Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  // [CONTACT] SCHEDULE vs REALITY ($SIM_CONTACT_DBG=1).
  //
  // The MPC solves against a CONTACT SCHEDULE. Force it commands for a foot the
  // schedule calls "stance" goes into the ground only if that foot is actually
  // ON the ground. If the swing leg cannot complete its trajectory in the time
  // the gait allows - which gets harder as speed rises and the swing window
  // shrinks - the schedule runs ahead of reality and the solved force is
  // commanded into air. Commanded force is not delivered force.
  //
  // This is the remaining candidate for the 3.5 m/s wall, where force (2.5x mg),
  // orientation gains, joint torque (74% of limit), leg reach (0.393 of 0.430)
  // and swing clearance have all been eliminated by measurement. Supporting
  // hint: raising swing height makes 3.5 monotonically WORSE (27.7 -> 22.1 ->
  // 13.8 m), which is what a time-starved swing would do.
  {
    static const bool cdbg = getenv("SIM_CONTACT_DBG") && atoi(getenv("SIM_CONTACT_DBG")) != 0;
    if (cdbg) {
      static int nc = 0;
      static long schedStance = 0, airborneWhileStance = 0;
      static float worstAir = 0.f;
      for (int leg = 0; leg < 4; ++leg) {
        if (se_contactState[leg] > 0.f) {           // schedule says this foot is down
          ++schedStance;
          const float h = pFoot[leg][2];            // world height of the foot
          if (h > 0.04f) { ++airborneWhileStance; if (h > worstAir) worstAir = h; }
        }
      }
      if ((nc++ % 50) == 0 && schedStance > 0) {
        shmtrace::logf(0.0, "[CONTACT] t=%.2f schedStance=%ld airborneWhileStance=%ld (%.1f%%) "
               "worstAirGap=%.3f m",
               nc * 0.002f, schedStance, airborneWhileStance,
               100.0 * (double)airborneWhileStance / (double)schedStance, worstAir);
      }
    }
  }

  // [MPCZ] VERTICAL FORCE BUDGET ($SIM_MPCZ=1).
  // At 2.0 m/s commanded the robot does not tip - it SINKS (measured: height
  // 0.222 -> 0.139 while body vz oscillation grows to 0.55 m/s, then
  // "[FALL] collapsed roll=0 pitch=-0 z=0.028"). That is a vertical force
  // budget failure, so measure the budget directly rather than inferring it.
  //
  // To HOLD height a gait with stance duty d must average m*g over the cycle,
  // i.e. command m*g/d while feet are actually down. Commanding exactly m*g
  // during stance yields m*g*d on average and the body falls at (1-d)*g. This
  // prints what the solver actually asked for against that requirement, so
  // "the MPC under-commands" and "the legs fail to deliver" stop being the
  // same observation.
  {
    static const bool mpcz = getenv("SIM_MPCZ") && atoi(getenv("SIM_MPCZ")) != 0;
    if (mpcz) {
      static int nz = 0;
      if ((nz++ % 50) == 0) {
        float fzTot = 0.f; int nStance = 0;
        for (int foot = 0; foot < 4; ++foot) {
          if (Fr_des[foot][2] > 1.0f) { fzTot += Fr_des[foot][2]; ++nStance; }
        }
        const float mg = 12.859f * 9.81f;             // corrected Go1 total mass
        shmtrace::logf(0.0, "[MPCZ] t=%.2f z=%.3f zref=%.3f vz=%+.3f nSt=%d Fz=%.1f mg=%.1f "
               "Fz/mg=%.2f need=%.2f",
               nz * 0.002f, seResult.position[2], _body_height, seResult.vWorld[2],
               nStance, fzTot, mg, fzTot / mg,
               nStance > 0 ? 4.0f / (float)nStance : 0.f);
      }
    }
  }

  // CONTACT DETECTION (opt-in, $SIM_CONTACT_DETECT=1).
  //
  // MIT's ContactEstimator is a PASS-THROUGH - its own header says so: "it just
  // has a pass-through algorithm which passes the phase estimation to the state
  // estimator. This will need to change once we move contact detection to C++".
  // So the KF believes a foot is down because the GAIT SCHEDULE says it should
  // be, never because anything measured it. And the KF acts hard on that belief:
  // during scheduled swing it inflates that foot's measurement noise by up to
  // 100x (`high_suspect_number`), i.e. it discards the odometry entirely.
  //
  // When schedule and reality disagree, both failure directions are bad: a foot
  // that is actually loaded gets its good odometry thrown away, and a foot that
  // is actually airborne gets garbage fused. Measured here: pronking's body
  // height falls monotonically (0.290 -> 0.140) so the robot NEVER leaves the
  // ground, while its schedule calls 60% of the cycle flight.
  //
  // Detect it instead, from kinematics + IMU:
  //   * a foot's height below the body is known from FK (`datas[i].p`) rotated
  //     into the world frame - the LOWEST foot is the contact candidate, which
  //     is a RELATIVE test and so does not depend on the body-height estimate it
  //     would otherwise be circular with;
  //   * in genuine free flight the accelerometer reads ~0 rather than ~1g, so a
  //     low specific-force magnitude vetoes contact on every foot at once.
  // Applied to the ESTIMATOR only. The MPC's contact table stays scheduled -
  // that is a PLAN for the future, and rewriting it was already measured to make
  // things worse.
  {
    static const bool detect = getenv("SIM_CONTACT_DETECT") &&
                               atoi(getenv("SIM_CONTACT_DETECT")) != 0;
    if (detect) {
      const auto& se = data._stateEstimator->getResult();
      // Foot heights in the world frame, relative to the body.
      float footZ[4];
      float lowest = 1e9f;
      for (int i = 0; i < 4; i++) {
        Vec3<float> pw = se.rBody.transpose() *
            (data._quadruped->getHipLocation(i) + data._legController->datas[i].p);
        footZ[i] = pw[2];
        if (footZ[i] < lowest) lowest = footZ[i];
      }
      // Free-fall veto: specific force well under gravity means nothing is
      // pushing on the robot, so no foot can be bearing load.
      float aMag = 0.f;
      for (int k = 0; k < 3; k++) aMag += se.aBody[k] * se.aBody[k];
      aMag = std::sqrt(aMag);
      static const float ff_thresh = getenv("SIM_FREEFALL_G")
                                   ? atof(getenv("SIM_FREEFALL_G")) : 3.0f;
      const bool freeFall = (aMag < ff_thresh);

      static const float band = getenv("SIM_CONTACT_BAND")
                              ? atof(getenv("SIM_CONTACT_BAND")) : 0.02f;
      Vec4<float> detected;
      for (int i = 0; i < 4; i++) {
        const bool down = (!freeFall) && (footZ[i] < lowest + band);
        // Blend with the schedule rather than replacing it outright: the
        // schedule carries phase information (how far through stance) that a
        // binary detector does not, and the KF's trust ramp wants a phase.
        detected[i] = down ? std::max(se_contactState[i], 0.5f)
                           : std::min(se_contactState[i], 0.5f);
      }
      se_contactState = detected;
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.; 
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate + _yaw_rate_ff;   // + heading feedback (see _SetupCommand)

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update


}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<double>& data) {
  (void)data;
  shmtrace::logf(0.0, "call to old CMPC with double!");

}

/*!
 * True while the operator command has been effectively zero long enough that a
 * dynamic gait should not be running.
 *
 * Ported from Unitree's ConvexMPCLocomotion::zeroVelTransitionAmend (0xe41c0),
 * which upstream MIT does not have. Note what that function actually DOES: it
 * classifies the command against a 0.01 m/s threshold, runs a debounced counter
 * so one tick of zero cannot trigger anything, and then raises a TRANSITION
 * REQUEST (data+0x36a). It does not rewrite the contact schedule.
 *
 * That distinction was established by measurement here, not by reading. Patching
 * the MPC table directly - forcing every airborne horizon step back to stance -
 * was tried first and does nothing: pronking still reached 0.00 m and collapsed
 * at 10 s with the amendment confirmed firing ("6/10 horizon steps were airborne
 * ... forced to stance"). It cannot work, because the table only tells the MPC
 * what to solve against; the gait's own contact/swing states still swing the
 * legs, so the MPC ends up commanding forces into feet that are in the air.
 *
 * The mechanism that does matter: at zero commanded velocity a flight-phase gait
 * is a jump with nothing asked of it. All four feet leave the ground, the MPC has
 * no contact to push against, and with Kp_stance = 0 (MIT's design, which Unitree
 * keeps - their stance gain vector at .rodata 0x2fe0f8 is zeros) nothing else
 * holds the body up. Measured signature: dead level, sinking to ~0.10 m, no
 * orientation trip.
 *
 * So hold MIT's standing gait until there is actually a velocity to deliver.
 * $CTRL_ZEROVEL_HOLD_GAIT=0 restores stock behaviour; $CTRL_ZEROVEL_HOLD sets the
 * debounce in control ticks.
 */
/**
 * GAIT/SPEED PARAMETER SCHEDULE - the lookup half.
 *
 * Every entry below is MEASURED on the 100 m dash (real estimator, qpOASES).
 * Where a cell was never tested it inherits the nearest measured one and says
 * so, because an interpolated guess that looks like data is worse than a gap.
 *
 * Gait segment (ms) is the sharp parameter: stance duration x commanded speed
 * is the distance a stance foot must sweep, against ~0.318 m of horizontal
 * reach, so a faster gait needs a SHORTER cycle - but too short fails too, so
 * the optimum is interior and must be looked up rather than extrapolated.
 *
 *   trotting (9)    22 ms: crosses at 2.0 (47.2 s), 3.0 (33.2 s), 3.1 (32.2 s)
 *                   26 ms: crosses at 2.0/2.5/2.75, FAILS at 3.0 (67 m)
 *                   -> 26 below 2.9 (measured best times there), 22 at/above
 *   trotRunning (5) 26 ms: crosses 2.0 through 4.0 (24.8 s at 4.0)
 *                   22 ms: crosses to 3.75 but FAILS at 4.0 (32 m)
 *                   -> 26 everywhere
 *   walking (20)    22 ms: 46.6 s at 2.0 (26 ms: 48.3 s) -> 22
 *
 * Swing clearance is flat at 0.11 for every gait measured: 0.07 scuffs, and
 * raising it is HARMFUL at speed (3.5 m/s: 0.11 -> 27.7 m, 0.14 -> 22.1 m,
 * 0.17 -> 13.8 m) because a taller swing must be covered in the same window.
 */
ConvexMPCLocomotion::SchedParams
ConvexMPCLocomotion::scheduleFor(int gaitNumber, float speedCmd) {
  SchedParams p{22, 0.11f, 2.0f};
  const float v = std::fabs(speedCmd);
  switch (gaitNumber) {
    case 5:                       // trotRunning - 40% duty, flight phase
      p.segMs = 26;  p.vMax = 4.0f;  break;
    case 9:                       // trotting - 50% duty
      // 22 ms at EVERY speed, deliberately NOT speed-scheduled.
      // A "26 below 2.9, 22 above" rule looked better on paper - 26 gave
      // marginally quicker times at 2.5-2.75 - but those were FIXED-segment
      // runs. Scheduling it introduced a segment change at v~2.93, i.e. in the
      // middle of hard acceleration, and it cost the whole cell: trot at 3.0
      // went from 33.2/33.2/33.3 s (3/3) to falling at 21 m. 22 ms is measured
      // good across trotting's entire range (47.0 s at 2.0, 33.2 s at 3.0,
      // 32.2 s at 3.1), so the switch bought nothing and broke something.
      // LESSON: a mid-run parameter change is not free. It has to beat the
      // TRANSIENT it causes, not just the steady-state number it targets.
      p.segMs = 22;  p.vMax = 3.1f;  break;
    case 20:                      // walking - 4-beat, 50% duty
      p.segMs = 22;  p.vMax = 2.25f;  break;
    case 4:                       // standing - no travel
      p.segMs = 22;  p.vMax = 0.5f;  break;
    default:                      // untested gaits: the conservative envelope
      p.segMs = 22;  p.vMax = 2.0f;  break;
  }
  // $CTRL_MPC_MS / $CTRL_SWING_H still override, for experiments only.
  if (const char* e = getenv("CTRL_MPC_MS"))  { int v2 = atoi(e); if (v2 >= 10 && v2 <= 80) p.segMs = v2; }
  if (const char* e = getenv("CTRL_SWING_H")) { float h = atof(e); if (h > 0.f && h < 0.3f) p.swingH = h; }
  return p;
}

/**
 * GAIT/SPEED PARAMETER SCHEDULE - the application half.
 *
 * Both parameters are applied only where a change cannot produce a
 * discontinuity, which is what makes a speed or gait transition seamless
 * rather than a step input into a running gait:
 *
 *  - SEGMENT changes the gait clock. `OffsetDurationGait::setIterations` derives
 *    phase as `(counter % (itersPerMPC * nIterations)) / (itersPerMPC *
 *    nIterations)`, so changing itersPerMPC mid-cycle teleports the phase and
 *    every foot with it. It is therefore applied ONLY as the cycle wraps.
 *  - SWING HEIGHT feeds a Bezier evaluated over the whole swing, so changing it
 *    mid-flight moves the foot's target under it. It is latched PER LEG at that
 *    leg's swing start (MIT's own `if(firstSwing[i])` guard around setHeight is
 *    commented out upstream, which is why this has to be done here).
 */
void ConvexMPCLocomotion::applySchedule(int gaitNumber, float speedCmd, Gait* activeGait) {
  const SchedParams p = scheduleFor(gaitNumber, speedCmd);
  _segMsPending = p.segMs;

  // ---- ENVELOPE CLAMP: never run a gait faster than that gait can hold ----
  // A gait switch is instantaneous; a speed change is not. Commanding
  // "switch to walking" while travelling 2.5 m/s ran walking 0.25 m/s past its
  // own ceiling and put the robot down inside two seconds - measured, with the
  // fall landing between the [profile] gait switch and the next speed step.
  // The commander should not have to know each gait's envelope and sequence
  // deceleration before the switch; the controller refuses to exceed it and
  // DECELERATES INTO the new gait instead. Rate-limited so the clamp itself is
  // not a step input (2.0 m/s^2, about the ramp rate the gaits tolerate).
  if (_x_vel_des > p.vMax) {
    const float decel = 2.0f * dt;
    _x_vel_des = std::max(p.vMax, _x_vel_des - decel);
    static int nclamp = 0;
    if ((nclamp++ % 250) == 0) {
      shmtrace::logf(0.0, "[SCHED] gait=%d ENVELOPE CLAMP: vx %.2f -> %.2f (gait max %.2f)",
             gaitNumber, speedCmd, _x_vel_des, p.vMax);
    }
  }

  // ---- DO NOT STACK PERTURBATIONS ON A GAIT CHANGE ----
  // A gait switch already rearranges which feet are down; landing a segment
  // change on the same tick perturbs the clock at the same moment and the two
  // compound. Measured: walking -> trotting at 2.0 m/s (both speeds legal for
  // both gaits) fell immediately, with the [SCHED] segment change 22 -> 26 ms
  // logged on the same line as the switch and roll reaching -23 deg.
  // The gait gets a settling window to itself before the clock is touched.
  static int lastGaitSeen = -1;
  static int64_t gaitChangedMs = 0;
  const int64_t nowms = nowMs();
  if (gaitNumber != lastGaitSeen) {
    lastGaitSeen = gaitNumber;
    gaitChangedMs = nowms;
  }
  const bool settling = (nowms - gaitChangedMs) < 500;   // ~2 gait cycles

  const int wantIters = std::max(1, (int)std::lround(p.segMs / (1000.f * dt)));
  if (wantIters != iterationsBetweenMPC && !settling) {
    // Only at a cycle boundary. getCurrentGaitPhase() returns the integer
    // SEGMENT INDEX within the cycle (not a 0-1 phase), so index 0 is the wrap.
    const int seg = activeGait ? activeGait->getCurrentGaitPhase() : 0;
    if (seg == 0) {
      shmtrace::logf(0.0, "[SCHED] gait=%d v=%.2f  segment %d -> %d ms (iters %d -> %d)",
             gaitNumber, speedCmd, _segMsCurrent ? _segMsCurrent : p.segMs,
             p.segMs, iterationsBetweenMPC, wantIters);
      // Write the DEFAULT, not just the live value: run() calls
      // recompute_timing(default_iterations_between_mpc) every tick, which would
      // otherwise clobber this on the very next iteration. (Measured: the
      // schedule logged "iters 11 -> 13" 261 times in one run and nothing ever
      // changed, because the assignment was overwritten before it could act.)
      default_iterations_between_mpc = wantIters;
      iterationsBetweenMPC = wantIters;
      dtMPC = dt * iterationsBetweenMPC;
      _segMsCurrent = p.segMs;
      setup_problem(dtMPC, horizonLength, 0.4, 120);
    }
  } else if (_segMsCurrent == 0) {
    _segMsCurrent = p.segMs;
  }
}

bool ConvexMPCLocomotion::zeroVelHold() {
  // UNCONDITIONAL, for the same reason as heading hold: there is no operating
  // condition in which engaging a dynamic gait against a zero velocity command
  // is preferable. A real operator does exactly this by hand - stand, let the
  // gait settle, then push the stick - and Unitree's own build raises a
  // transition request rather than swinging legs at a standstill.

  const float kZeroVel = 0.01f;      // Unitree's threshold, .rodata 0x2665e0
  // MEASURED AND REVERTED (2026-08-24): raising this to 600 ticks (1.2 s
  // of trot-in-place before the standing gait engages, to actively shed
  // arrival sway at a stop) made the oval's stop WORSE, not better - 7 of
  // 8 stop arrivals tipped sideways at a consistent roll ~88 deg with the
  // long hold, vs ~1 in 3 at the stock 50 ms (A/B'd via
  // CTRL_ZEROVEL_HOLD=25, 2 PASS / 1 tip). The plausible-sounding idea
  // ("a stepping trot balances actively") loses to the data: a trot
  // commanded to zero while the body still carries ~0.9 m/s of arrival
  // momentum fights it for the whole window instead of letting the stand
  // catch it. Do not raise this again without an interleaved A/B.
  static const int kHold =
      getenv("CTRL_ZEROVEL_HOLD") ? atoi(getenv("CTRL_ZEROVEL_HOLD")) : 25;

  const bool zeroish =
      std::sqrt(_x_vel_des * _x_vel_des + _y_vel_des * _y_vel_des) < kZeroVel &&
      std::fabs(_yaw_turn_rate) < kZeroVel;

  static int zeroTicks = 0;
  zeroTicks = zeroish ? (zeroTicks + 1) : 0;
  return zeroTicks >= kHold;
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode) {
  //iterationsBetweenMPC = 30;
  // ASYNC: dispatch whenever the worker is idle, not only on segment
  // boundaries. MIT's synchronous solver refreshes forces within one segment
  // of any gait-table change by construction. The async port only snapshotted
  // at boundaries, so a table flip (standing -> trot at engage) left the legs
  // running the PREVIOUS gait's forces for up to a full segment plus the
  // solve time - ~77 ms of half-support with a swing pair already lifting,
  // which collapsed the robot at every gait engage. Continuous dispatch cuts
  // the exposure to the solve time alone (~32 ms) and raises the effective
  // MPC rate to whatever the worker can sustain (~30 Hz).
  // Boundary-only dispatch: the pipeline in run() hands this the NEXT
  // segment's contact table, and the solve (32 ms) fits inside the 45 ms
  // segment, so each boundary applies the previous solve and starts the next.
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};


    //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

    if(current_gait == 4)
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)_body_height/*fsm->main_control_settings.p_des[2]*/,
        0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    }

    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {(float)rpy_comp[0],  // 0
        (float)rpy_comp[1],    // 1
        _yaw_des,    // 2
        //yawStart,    // 2
        xStart,                                   // 3
        yStart,                                   // 4
        (float)_body_height,      // 5
        0,                                        // 6
        0,                                        // 7
        _yaw_turn_rate,  // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        0};                                       // 11

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }

      // BALLISTIC VERTICAL REFERENCE FOR GAITS WITH A FLIGHT PHASE.
      //
      // Above, every horizon step gets z = _body_height and vz = 0 - MIT's
      // stock reference. For a gait that is airborne by design that is
      // incoherent with the gait's own contact schedule. MIT's `pronking` is
      // offsets(0,0,0,0)/durations(4,4,4,4): SIX of ten segments with all four
      // feet off the ground. To stay up for 60% of the cycle the body has to be
      // LAUNCHED, and a reference that says "hold 0.30 m, zero vertical
      // velocity" gives the optimiser no reason to ever build vertical
      // velocity. So the MPC never launches, the schedule lifts the feet
      // anyway, and the robot falls - which is exactly what pronking and
      // galloping do here, immediately on engagement, level and sinking.
      //
      // Build the reference the schedule actually implies instead: at the last
      // stance step before a flight of Tf segments, command the takeoff
      // velocity that returns the body to _body_height, vz = g*Tf*dt/2; during
      // flight integrate ballistically; during stance hold nominal.
      // $SIM_BALLISTIC_Z=0 restores stock MIT.
      {
        // DEFAULT OFF - measured to give no benefit, and harmful paired with
        // the flight cost gate (see SolverMPC). Stock MIT already commands
        // sensible pronking forces (39-42 N/foot); the reference was not the
        // thing that was broken.
        static const bool ballistic = getenv("SIM_BALLISTIC_Z") &&
                                      atoi(getenv("SIM_BALLISTIC_Z")) != 0;
        if (ballistic) {
          auto isFlight = [&](int k) {
            if (k < 0 || k >= horizonLength) return false;
            const int* c = mpcTable + k * 4;
            return c[0] == 0 && c[1] == 0 && c[2] == 0 && c[3] == 0;
          };
          bool anyFlight = false;
          for (int i = 0; i < horizonLength && !anyFlight; ++i) anyFlight = isFlight(i);

          if (anyFlight) {
            float z_ref  = _body_height;
            float vz_ref = 0.f;
            for (int i = 0; i < horizonLength; i++) {
              if (isFlight(i)) {
                vz_ref -= 9.81f * dtMPC;          // ballistic
              } else {
                // stance: we have authority. If flight starts next step, this
                // is the launch - command the impulse that gets us back.
                int Tf = 0;
                for (int k = i + 1; k < horizonLength && isFlight(k); ++k) ++Tf;
                if (Tf > 0) {
                  vz_ref = 0.5f * 9.81f * ((float)Tf * dtMPC);
                } else {
                  z_ref = _body_height;
                  vz_ref = 0.f;
                }
              }
              z_ref += vz_ref * dtMPC;
              trajAll[12*i + 5]  = z_ref;
              trajAll[12*i + 11] = vz_ref;
            }
          }
        }
      }
    }
    Timer solveTimer;

    if(_parameters->cmpc_use_sparse > 0.5) {
      solveSparseMPC(mpcTable, data);
    } else {
      solveDenseMPC(mpcTable, data);
    }
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};

  // $CTRL_MPC_Q selects an alternate state-cost vector.
  //   1 = Unitree's second vector, recovered from Legged_sport .rodata 0x2fe390:
  //       {0.5,0.5,10, 20,20,15, 0.1,0.1,1, 0.5,0.5,0.5}
  //       Ten times MIT's position weight (20 vs 2) and a LOWER z weight (15 vs
  //       50), with non-zero rate weights. MIT's own vector appears verbatim
  //       twice in the same binary, so this is a deliberate second tuning that
  //       Unitree ships for some other mode - worth testing against speed,
  //       where holding commanded position matters more than holding height.
  {
    static const int qsel = getenv("CTRL_MPC_Q") ? atoi(getenv("CTRL_MPC_Q")) : 0;
    if (qsel == 1) {
      const float qU[12] = {0.5f,0.5f,10.f, 20.f,20.f,15.f,
                            0.1f,0.1f,1.f,  0.5f,0.5f,0.5f};
      for (int i = 0; i < 12; i++) Q[i] = qU[i];
    }
  }

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  static const float alpha_env = getenv("CTRL_MPC_ALPHA") ? atof(getenv("CTRL_MPC_ALPHA")) : 4e-5f;
  float alpha = alpha_env; // input-cost weight (MIT default 4e-5); env knob for
                           // short-horizon retunes
  //float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if(alpha > 1e-4) {
    shmtrace::logf(0.0, "Alpha was set too high (%g) adjust to 1e-5", (double)alpha);
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  // NOTE: setup_problem() is NOT called here any more. It calls
  // resize_qp_mats(), which setZero()s S, fmat, qH, qg and friends - and with
  // the solve now on a worker thread, doing that from the control thread wipes
  // the matrices the worker is mid-way through building. That is exactly what
  // happened: the worker's problem came out with |S| = 0 and |fmat| = 0, i.e.
  // no state cost and no friction constraints, so the QP reduced to
  // min alpha*||u||^2 and both solvers correctly returned ZERO force.
  // The worker calls setup_problem itself, on its own thread, before each solve.
  if (!_mpcAsync) setup_problem(dtMPC,horizonLength,0.4,MPC_F_MAX);
  update_x_drag(x_comp_integral);
  if(vxy[0] > 0.3 || vxy[0] < -0.3) {
    //x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
      _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  //t1.stopPrint("Setup MPC");

  // ---- hand the solve to the worker, keep using the latest solution --------
  // Everything above (weights, alpha, x_drag, solver settings) is unchanged
  // MIT; only WHERE the solve runs changes. setup_problem/update_problem_data/
  // get_solution touch solver globals and are not re-entrant, so ONLY the
  // worker ever calls them.
  if (!_mpcAsync) {                    // stock behaviour: solve right here
    MpcSnapshot in;
    for (int i = 0; i < 3; ++i) { in.p[i]=p[i]; in.v[i]=v[i]; in.w[i]=w[i]; }
    for (int i = 0; i < 4; ++i) in.q[i]=q[i];
    for (int i = 0; i < 12; ++i) in.r[i]=r[i];
    in.yaw=yaw; in.alpha=alpha; in.horizon=horizonLength; in.dtMPC=dtMPC;
    in.rBody=seResult.rBody;
    for (int i = 0; i < 12*horizonLength; ++i) in.traj[i]=trajAll[i];
    for (int i = 0; i < 4*horizonLength; ++i) in.table[i]=mpcTable[i];
    in.t_ms = nowMs();
    Vec3<float> trInl[3][4];
    _runSolve(in, trInl);
    for (int leg = 0; leg < 4; ++leg) {
      f_ff[leg]   = -seResult.rBody * trInl[0][leg];
      Fr_des[leg] = trInl[0][leg];
    }
    return;
  }
  {
    std::unique_lock<std::mutex> lk(_mpcMtx);
    if (getenv("STM32MP1_MPC_IN")) {
      static int _dsp = 0;
      if ((++_dsp % 22) == 1)
        shmtrace::logf(0.0, "[DSP] attempt #%d busy=%d haveSol=%d", _dsp, (int)_mpcBusy,
               (int)_mpcHaveSolution.load());
    }
    if (!_mpcBusy) {
      for (int i = 0; i < 3; ++i) { _mpcIn.p[i]=p[i]; _mpcIn.v[i]=v[i]; _mpcIn.w[i]=w[i]; }
      for (int i = 0; i < 4; ++i) _mpcIn.q[i]=q[i];
      for (int i = 0; i < 12; ++i) _mpcIn.r[i]=r[i];
      _mpcIn.yaw = yaw;
      _mpcIn.alpha = alpha;
      _mpcIn.horizon = horizonLength;
      _mpcIn.dtMPC = dtMPC;
      _mpcIn.rBody = seResult.rBody;
      _mpcIn.t_ms = nowMs();
      int ntraj = 12 * horizonLength, ntab = 4 * horizonLength;
      if (ntraj > (int)(sizeof(_mpcIn.traj)/sizeof(float))) ntraj = sizeof(_mpcIn.traj)/sizeof(float);
      if (ntab  > (int)(sizeof(_mpcIn.table)/sizeof(int)))  ntab  = sizeof(_mpcIn.table)/sizeof(int);
      for (int i = 0; i < ntraj; ++i) _mpcIn.traj[i] = trajAll[i];
      for (int i = 0; i < ntab;  ++i) _mpcIn.table[i] = mpcTable[i];
      _mpcRequest = true;
      _mpcBusy = true;
      _mpcCv.notify_one();
    }
    // BOOTSTRAP. The first solve takes 73-240 ms on this board, and until it
    // lands f_ff is zero - with MIT's Kp_stance = 0 that is zero support, so
    // the robot collapses before its first MPC solution ever arrives and every
    // later solve then sees a robot already on the floor (which is why the
    // solutions came back all-zero). Hold static equilibrium until then:
    // bodyweight shared over the feet the gait currently calls stance. This is
    // the same quantity BALANCE_STAND is already holding the robot up with, so
    // it is continuous across the transition.
    if (!_mpcHaveSolution.load()) {
      int nStance = 0;
      for (int leg = 0; leg < 4; ++leg) if (mpcTable[leg]) nStance++;
      if (nStance < 1) nStance = 1;
      const float W = 13.1f * 9.81f;
      for (int leg = 0; leg < 4; ++leg) {
        Vec3<float> fw(0.f, 0.f, mpcTable[leg] ? (W / (float)nStance) : 0.f);
        f_ff[leg]   = -seResult.rBody * fw;
        Fr_des[leg] = fw;
      }
    }
    // (The solution is applied in run() every tick, indexed by elapsed time -
    // see _frTraj in the header. Nothing further to publish here.)
  }
}

//! The original MIT solve, run on the worker thread against a snapshot.
void ConvexMPCLocomotion::_runSolve(const MpcSnapshot& in, Vec3<float> frTraj[3][4]) {
  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};
  setup_problem(in.dtMPC, in.horizon, 0.4, MPC_F_MAX);
  update_x_drag(x_comp_integral);
  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
      _parameters->jcqp_sigma, _parameters->jcqp_alpha,
      _parameters->jcqp_terminate, _parameters->use_jcqp);
  if (getenv("STM32MP1_MPC_IN")) {
    static int _ic = 0;
    if ((++_ic % 10) == 1) {
      shmtrace::logf(0.0, "[MPCIN] p=%.2f %.2f %.2f  v=%.2f %.2f %.2f  yaw=%.2f h=%d",
             in.p[0], in.p[1], in.p[2], in.v[0], in.v[1], in.v[2], in.yaw, in.horizon);
      shmtrace::logf(0.0, "[MPCIN] table[0..7]=%d%d%d%d %d%d%d%d  traj[0..5]=%.2f %.2f %.2f %.2f %.2f %.2f",
             in.table[0],in.table[1],in.table[2],in.table[3],
             in.table[4],in.table[5],in.table[6],in.table[7],
             in.traj[0],in.traj[1],in.traj[2],in.traj[3],in.traj[4],in.traj[5]);
      shmtrace::logf(0.0, "[MPCIN] r(foot rel CoM) x=%.2f %.2f %.2f %.2f  z=%.2f %.2f %.2f %.2f",
             in.r[0],in.r[1],in.r[2],in.r[3], in.r[8],in.r[9],in.r[10],in.r[11]);
    }
  }
  update_problem_data_floats((float*)in.p, (float*)in.v, (float*)in.q, (float*)in.w,
                             (float*)in.r, in.yaw, Q, (float*)in.traj,
                             in.alpha, (int*)in.table);
  // Read the first three SEGMENTS of the solution, not just step 0 - the
  // solver computes the whole horizon anyway, and the control loop needs the
  // later steps to compensate its own solve latency. World frame; rotation
  // into the body frame happens at APPLICATION time with the current attitude.
  for (int st = 0; st < 3; ++st) {
    int step = (st < in.horizon) ? st : in.horizon - 1;
    for (int leg = 0; leg < 4; ++leg)
      for (int axis = 0; axis < 3; ++axis)
        frTraj[st][leg][axis] = get_solution(12*step + leg*3 + axis);
  }
}

void ConvexMPCLocomotion::_mpcWorker() {
  // The worker MUST NOT compete with the 500 Hz control loop. It inherits the
  // creator's SCHED_FIFO priority (49 here), and with the control loop and the
  // motor task also at FIFO 49 on a DUAL-core A7, an equal-priority 60 ms solve
  // preempts the very thing it was supposed to protect - moving the solve off
  // the control thread bought nothing at all until this was fixed.
  // Drop to SCHED_OTHER so the RT threads always win, and pin to the second
  // core so the solve does not evict the control loop's cache either.
  {
    // Priority: BELOW the 500 Hz control loop and motor task (both FIFO 49) so
    // it can never preempt them, but still real-time so it is not starved to
    // death by them - as SCHED_OTHER was: one solve took 359 ms instead of 60,
    // and the run completed a single MPC solve in 28 s.
#ifdef __linux__
    int prio = getenv("CTRL_MPC_PRIO") ? atoi(getenv("CTRL_MPC_PRIO")) : 20;
    struct sched_param sp; sp.sched_priority = prio;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      sp.sched_priority = 0;
      pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
      shmtrace::logf(0.0, "[Convex MPC] worker: SCHED_OTHER (FIFO %d refused)", prio);
    } else {
      shmtrace::logf(0.0, "[Convex MPC] worker: SCHED_FIFO %d", prio);
    }
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(1, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
#else
    // Host (Mac-first) build: no SCHED_FIFO/affinity - the machine is ~30x the
    // board and the default scheduler is fine for iterating the math.
    shmtrace::logf(0.0, "[Convex MPC] worker: host build, default scheduling");
#endif
  }
  while (!_mpcQuit.load()) {
    MpcSnapshot in;
    {
      std::unique_lock<std::mutex> lk(_mpcMtx);
      _mpcCv.wait(lk, [&]{ return _mpcRequest || _mpcQuit.load(); });
      if (_mpcQuit.load()) return;
      in = _mpcIn;
      _mpcRequest = false;
    }
    Timer _tsolve;
    Vec3<float> tr[3][4];
    _runSolve(in, tr);
    // NaN GUARD. An ill-conditioned or diverging QP can return non-finite
    // forces; those propagate through J^T into the joint torques and Gazebo
    // rejects them ("Invalid joint force value [nan]"). On real hardware a NaN
    // torque command is undefined behaviour, so never publish one - drop the
    // solution and keep the previous (or the bootstrap) instead.
    bool finite = true;
    for (int st = 0; st < 3 && finite; ++st)
      for (int leg = 0; leg < 4 && finite; ++leg)
        for (int a = 0; a < 3; ++a)
          if (!std::isfinite(tr[st][leg][a])) finite = false;
    if (!finite) {
      static int _nanc = 0;
      ++_nanc;
      if ((_nanc % 20) == 1)
        shmtrace::logf(0.0, "[MPCW] REJECTED non-finite solution (%d so far)", _nanc);
      // CLEAR THE BUSY FLAG. The original `continue` skipped the publish block
      // below - which is also what releases _mpcBusy - so a single non-finite
      // solution silenced the MPC for the rest of the run: no new dispatch is
      // ever accepted while busy, the stale trajectory keeps being applied at
      // kSeg=2 (whose forces for the current stance feet are ~zero), and the
      // robot sags on kp=8 joint PD alone for ~1.5 s until the orientation
      // E-stop. Measured in the bridge command dump: all four knee tau_ff
      // collapse from -13 Nm to 0.3 Nm at t=9.85 and never recover.
      {
        std::lock_guard<std::mutex> lk(_mpcMtx);
        _mpcBusy = false;
      }
      continue;
    }
    if (getenv("STM32MP1_EST_DBG")) {
      static int _sc = 0;
      if ((++_sc % 10) == 1)
        shmtrace::logf(0.0, "[MPCW] solve #%d took %.1f ms, world fz=[%.0f %.0f %.0f %.0f]", _sc, _tsolve.getMs(),
               tr[0][0][2], tr[0][1][2], tr[0][2][2], tr[0][3][2]);
    }
    {
      std::lock_guard<std::mutex> lk(_mpcMtx);
      for (int st = 0; st < 3; ++st)
        for (int leg = 0; leg < 4; ++leg)
          _frTraj[st][leg] = tr[st][leg];
      _snapMs = in.t_ms;
      _snapDtMPC = in.dtMPC;
      _mpcBusy = false;
    }
    _mpcHaveSolution.store(true);
  }
}

void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data) {
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for(int i = 0; i < horizonLength; i++) {
    contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
  }

  for(int i = 0; i < horizonLength; i++) {
    for(u32 j = 0; j < 12; j++) {
      _sparseTrajectory[i][j] = trajAll[i*12 + j];
    }
  }

  Vec12<float> feet;
  for(u32 foot = 0; foot < 4; foot++) {
    for(u32 axis = 0; axis < 3; axis++) {
      feet[foot*3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for(u32 foot = 0; foot < 4; foot++) {
    Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
    //printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC() {
  Mat3<double> baseInertia;
#ifdef USE_GO1_MODEL
  // Match RobotState (the dense-MPC path): Go1 is 13.1 kg, not the 9 kg
  // mini-cheetah. Unused while cmpc_use_sparse is 0, but a 30% mass error
  // sitting in the alternate solver is a trap for whoever enables it.
  baseInertia << 0.102, 0, 0,
              0, 0.379, 0,
              0, 0, 0.352;
  double mass = 13.1;
#else
  baseInertia << 0.07, 0, 0,
              0, 0.26, 0,
              0, 0, 0.242;
  double mass = 9;
#endif
  double maxForce = 120;

  std::vector<double> dtTraj;
  for(int i = 0; i < horizonLength; i++) {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(0.4);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}

