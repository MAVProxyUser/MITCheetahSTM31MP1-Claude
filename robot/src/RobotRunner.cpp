/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include <atomic>
#include <cstdio>   // snprintf, for the non-finite field report
#include <chrono>
#include <unistd.h>

// Defined further down next to setEdamp; declared here because the fall
// detector (earlier in the file) gates its z-branch on it.
extern std::atomic<bool> g_fallZEnable;

// HOST STALL flag - set by the stall detector in run(), read by the mission
// thread (mit_sim_main) so it can stop translating and hold. Defined below
// next to setEdamp with the other cross-thread controls.
extern std::atomic<bool> g_hostStall;

#include <cmath>
#include <cstdlib>
#include "RobotRunner.h"
#include <Controllers/SprawlGuard.h>
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
// Per-tick SHM tracing, built to root-cause the fleet-only fast-fall
// mystery (clean control-loop timing rules out a scheduling stall, so
// whatever is happening lives in the control DATA at a resolution the
// existing 1-in-50-tick printf logging cannot see). Lives under
// stm32mp1/gazebo/ rather than robot/include/ because it is a SITL
// debugging tool, not part of the hardware-facing interface - lazily
// opens shared memory only on first log() call, so a board build that
// never calls it never pays for the mmap. See ShmTrace.h's own header
// for the full design (lock-free ring buffer, separate reaper process).
#include "../../stm32mp1/gazebo/ShmTrace.h"

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
  shmtrace::logf(0.0, "[RobotRunner] initialize");

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
  shmtrace::logf(0.0, "[RobotRunner] absAiding at init = %p", (void*)absAiding);
  if (absAiding) _stateEstimator->setAbsoluteAiding(absAiding);

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
  // ACTUAL TICK DURATION, measured once here and shared by everything below
  // that used to either assume a fixed 2 ms (the estimator's own
  // integration - see setActualDt() below) or re-measure it separately
  // (the host-stall detector further down, which now reads dt_ms_actual
  // instead of keeping its own clock). First call has no prior sample to
  // diff against, so it falls back to the nominal dt rather than reporting
  // a huge or garbage duration.
  static auto _lastTickTime = std::chrono::steady_clock::now();
  const auto _nowTickTime = std::chrono::steady_clock::now();
  static bool _firstTick = true;
  const double dt_ms_actual = _firstTick ? controlParameters->controller_dt * 1000.0
      : std::chrono::duration<double, std::milli>(_nowTickTime - _lastTickTime).count();
  _lastTickTime = _nowTickTime;
  _firstTick = false;
  // Elapsed mission time by NOMINAL dt accumulation (matches every other
  // "t=%.1fs" printed elsewhere in this port, e.g. RobotRunner's own [YAW]/
  // [ESTERR] traces use n*0.002f) - shared by every shmtrace::log() call
  // site in this function so the whole trace uses ONE consistent clock.
  static double _shmElapsed = 0.0;
  _shmElapsed += controlParameters->controller_dt;

  // Run the state estimator step
  //_stateEstimator->run(cheetahMainVisualization);
  // DT-AWARE: feed the estimator the tick duration actually measured above,
  // not the fixed nominal controller_dt every estimator integrated against
  // before this existed. See StateEstimatorData::actualDt's comment
  // (StateEstimatorContainer.h) and LinearKFPositionVelocityEstimator::run's
  // own comment (PositionVelocityEstimator.cpp) for the full mechanism and
  // why a stalled tick otherwise silently under-integrates.
  _stateEstimator->setActualDt((float)(dt_ms_actual / 1000.0));
  // A LATCHED FALL OUTRANKS EVERYTHING BELOW. Hold the legs at zero and
  // keep looping: the point of latch-limp is that the process stays alive
  // and answering, so the controller must not be allowed to re-command the
  // legs on the next tick.
  if (_fallLatched) {
    for (int leg = 0; leg < 4; leg++) _legController->commands[leg].zero();
    finalizeStep();
    return;
  }
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
  bool _shmFiniteOk = true;   // fed to the per-tick SHM trace at the end of run()
  {
    const auto& r = _stateEstimate;
    bool bad = false;
    for (int i = 0; i < 3 && !bad; ++i)
      if (!std::isfinite(r.position[i]) || !std::isfinite(r.vWorld[i]) ||
          !std::isfinite(r.vBody[i])    || !std::isfinite(r.rpy[i])) bad = true;
    for (int i = 0; i < 4 && !bad; ++i)
      if (!std::isfinite(r.orientation[i])) bad = true;
    if (bad) {
      _shmFiniteOk = false;
      static int nanCount = 0;
      ++nanCount;
      // NAME THE CULPRIT (OPEN-6, 2026-08-28). The blip fires in ~2/3 of all
      // runs, always in the boot window, and was never diagnosable because
      // this line said only THAT the estimate was non-finite, never WHICH
      // field or WHEN. A guard that reinitialises without reporting what it
      // caught cannot be investigated - the whole item stalled on that.
      char who[160]; int wl = 0;
      const char* nm3[4] = {"pos", "vWorld", "vBody", "rpy"};
      const float* v3[4] = {r.position.data(), r.vWorld.data(),
                             r.vBody.data(), r.rpy.data()};
      for (int f = 0; f < 4 && wl < 120; ++f)
        for (int i = 0; i < 3 && wl < 120; ++i)
          if (!std::isfinite(v3[f][i]))
            wl += snprintf(who + wl, sizeof(who) - wl, "%s[%d] ", nm3[f], i);
      for (int i = 0; i < 4 && wl < 120; ++i)
        if (!std::isfinite(r.orientation[i]))
          wl += snprintf(who + wl, sizeof(who) - wl, "quat[%d] ", i);
      if (!wl) snprintf(who, sizeof(who), "(none? race) ");
      shmtrace::logf(_shmElapsed,
                     "[stm32mp1] NONFINITE-FIELDS (%d) t=%.3f iter=%llu bad: %s"
                     " | quat=%.3f %.3f %.3f %.3f pos=%.3f %.3f %.3f",
                     nanCount, _shmElapsed,
                     (unsigned long long)_iterations, who,
                     (double)r.orientation[0], (double)r.orientation[1],
                     (double)r.orientation[2], (double)r.orientation[3],
                     (double)r.position[0], (double)r.position[1],
                     (double)r.position[2]);
      shmtrace::logf(_shmElapsed, "[stm32mp1] STATE ESTIMATE WENT NON-FINITE (%d) - reinitialising", nanCount);
      initializeStateEstimator(_cheaterModeEnabled);
      _stateEstimator->run();
    }
  }

  // [ESTERR] ESTIMATOR ERROR AGAINST GROUND TRUTH ($SIM_ESTERR=1).
  // The measured wall on this port is the state estimator: every gait loses one
  // to two speed rungs running on its own estimate versus ground truth, and trot
  // goes from 100 m at 1.0 m/s to falling at 4.4 m. That gap was previously
  // "explained" by switching the estimator OFF (cheater mode), which measures
  // nothing and is now deleted. This measures it instead: truth is LOGGED beside
  // the estimate, never fed to it, so the number describes the real controller.
  // Watch dv (forward-velocity error) in the seconds before a fall - the MPC's
  // Raibert foothold and state cost both consume vBody, so a velocity estimate
  // that lags is a foothold placed in the wrong place.
  if (cheaterState) {
    static const bool esterr = getenv("SIM_ESTERR") && atoi(getenv("SIM_ESTERR")) != 0;
    if (esterr) {
      static int n = 0;
      if ((n++ % 50) == 0) {                       // 10 Hz at a 500 Hz loop
        const auto& e = _stateEstimate;
        Vec3<float> pT = cheaterState->position.template cast<float>();
        Vec3<float> vT = cheaterState->vBody.template cast<float>();
        shmtrace::logf(_shmElapsed,
               "[ESTERR] t=%.2f pT=%.3f,%.3f,%.3f pE=%.3f,%.3f,%.3f "
               "vT=%.3f,%.3f,%.3f vE=%.3f,%.3f,%.3f dvx=%+.3f dp=%.3f",
               n * 0.002f,
               pT[0], pT[1], pT[2], e.position[0], e.position[1], e.position[2],
               vT[0], vT[1], vT[2], e.vBody[0], e.vBody[1], e.vBody[2],
               e.vBody[0] - vT[0], (e.position - pT).norm());
      }
    }
  }

  // [YAW] TURNING PERFORMANCE ($SIM_YAWDBG=1).
  // Commanded yaw rate vs ACHIEVED yaw rate, plus the attitude the robot holds
  // while turning. Needed for three things that all live on this axis: the
  // maximum sustainable yaw rate, cornering (turning while translating), and
  // spinning in place to face a new waypoint without falling over.
  // Ground truth is LOGGED for comparison and never fed to the controller.
  if (cheaterState) {
    static const bool ydbg = getenv("SIM_YAWDBG") && atoi(getenv("SIM_YAWDBG")) != 0;
    if (ydbg) {
      static int ny = 0;
      static float lastTrueYaw = 0.f; static bool haveLast = false;
      if ((ny++ % 50) == 0) {                      // 10 Hz at a 500 Hz loop
        const auto& e = _stateEstimate;
        // true yaw from the sim quaternion (w,x,y,z)
        Quat<double> q = cheaterState->orientation;
        const double sy = 2.0 * (q[0]*q[3] + q[1]*q[2]);
        const double cy = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3]);
        const float trueYaw = (float)std::atan2(sy, cy);
        float trueRate = 0.f;
        if (haveLast) {
          float d = trueYaw - lastTrueYaw;
          while (d >  M_PI) d -= 2.f * M_PI;       // unwrap
          while (d < -M_PI) d += 2.f * M_PI;
          trueRate = d / 0.1f;                     // 10 Hz sampling
        }
        lastTrueYaw = trueYaw; haveLast = true;
        const float wzCmd = driverCommand ? driverCommand->rightStickAnalog[0] : 0.f;
        shmtrace::logf(_shmElapsed,
               "[YAW] t=%.2f wzCmd=%+.3f wzEst=%+.3f wzTrue=%+.3f "
               "yawEst=%+.3f yawTrue=%+.3f roll=%+.1f pitch=%+.1f z=%.3f",
               ny * 0.002f, wzCmd, e.omegaBody[2], trueRate,
               e.rpy[2], trueYaw,
               e.rpy[0] * 57.2958f, e.rpy[1] * 57.2958f, e.position[2]);
      }
    }
  }

  /*
   * HOST STALL DETECTOR.
   *
   * The control loop is WALL-CLOCK timed: it computes forces for a 2 ms
   * step and the simulator integrates them over however long the tick
   * actually took. A scheduling stall therefore multiplies every commanded
   * force by (actual/target) - a 16 ms tick is an 8x impulse - and it is
   * invisible in every other instrument, because the bridge's command rate
   * and the estimator both look fine either side of it. Measured on this
   * Mac: a genuine stall shows up as ONE tick at 13-17 ms across all three
   * dogs of a fleet in the same instant, against a clean 2.5-3.5 ms in the
   * runs either side.
   *
   * A single spike cannot be predicted, only reacted to - so the useful
   * definition of "pre-emptive" is: the FIRST overrun latches a stall and
   * the mission stops being vulnerable (stop translating, hold, and lie
   * down if it persists) instead of continuing to sprint into whatever
   * comes next. Over-4 ms ticks are essentially absent from healthy runs
   * here (over4=0 of ~140 samples), so this is a sensitive detector with a
   * cheap false-positive cost: a brief pause.
   *
   * $SIM_STALL_MS (default 4.0 = 2x the 2 ms target), $SIM_STALL_CLEAR
   * (default 500 clean ticks = 1 s), $SIM_STALL_DETECT=0 disables.
   */
  {
    static const bool stall_on =
        !getenv("SIM_STALL_DETECT") || atoi(getenv("SIM_STALL_DETECT")) != 0;
    // 8 ms (4x the 2 ms target), NOT 4 ms. A single 4.8 ms tick is ordinary
    // macOS scheduler jitter - the operator's machine never exceeds ~24% CPU
    // and still produces them - and reacting to one cost a dash outright:
    // the safe hold zeroed a 3.00 m/s sprint and flipped the dog to
    // roll=154. Genuine damaging stalls measured here are 13-17 ms, so the
    // trigger sits above the noise and below the real thing. "Host stall"
    // is also the wrong mental model for the small ones: this is a thread
    // missing its slot, not a saturated machine.
    static const double sick_ms =
        getenv("SIM_STALL_MS") ? atof(getenv("SIM_STALL_MS")) : 8.0;
    static const int clear_ticks =
        getenv("SIM_STALL_CLEAR") ? atoi(getenv("SIM_STALL_CLEAR")) : 500;
    static long armed = 0;
    static int clean = 0;
    // dt_ms_actual: the SAME tick-duration measurement taken at the top of
    // run() for the estimator's benefit - previously this block kept its
    // own separate clock computing the identical quantity a few dozen
    // lines later. One shared measurement, not two clocks that could in
    // principle disagree.
    const double dt_ms = dt_ms_actual;
    // Ignore the first second: startup ticks are irregular by construction
    // (world load, first MPC solve, JPos init) and mean nothing about the host.
    if (stall_on && ++armed > 500) {
      if (dt_ms > sick_ms) {
        clean = 0;
        if (!g_hostStall.exchange(true)) {
          shmtrace::logf(_shmElapsed, "[STALL] control period %.1f ms (limit %.1f) - host stalled, "
                 "mission entering safe hold", dt_ms, sick_ms);
          const auto& rs = _stateEstimate;
          const float c4[4] = {(float)rs.contactEstimate[0], (float)rs.contactEstimate[1],
                                (float)rs.contactEstimate[2], (float)rs.contactEstimate[3]};
          shmtrace::log("STALL", _shmElapsed, rs.rpy[0], rs.rpy[1], rs.rpy[2],
                        rs.omegaBody[0], rs.omegaBody[1], rs.omegaBody[2],
                        rs.vBody[0], rs.vBody[1], rs.vBody[2], rs.position[2],
                        (float)dt_ms, 0, 1, c4);
        }
      } else if (g_hostStall.load() && ++clean >= clear_ticks) {
        g_hostStall = false;
        shmtrace::logf(_shmElapsed, "[STALL] clear after %d clean ticks - safe to resume", clean);
      }
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
      // 0.15 was FAR too close to the operating height and produced false
      // positives that aborted valid runs: with the real estimator the robot
      // WALKS at ~0.175 estimated and dips to 0.149 on the stand-up transient,
      // so a 0.15 threshold killed every real-estimator run at startup while
      // the robot was in fact fine (Gazebo truth 0.197, still walking at 45 s).
      // A collapse puts the belly on the deck at ~0.08-0.10, so 0.10 keeps a
      // real margin below any walking height. NOTE this reads the ESTIMATE, so
      // it inherits estimator error - which is exactly how it misfired.
      static const float fall_z =
          getenv("SIM_FALL_Z") ? atof(getenv("SIM_FALL_Z")) : 0.10f;
      static bool stood = false;
      const float bodyZ = _stateEstimate.position[2];
      if (bodyZ > 0.25f) stood = true;   // clearly standing, not mid-transient

      const float roll  = std::fabs(_stateEstimate.rpy[0]);
      const float pitch = std::fabs(_stateEstimate.rpy[1]);
      const bool tipped    = (roll > fall_rad || pitch > fall_rad);
      // The z test CANNOT tell a commanded lie-down from a collapse - both
      // are a level body descending through 0.10 m. Every "fall during
      // lie-down" of 2026-08-24 was this branch firing DURING the mission's
      // own intentional crouch (verified against Gazebo truth: estimate
      // 0.094, truth 0.11, robot exactly where it was told to go - the
      // process then _exit()s mid-lie-down, which also reads as "the dog
      // never stands back up"). The mission suspends the z branch around
      // its commanded lie-downs via setFallZEnable(); the ATTITUDE branch
      // stays armed throughout - a lie-down is level by definition, so a
      // genuine tip during one still trips.
      const bool collapsed = (g_fallZEnable.load() &&
                              stood && std::isfinite(bodyZ) && bodyZ < fall_z);
      if (tipped || collapsed)
        fallen_for += controlParameters->controller_dt;
      else
        fallen_for = 0.f;
      if (fallen_for >= fall_hold) {
        shmtrace::logf(_shmElapsed,
               "[FALL] %s: roll=%.0f deg pitch=%.0f deg z=%.3f m held %.2f s - "
               "robot is down, stopping (legs go limp via the bridge watchdog)",
               tipped ? "tipped over" : "collapsed",
               _stateEstimate.rpy[0] * 57.2958f, _stateEstimate.rpy[1] * 57.2958f,
               bodyZ, fallen_for);
        // Tag the ring buffer with the confirmed-crash marker BEFORE the
        // _exit() below - the write itself survives an unclean exit (the
        // shm segment is not torn down by _exit skipping destructors), but
        // this record has to actually land first. This is the event the
        // reaper watches for to decide "archive this dog's trace now."
        {
          const auto& rf = _stateEstimate;
          const float c4[4] = {(float)rf.contactEstimate[0], (float)rf.contactEstimate[1],
                                (float)rf.contactEstimate[2], (float)rf.contactEstimate[3]};
          shmtrace::log("FALL", _shmElapsed, rf.rpy[0], rf.rpy[1], rf.rpy[2],
                        rf.omegaBody[0], rf.omegaBody[1], rf.omegaBody[2],
                        rf.vBody[0], rf.vBody[1], rf.vBody[2], bodyZ,
                        (float)fallen_for, tipped ? 2 : 0, 1, c4);
        }
        for (int leg = 0; leg < 4; leg++) _legController->commands[leg].zero();
        finalizeStep();

        // LATCH-LIMP, OR EXIT? THEY ARE DIFFERENT MACHINES (OPEN-13 part 3).
        //
        // Exiting is right for a SWEEP: the harness wants the process gone so
        // the next cell can start, and a simulated robot has nothing to
        // protect. It is WRONG on hardware, and not subtly - process exit
        // also stops whatever was feeding the motor watchdog, so the fault
        // response becomes "stop talking to the motors and disappear" at the
        // exact moment a human needs the machine to be holding still,
        // reporting, and answering. There is nothing left to command a
        // controlled recovery, nothing to keep telemetry flowing while
        // someone walks over, and no way to distinguish "the detector
        // tripped" from "the controller crashed".
        //
        // So the default is now LATCH-LIMP-AND-HOLD: keep the control loop
        // running, keep every leg commanded to zero every tick, keep the
        // watchdog fed and the logs flowing, and stay that way until
        // something deliberately clears it. $SIM_FALL_EXIT=1 restores the
        // process exit for harness use - the sweeps that want it ask for it,
        // rather than every machine inheriting a sweep's convenience.
        //
        // NOTE the ATTITUDE test is the one that should key this on hardware.
        // The z test reads the ESTIMATE, which is why it has misfired twice
        // in this project's history (0.15 killed a day of valid runs; 0.10
        // fired during commanded lie-downs until setFallZEnable gated it).
        // It stays for the sim, where it is cheap and the estimate is good.
        static const bool fall_exit =
            getenv("SIM_FALL_EXIT") && atoi(getenv("SIM_FALL_EXIT")) != 0;
        if (fall_exit) {
          // _exit, not exit: this runs on the control thread while the MPC
          // worker and the UDP threads are still live, and running static
          // destructors underneath them aborts (SIGABRT) instead of exiting
          // cleanly. Flush first, since _exit does not.
          fflush(nullptr);
          _exit(0);
        }
        if (!_fallLatched) {
          _fallLatched = true;
          shmtrace::logf(_shmElapsed,
                 "[FALL] LATCHED LIMP - legs held at zero, control loop still "
                 "running so the watchdog stays fed and telemetry keeps "
                 "flowing. This does NOT clear itself. ($SIM_FALL_EXIT=1 "
                 "exits the process instead, for sweeps.)");
        }
      }
    }
  }

  // STM32MP1 SITL debug: throttled estimator dump so we can compare the
  // estimated body state to Gazebo ground truth (set STM32MP1_EST_DBG=1).
  if (getenv("STM32MP1_EST_DBG")) {
    static int _estdbg = 0;
    if ((++_estdbg % 25) == 0) {   // 20 Hz at 500 Hz
      shmtrace::logf(_shmElapsed,
             "[EST] rpy=%.3f %.3f %.3f pos=%.3f %.3f %.3f vB=%.3f %.3f %.3f wB=%.3f %.3f %.3f",
             _stateEstimate.rpy[0], _stateEstimate.rpy[1], _stateEstimate.rpy[2],
             _stateEstimate.position[0], _stateEstimate.position[1], _stateEstimate.position[2],
             _stateEstimate.vBody[0], _stateEstimate.vBody[1], _stateEstimate.vBody[2],
             _stateEstimate.omegaBody[0], _stateEstimate.omegaBody[1], _stateEstimate.omegaBody[2]);
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
      if(count_ini%1000 ==0)   shmtrace::logf(_shmElapsed, "ESTOP!");
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
              // Foot position (FK) vs commanded target, in the leg frame. During
              // a flight phase every leg is in swing and the swing planner owns
              // all four, so this is where a flight gait either works or does not.
              const float reach = std::sqrt(d.p[0]*d.p[0] + d.p[1]*d.p[1] + d.p[2]*d.p[2]);
              const float reachDes = std::sqrt(c.pDes[0]*c.pDes[0] +
                                               c.pDes[1]*c.pDes[1] + c.pDes[2]*c.pDes[2]);
              shmtrace::logf(_shmElapsed,
                     "[LEG%d] q=%.2f %.2f %.2f p=%.3f %.3f %.3f |p|=%.3f "
                     "pDes=%.3f %.3f %.3f |pDes|=%.3f tff=%.2f %.2f %.2f",
                     leg, d.q[0], d.q[1], d.q[2],
                     d.p[0], d.p[1], d.p[2], reach,
                     c.pDes[0], c.pDes[1], c.pDes[2], reachDes,
                     c.tauFeedForward[0], c.tauFeedForward[1], c.tauFeedForward[2]);
            }
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

  // PER-TICK SHM TRACE. See ShmTrace.h's own header for why this exists
  // (root-causing the fleet-only fast-fall mystery needs per-tick
  // resolution the existing printf logging cannot give without itself
  // perturbing the timing being investigated). $SIM_SHM_TRACE=0 disables
  // it entirely - default ON since the whole design point is "cheap
  // enough to always run", but an explicit escape hatch costs nothing.
  {
    static const bool shm_trace_on =
        !getenv("SIM_SHM_TRACE") || atoi(getenv("SIM_SHM_TRACE")) != 0;
    if (shm_trace_on) {
      const auto& r = _stateEstimate;
      const uint8_t opMode = (_robot_ctrl && _robot_ctrl->isEstopped()) ? 2 : 0;
      const float contact4[4] = {(float)r.contactEstimate[0], (float)r.contactEstimate[1],
                                  (float)r.contactEstimate[2], (float)r.contactEstimate[3]};
      shmtrace::log("tick", _shmElapsed,
                    r.rpy[0], r.rpy[1], r.rpy[2],
                    r.omegaBody[0], r.omegaBody[1], r.omegaBody[2],
                    r.vBody[0], r.vBody[1], r.vBody[2],
                    r.position[2], (float)dt_ms_actual, opMode,
                    _shmFiniteOk ? 1 : 0, contact4);
    }
  }
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
    shmtrace::logf(_iterations * 0.002, "[RobotRunner] Transitioning to Cheater Mode...");
    initializeStateEstimator(true);
    // todo any configuration
    _cheaterModeEnabled = true;
  }

  // check transition from cheater mode:
  if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
    shmtrace::logf(_iterations * 0.002, "[RobotRunner] Transitioning from Cheater Mode...");
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
/*!
 * DAMPING HOLD ($setEdamp > 0) - Unitree's second laydown phase.
 *
 * Their sequence is: FSM_State_StandDown interpolates the joints down, then the
 * legs are put in damping (edampCommand) so the robot settles compliant instead
 * of either holding a pose stiffly or going limp. Set non-zero and the control
 * loop overrides whatever the FSM produced with a pure-damper command, every
 * tick, until it is cleared.
 *
 * NOTE edampCommand is UPSTREAM MIT - Unitree inherited it, they did not write
 * it. Worth remembering: their binary is a MIT fork, so anything the decompile
 * turns up probably already exists in this tree. I nearly re-ported it before
 * checking. (One real difference: MIT's mini-cheetah branch damps kdJoint only,
 * 12 stores; Unitree's writes 24 - both kdJoint AND kdCartesian.)
 */
std::atomic<double> g_edampGain{0.0};
void setEdamp(double d) { g_edampGain = d; }

// Fall detector z-branch gate - see the comment at the `collapsed` test.
// Default ON; the mission turns it off around a COMMANDED lie-down and back
// on once the robot is standing again. Attitude tripping is unaffected.
std::atomic<bool> g_fallZEnable{true};
void setFallZEnable(bool on) { g_fallZEnable = on; }

// See the HOST STALL DETECTOR in run(). Latched on the first control-period
// overrun, cleared after a second of clean ticks; the mission thread reads it
// through hostStallActive() and degrades gracefully instead of sprinting on.
std::atomic<bool> g_hostStall{false};
bool hostStallActive() { return g_hostStall.load(); }

// MIT orientation-ESTOP gate, honored by ControlFSM::safetyPreCheck().
// checkSafeOrientation is a ZERO-debounce 28.6 deg trip evaluated every
// 2 ms tick that force-ESTOPs to PASSIVE - motors cut - on a single bad
// sample. Right for cruise; wrong DURING A COMMANDED STOP: the settle and
// crouch carry transient attitude wobble, one 28.6 deg tick there cuts the
// motors mid-maneuver and CAUSES the very fall it exists to prevent (the
// stop sequence, unaware, then drives a dead robot - measured as a 3-hour
// hung mission). The mission gates it off from the stop command until it
// is standing and driving again; inside that window the DEBOUNCED fall
// detector above (50 deg held 0.5 s) remains the arbiter, so a genuine
// tip still ends the run.
std::atomic<bool> g_orientTripEnable{true};
void setOrientTripEnable(bool on) { g_orientTripEnable = on; }

/*!
 * ATTITUDE TRACE ($CTRL_ATT_DBG = tick decimation) - the raw material for a
 * fall signature.
 *
 * Every failure this port logs ends at `checkSafeOrientation`, |roll| or
 * |pitch| >= 0.5 rad, and the `[FALL]` line printed afterwards describes the
 * aftermath (legs already limp, body settled flat) rather than the event. So
 * nothing in the logs so far says what attitude was doing on the way IN. This
 * prints it: roll, pitch, their rates, yaw rate, and height, all from the
 * estimator, so a signature can be built from many falls instead of guessed
 * from one.
 */
void RobotRunner::attitudeTrace() {
  static int every = -1;
  if (every < 0) {
    const char* e = getenv("CTRL_ATT_DBG");
    every = e ? atoi(e) : 0;
    if (e && every <= 0) every = 10;
  }
  if (every == 0) return;
  static int n = 0;
  if (++n % every) return;
  const auto& r = _stateEstimate;
  shmtrace::logf(_iterations * 0.002,
         "[ATT] t=%.3f roll=%+.4f pitch=%+.4f wx=%+.3f wy=%+.3f wz=%+.3f "
         "z=%.3f vx=%+.2f vy=%+.2f",
         _iterations * 0.002, r.rpy[0], r.rpy[1],
         r.omegaBody[0], r.omegaBody[1], r.omegaBody[2],
         r.position[2], r.vBody[0], r.vBody[1]);
}

/*!
 * LAST-DITCH ROLL ARREST ($CTRL_SPRAWL=1) - see SprawlGuard.h.
 *
 * Sits here rather than in the FSM for the same reason edamp does: it has to
 * overwrite whatever the controller produced, on every tick, after the fact.
 * The FSM's own answer to a roll-out is ESTOP, which is not an answer.
 */
static SprawlGuard g_sprawl;
static Vec3<float> g_sprawlLatch[4];

void RobotRunner::sprawlStep() {
  if (!g_sprawl.update(_stateEstimate.rpy[0], _stateEstimate.rpy[1], 0.002f)) return;

  if (g_sprawl.needsLatch()) {
    // Freeze the posture the legs were in when the guard took over. Commanding
    // qDes = q every tick would be a zero-error PD and hold nothing.
    for (int leg = 0; leg < 4; ++leg) g_sprawlLatch[leg] = _legController->datas[leg].q;
    g_sprawl.markLatched();
  }

  for (int leg = 0; leg < 4; ++leg) {
    auto& c = _legController->commands[leg];
    const float ss = Quadruped<float>::getSideSign(leg);
    if (g_sprawl.mode >= 2) {
      // Full latch - documented in SprawlGuard.h as the version that arrests
      // the roll and then pitches the robot over its own planted feet.
      c.tauFeedForward.setZero();
      c.forceFeedForward.setZero();
      c.kpCartesian.setZero();
      c.kdCartesian.setZero();
      c.qDes  = g_sprawlLatch[leg];
      c.qDes[0] = g_sprawl.abadTarget(ss, g_sprawlLatch[leg][0]);
      c.qDes[1] = g_sprawl.hipTarget(leg, g_sprawlLatch[leg][1]);
      c.qdDes.setZero();
      c.kpJoint = Mat3<float>::Identity() * g_sprawl.kp;
      c.kdJoint = Mat3<float>::Identity() * g_sprawl.kd;
    } else {
      // ABAD ONLY, ADDITIVE. updateCommand() sums joint PD on top of
      // J^T * footForce, so this lays a lateral splay over a gait that carries
      // on swinging its legs - the body widens without the feet being planted.
      // Lateral splay on abad (roll axis) and fore-aft reach on hip (pitch
      // axis). Guarding roll alone just moved the failure into pitch.
      c.qDes[0]  = g_sprawl.abadTarget(ss, g_sprawlLatch[leg][0]);
      c.qdDes[0] = 0.f;
      c.kpJoint(0, 0) = g_sprawl.kp;
      c.kdJoint(0, 0) = g_sprawl.kd;
      if (g_sprawl.pitchSeverity() > 0.f) {
        c.qDes[1]  = g_sprawl.hipTarget(leg, g_sprawlLatch[leg][1]);
        c.qdDes[1] = 0.f;
        c.kpJoint(1, 1) = g_sprawl.kp;
        c.kdJoint(1, 1) = g_sprawl.kd;
      }
    }
  }
}

void RobotRunner::finalizeStep() {
  attitudeTrace();
  sprawlStep();
  const double ed = g_edampGain.load();
  if (ed > 0.0) _legController->edampCommand(robotType, (float)ed);

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
  // Re-attach aiding: addEstimator copies _data into each estimator, so a
  // rebuild (cheater transition, NaN recovery) would otherwise drop it.
  if (absAiding) _stateEstimator->setAbsoluteAiding(absAiding);
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
  delete _jpos_initializer;
}

void RobotRunner::cleanup() {}
