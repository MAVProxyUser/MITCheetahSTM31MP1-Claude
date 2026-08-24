/*!
 * @file mit_sim_main.cpp
 * @brief STM32MP1 MIT_Controller (convex MPC + WBC) against a Gazebo (Go1) SITL.
 *   mit_ctrl_sim [peer_ip] [robot_yaml] [user_yaml]
 * Runs the full locomotion FSM. With $WP_MISSION set it also closes an
 * OpenPilot-style GPS waypoint loop around it - see navThread() below.
 */
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <thread>
#include <chrono>
#include <unistd.h>   // _exit

#include "Stm32mp1HardwareBridge.h"
#include "MIT_Controller.hpp"
#include "WaypointNav.hpp"
#include "Planning/BodyPathPlanner.h"
#include "Planning/MissionAnalyzer.h"
#include "rt/rt_gazebo.h"      // gazebo_get_aux(): GPS, the same data the real dog gets over CAN

//! Defined in FSM_State_StandUp.cpp - lets the mission lower the stance target
//! so re-entering STAND_UP performs a controlled lie-down.
void setStandUpHeight(double h);
//! Defined in RobotRunner.cpp - Unitree-style damping hold (edampCommand).
void setEdamp(double d);
//! Defined in RobotRunner.cpp - gates the fall detector's z-collapse branch.
//! A COMMANDED lie-down is a level body descending through the detector's
//! 0.10 m threshold - indistinguishable from the collapse it exists to
//! catch - so the mission suspends the z branch around its own lie-downs
//! (the attitude branch stays armed; a lie-down is level by definition).
void setFallZEnable(bool on);
//! Defined in RobotRunner.cpp - gates MIT's ZERO-debounce 28.6 deg
//! orientation ESTOP (ControlFSM::safetyPreCheck). During a commanded stop
//! the settle/crouch wobble can cross 28.6 deg for a tick; the ESTOP then
//! cuts the motors mid-maneuver and CAUSES the fall (and the stop sequence,
//! unaware the FSM was yanked to PASSIVE under it, drives a dead robot).
//! Gated off from the stop command until standing-and-driving again; the
//! debounced fall detector (50 deg / 0.5 s) stays the arbiter inside the
//! window, so a genuine tip still ends the run.
void setOrientTripEnable(bool on);
//! Situational stance-height bias into the controller's height governor, m.
void setHeightBias(double b);

static std::string g_peer;

/*!
 * Waypoint navigation for the MIT stack.
 *
 * The crawl (StaticGaitController) drives nav from inside its own controller
 * because it owns its gait. The MIT stack must not be reached into like that -
 * ConvexMPCLocomotion already consumes a velocity and a yaw rate through
 * DesiredStateCommand, which is exactly the interface an operator's stick uses.
 * So nav writes the SAME two channels the sequencer and a gamepad write:
 *   leftStickAnalog[1]  -> forward velocity  (_x_vel_des)
 *   rightStickAnalog[0] -> yaw rate          (_yaw_turn_rate)
 * Nothing in the FSM, the MPC or the WBC has to know navigation exists.
 *
 * Takeover: the bridge sequencer already stands the robot, enters LOCOMOTION,
 * lets the gait engage and ramps velocity to $SIM_VX. Nav waits for that ramp
 * to finish (it watches the stick the sequencer is driving) and then owns both
 * channels. Because the star mission's first waypoint lies due north and the
 * dog spawns facing north, the handover happens mid-stride on the correct
 * heading - no pivot in place.
 */
static void navThread(Stm32mp1HardwareBridge* bridge) {
  const char* mission = getenv("WP_MISSION");
  if (!mission) return;

  const float vx = getenv("SIM_VX") ? atof(getenv("SIM_VX")) : 0.6f;
  WaypointNav nav;
  float r = 3.f, d = 5.f; int pts = 5;
  if (sscanf(mission, "star:%f:%d", &r, &pts) >= 1)        nav.makeStar(r, pts, vx);
  else if (sscanf(mission, "oval:%f:%f", &d, &r) >= 1) {
    // "oval:<straight m>:<radius m>" - see makeOval for why this course exists.
    if (sscanf(mission, "oval:%f:%f", &d, &r) < 2) r = 3.0f;
    nav.makeOval(d, r,
                 getenv("WP_OVAL_DS") ? atof(getenv("WP_OVAL_DS")) : 1.2f, vx);
  }
  else if (sscanf(mission, "atom:%f:%d", &r, &pts) >= 1) {
    // "atom:<outer radius m>[:<lobes>]" - depth and waypoint spacing are
    // secondary knobs, so the mission string keeps only the two that change
    // the character of the course.
    if (sscanf(mission, "atom:%f:%d", &r, &pts) < 2) pts = 6;   // 6 = the logo
    nav.makeAtom(r, pts,
                 getenv("WP_ATOM_DEPTH") ? atof(getenv("WP_ATOM_DEPTH")) : 0.8f,
                 getenv("WP_ATOM_DS")    ? atof(getenv("WP_ATOM_DS"))    : 1.2f, vx);
  }
  else if (sscanf(mission, "circle:%f:%d", &r, &pts) >= 1) nav.makeCircle(r, pts, vx);
  else if (sscanf(mission, "outback:%f", &d) == 1)         nav.makeOutAndBack(d, vx);
  else                                                     nav.makeStar(5.3f, 5, vx);

  // $WP_DASH=<metres>: append a straight finishing sprint after the loop
  // above closes, continuing along whatever heading the final leg left the
  // dog on. This is the 100 m dash as a FINISH, not a standalone course - the
  // dog proves it can corner the whole loop, then gets to show what its gait
  // does in a straight line once there is nothing left to turn for.
  //
  // Per direct instruction, the handoff between loop and dash is not just a
  // waypoint change: the dog stops, lies down, and stands back up before the
  // sprint - the same lie-down/stand-up the QA ladder already rehearses at
  // the true end of every mission (see the END-OF-MISSION SEQUENCE below),
  // just run once more in the middle instead of only at the finish.
  // The dash target is always the LAST waypoint appendDash produced -
  // one point on an already-closed course (star/oval/atom all close their
  // own stroke now), two (return-to-wp00, then the sprint) on an open one.
  // The interlude fires once nav advances ONTO the dash point, i.e. the
  // dog has genuinely closed the shape before it stops and lies down.
  const int loop_wp_count = nav.count();
  bool dash_pending = false;
  if (getenv("WP_DASH")) {
    nav.appendDash(atof(getenv("WP_DASH")), vx);
    dash_pending = nav.count() > loop_wp_count;
  }
  const int dash_wp_index = nav.count() - 1;

  if (getenv("WP_ACCEPT"))   nav.accept_radius = atof(getenv("WP_ACCEPT"));
  if (getenv("WP_LOOP"))     nav.loop = true;
  // The MPC trot turns far harder than the crawl, which is what max_yawrate was
  // sized for (0.7 rad/s was "what the crawl can actually deliver").
  nav.max_yawrate = getenv("WP_MAX_YAWRATE") ? atof(getenv("WP_MAX_YAWRATE")) : 1.2f;
  nav.kp_heading  = getenv("WP_KP_HEADING")  ? atof(getenv("WP_KP_HEADING"))  : 2.2f;
  // Dynamic gaits arc through corners; they fall over trying to pivot in them.
  nav.turn_speed_floor = getenv("WP_TURN_FLOOR") ? atof(getenv("WP_TURN_FLOOR")) : 0.65f;

  // YAW SIGN: +1, and this was MEASURED, not reasoned. The old default of -1
  // captured 1 of 5 waypoints and never finished a mission; +1 captures 5 of 5
  // and completes the 100 m star in 46.5 s. Same binary, same run, one flag.
  //
  // Why -1 looked right and was wrong: the comment here used to say "MIT's yaw
  // rate is CCW-positive, nav's is compass-sense, so negate". Both halves are
  // true, but the chain has a THIRD negation in it -
  // `DesiredStateCommand.cpp` does `joystickRight[0] *= -1` (upstream MIT, and
  // correct: right stick right should turn the robot right). Two negations
  // cancel, so negating here inverted the steering and the robot turned away
  // from every waypoint. A sign argued from two of three terms is a guess.
  const float yaw_sign = getenv("WP_YAW_SIGN") ? atof(getenv("WP_YAW_SIGN")) : 1.f;

  /*
   * $WP_PLANNER=1 swaps the pure-pursuit follower for the Apollo-derived
   * planner: the waypoint polyline is rounded into fillet arcs, curvature gives
   * v_max(s) = sqrt(a_lat/kappa), and forward/backward accel passes make the
   * braking start BEFORE the corner instead of during it. The heuristic it
   * replaces (WP_TURN_FLOOR) completes 2 runs in 3 and fails badly if tuned any
   * deeper, which is what a hand-picked constant tends to do.
   */
  const bool use_planner = getenv("WP_PLANNER") && atoi(getenv("WP_PLANNER")) != 0;
  const bool  use_analyzer = getenv("WP_ANALYZER") && atoi(getenv("WP_ANALYZER")) != 0;
  const float analyzer_lead = getenv("WP_ANALYZER_LEAD") ? atof(getenv("WP_ANALYZER_LEAD")) : 4.0f;

  planning::BodyPathPlanner planner;
  planning::MissionAnalyzer analyzer;
  // Stop coordinates, kept for the nav loop's "no gait changes inside a
  // stop's braking zone" hold (see the gait-change gate below). [0] = the
  // dash-interlude closure (valid while dash_pending), [1] = the final
  // waypoint (always a stop - the end-of-mission lie-down).
  float hold_stop_n[2] = {0, 0}, hold_stop_e[2] = {0, 0};
  if (use_planner) {
    // The path must START WHERE THE ROBOT IS. Building it from the waypoints
    // alone begins it at wp00, 10.5 m away, so the follower spent the whole
    // mission chasing a path it was never on: 75.1 m of path (4 legs) instead
    // of 93 m (5), and 0 of 5 waypoints. The robot sits at the local-frame
    // origin when the GPS datum is taken, so (0,0) is the true first point.
    std::vector<double> wx{0.0}, wy{0.0};
    for (int i = 0; i < nav.count(); ++i) {
      wx.push_back(nav.waypoint(i).north);
      wy.push_back(nav.waypoint(i).east);
    }
    planning::BodyLimits lim;
    lim.v_cruise  = vx;
    lim.a_lat_max = getenv("WP_ALAT") ? atof(getenv("WP_ALAT")) : 2.5;

    lim.yaw_rate_max = nav.max_yawrate;
    lim.track_lag_s  = getenv("WP_LAG") ? atof(getenv("WP_LAG")) : 1.2;
    planner.setLimits(lim);
    if (getenv("WP_ALON")) planner.setAlonExplicit(atof(getenv("WP_ALON")));
    if (getenv("WP_AACC")) { auto L=planner.limits(); L.a_accel_max=atof(getenv("WP_AACC")); planner.setLimits(L); }
    if (getenv("WP_TURN_SOFT")) { auto L=planner.limits(); L.turn_soft=atof(getenv("WP_TURN_SOFT")); planner.setLimits(L); }
    if (getenv("WP_TURN_HARD")) { auto L=planner.limits(); L.turn_hard=atof(getenv("WP_TURN_HARD")); planner.setLimits(L); }
    if (getenv("WP_CSCALE"))    { auto L=planner.limits(); L.corner_scale_min=atof(getenv("WP_CSCALE")); planner.setLimits(L); }
    // Same turn_soft/turn_hard grading, applied to the fillet's geometric
    // corridor instead of the lateral-accel budget - shrinks how far an
    // acute corner's rounded arc misses the true vertex by. See the field
    // comment on BodyLimits::corridor_scale_min.
    if (getenv("WP_CORRIDOR_MIN")) { auto L=planner.limits(); L.corridor_scale_min=atof(getenv("WP_CORRIDOR_MIN")); planner.setLimits(L); }
    if (getenv("WP_HAIRPIN")) { auto L = planner.limits(); L.hairpin_rad = atof(getenv("WP_HAIRPIN")); planner.setLimits(L); }
    if (getenv("WP_VPIVOT"))  { auto L = planner.limits(); L.v_pivot     = atof(getenv("WP_VPIVOT"));  planner.setLimits(L); }
    const double corridor = getenv("WP_ACCEPT") ? atof(getenv("WP_ACCEPT")) : 1.0;
    /*
     * STOPS ARE PART OF THE PLAN (see BodyPathPlanner::addStopXY). The path
     * end always brakes to v_min (every mission finishes with a lie-down);
     * with a dash appended, the loop-closure waypoint - where the dog stops,
     * lies down and stands back up before the sprint - is a mid-path stop,
     * so the profile brakes into it and re-accelerates out of it with the
     * same two-pass math the corners use. Before this, the dog arrived at
     * both at full cruise and the stop sequence was really a crash-stop.
     */
    if (getenv("WP_END_BRAKE") && atoi(getenv("WP_END_BRAKE")) == 0)
      planner.setEndStop(false);
    if (dash_pending) {
      const auto& stopw = nav.waypoint(dash_wp_index - 1);
      planner.addStopXY(stopw.north, stopw.east);
      hold_stop_n[0] = stopw.north; hold_stop_e[0] = stopw.east;
      printf("[plan] mid-path stop registered at the loop closure "
             "(wp%02d N=%.2f E=%.2f) for the dash interlude\n",
             dash_wp_index - 1, stopw.north, stopw.east);
    }
    hold_stop_n[1] = nav.waypoint(nav.count() - 1).north;
    hold_stop_e[1] = nav.waypoint(nav.count() - 1).east;
    planner.plan(wx, wy, 0.10, false, corridor);
    /*
     * ANALYSE THE MISSION ONCE, HERE, BEFORE THE DOG MOVES.
     *
     * Everything about a route that can be known is known now: the geometry is
     * fixed, so the curvature, the speed each point allows, which turns last
     * long enough to need a different gait, and where the time is actually
     * lost are all computable before the first step. Re-deriving any of it at
     * 50 Hz from a filtered estimate is strictly worse, and has already
     * produced a decider that could not tell a star vertex from an atom lobe.
     */
    planning::MissionPolicy pol;
    pol.kappa_min     = getenv("WP_REGIME_K")   ? atof(getenv("WP_REGIME_K"))   : pol.kappa_min;
    pol.sustained_m   = getenv("WP_REGIME_LEN") ? atof(getenv("WP_REGIME_LEN")) : pol.sustained_m;
    pol.min_switch_m  = getenv("WP_MIN_SWITCH") ? atof(getenv("WP_MIN_SWITCH")) : pol.min_switch_m;
    pol.gait_fast     = getenv("WP_GAIT_FAST")   ? atoi(getenv("WP_GAIT_FAST"))   : pol.gait_fast;
    pol.gait_sustained= getenv("WP_GAIT_CORNER") ? atoi(getenv("WP_GAIT_CORNER")) : pol.gait_sustained;
    pol.hbias_max     = getenv("WP_HBIAS")      ? atof(getenv("WP_HBIAS"))      : pol.hbias_max;
    pol.v_sustained_max = getenv("WP_VSUS") ? atof(getenv("WP_VSUS")) : pol.v_sustained_max;
    // ALWAYS analyse (the brief is free and worth having in every log), but
    // only ACT on it when asked. applyTo() imposes the sustained-curve speed
    // ceiling, which is a behaviour change - running it unconditionally made
    // the "bare" arm of an A/B not bare, and would have silently treated the
    // control group on every atom phase where sustained segments dominate.
    analyzer.analyze(planner, pol);
    if (use_analyzer) {
      // Two-pass: classify, impose the ceiling, re-plan, re-classify so the
      // printed brief is the plan the robot actually flies.
      analyzer.applyTo(planner, pol);
      analyzer.analyze(planner, pol);
    }
    analyzer.print();

    double kk, vv; planner.tightestCorner(&kk, &vv);
    printf("[plan] %zu pts, %.1f m, tightest R=%.2f m -> %.2f m/s "
           "(cruise %.2f, a_lat %.2f, corridor %.2f)\n",
           planner.path().size(), planner.path().empty() ? 0.0 : planner.path().back().s,
           kk > 1e-6 ? 1.0/kk : 0.0, vv, lim.v_cruise, lim.a_lat_max, corridor);
    fflush(stdout);
  }

  /*
   * TAKE THE STICK OURSELVES, from LOCOMOTION entry - not by waiting for the
   * bridge's own straight-line ramp to finish, which is what used to run the
   * dog dead ahead for the whole delay+ramp window before nav ever steered
   * (see the WP_MISSION guard added around that ramp in
   * Stm32mp1HardwareBridge.cpp for the measured symptom and why it is wrong
   * for a mission specifically). Wait for control_mode == 4 (K_LOCOMOTION),
   * then hold the SAME gait-engage settle the old ramp used
   * ($SIM_VX_DELAY_S, still the standing-gait settle time, not a straight-line
   * hold) before calling nav.update() for the first time - the planner's own
   * accel limit (a_lon/a_accel_max) ramps speed from the standstill the robot
   * is actually at, while STEERING from tick one instead of after 15 s of
   * running straight.
   */
  while (bridge->getControlMode() != 4)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  {
    float settle_s = getenv("SIM_VX_DELAY_S") ? atof(getenv("SIM_VX_DELAY_S")) : 3.f;
    printf("[nav] LOCOMOTION engaged - holding %.1f s for the gait to settle "
           "before nav takes the stick\n", settle_s);
    fflush(stdout);
    std::this_thread::sleep_for(std::chrono::milliseconds((long)(settle_s * 1000.f)));
  }

  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed = [&t0]() {
    return std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
  };
  printf("[nav] taking the stick at t=%.1fs (mission %s)\n", elapsed(), mission);
  fflush(stdout);
  /*
   * Anchor for the forward-speed ramp below - NOT armed on the initial
   * takeover (restart_t starts a full ramp window in the past, so vscale is
   * 1.0 immediately). BodyPathPlanner::computeSpeedProfile() now forces
   * _path[0].v = 0 and ramps it forward itself with the SAME accel limit, so
   * applying a second, independent time-based ramp here on top of that
   * double-ramps and desyncs the caller's notion of "how far along the plan
   * am I" from the plan's own arc-length-based one - measured to cost the
   * star ~4 s and a visibly wide, arcing entry into its first corner (see
   * the comment at that _path[0].v = 0 assignment for the full account).
   * The dash interlude's stand-back-up IS a genuine standstill the
   * pre-computed profile does not know about (plan() is never called again
   * mid-mission), so THAT restart still arms this ramp for real.
   */
  const float vx_ramp_s = getenv("SIM_VX_RAMP_S") ? atof(getenv("SIM_VX_RAMP_S")) : 3.f;
  float restart_t = elapsed() - vx_ramp_s;

  /*
   * GAIT DECIDER ($WP_GAIT_DECIDER=1) - Apollo's task of the same name, using
   * the planner's curvature knowledge: run the FAST gait on straights and the
   * agile one through corners.
   *
   * The rule that makes it safe came from a controlled split, not a guess:
   *   switch gait, then accelerate             -> CROSSES (2/2)
   *   switch gait and accelerate together      -> CROSSES (30.3 s at 3.97 m/s)
   *   switch to a SLOWER-capable gait AT SPEED -> FAILS (2/2, ~35 m)
   * Only the last is dangerous. So the decider only ever switches while the
   * robot is ALREADY SLOW - which on a star is exactly the corner, where the
   * planner has braked for curvature anyway. Both directions then land in the
   * regime measured to work: drop to trotting while braking in, restore
   * trotRunning at the apex BEFORE accelerating out.
   */
  const bool gait_decider = getenv("WP_GAIT_DECIDER") && atoi(getenv("WP_GAIT_DECIDER")) != 0;
  /*
   * THREE TIERS, not a pair. There is no single transition rule and no reason
   * to force one: the course asks for different things and the gaits answer
   * differently, so the schedule is graded the same way the corner speed now is.
   *
   *   STRAIGHT  trotRunning (5)  40%% duty, flight phase - fastest, worst turner
   *   MIDDLE    trotting    (9)  50%% duty - the best all-rounder measured, and
   *                              the one to LEAN ON: it alone completes the star
   *                              at 2.0-2.5 m/s, 7/7
   *   TIGHT     walking    (20)  50%% duty 4-beat - slowest, most stable, for
   *                              the corners neither of the others survives
   *
   * Tiers are chosen from the PLANNED speed, which already encodes corner
   * angle through the angle-graded budget - so gait and speed grade together
   * instead of one fighting the other.
   */
  const int  gait_fast   = getenv("WP_GAIT_FAST")   ? atoi(getenv("WP_GAIT_FAST"))   : 5;
  const int  gait_corner = getenv("WP_GAIT_CORNER") ? atoi(getenv("WP_GAIT_CORNER")) : 9;
  const int  gait_tight  = getenv("WP_GAIT_TIGHT")  ? atoi(getenv("WP_GAIT_TIGHT"))  : 0;  // 0 = unused
  /*
   * $WP_GAIT_REGIME=1 switches the decider from "how slow is it ahead" to
   * "how LONG is the turn ahead" - see BodyPathPlanner::regimeAhead. kappa is
   * 1/R, so 0.15 is a 6.7 m radius; regime_len is the run length above which a
   * flight gait is judged unable to hold on.
   */
  /*
   * $WP_ANALYZER=1 hands gait and height-bias decisions to the pre-analysed
   * mission. `analyzer_lead` is how far ahead the robot reads - a gait change
   * needs its ~500 ms settling window BEFORE the corner, not in it.
   */
  const bool  regime_mode  = getenv("WP_GAIT_REGIME") && atoi(getenv("WP_GAIT_REGIME")) != 0;
  const float regime_kappa = getenv("WP_REGIME_K")   ? atof(getenv("WP_REGIME_K"))   : 0.15f;
  const float regime_len   = getenv("WP_REGIME_LEN") ? atof(getenv("WP_REGIME_LEN")) : 3.0f;
  const float v_tight    = getenv("WP_V_TIGHT")     ? atof(getenv("WP_V_TIGHT"))     : 1.1f;
  const float decide_v   = getenv("WP_GAIT_SWITCH_V") ? atof(getenv("WP_GAIT_SWITCH_V")) : 1.6f;
  /*
   * cur_gait MUST start as the gait the robot is ACTUALLY running, which is
   * cmpc_gait from the yaml - not gait_corner.
   *
   * Initialising it to gait_corner made the decider believe it was already in
   * the corner gait, so it never switched INTO it: the robot took every corner
   * in the fast gait and fell in the first one, while the log showed a single
   * no-op "switch" to the gait it was already in. That bug is why
   * trotting-straights/walking-corners looked like a dead end - the pairing was
   * never actually exercised.
   */
  int cur_gait = bridge->userParams()
      ? (int)((MIT_UserParameters*)bridge->userParams())->cmpc_gait
      : gait_corner;
  /*
   * LOOKAHEAD: switch on the curvature AHEAD, not underfoot. An animal changes
   * gait BEFORE the corner, while it still has time; deciding from the speed at
   * the current point means the decision arrives once already committed.
   */
  const float decide_ahead = getenv("WP_GAIT_LOOKAHEAD") ? atof(getenv("WP_GAIT_LOOKAHEAD")) : 4.0f;

  // Height pre-load for corners - see setHeightBias() below. Off by default so
  // the governor's own contribution can be measured before this stacks on it.
  const float hbias_gain  = getenv("WP_HBIAS")       ? atof(getenv("WP_HBIAS"))       : 0.0f;
  const float hbias_ahead = getenv("WP_HBIAS_AHEAD") ? atof(getenv("WP_HBIAS_AHEAD")) : 5.0f;

  const float dt = 0.02f;      // 50 Hz: the gait's own bandwidth is far below this
  float yaw_ref = NAN;
  int   lastIdx = -1;
  bool  done = false;

  while (!done) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (!bridge->robotRunner()) continue;
    const auto& est = bridge->robotRunner()->getStateEstimate();

    SimAuxSensors aux;
    gazebo_get_aux(&aux);
    if (!nav.originSet()) {
      if (aux.gps_lat == 0.0) continue;
      nav.setOrigin(aux.gps_lat, aux.gps_lon);
      yaw_ref = est.rpy[2];    // the dog always starts facing north
      printf("[nav] GPS origin set, heading datum %.1f deg\n", yaw_ref * 57.2958f);
      fflush(stdout);
    }

    float N, E;
    nav.toLocal(aux.gps_lat, aux.gps_lon, &N, &E);
    // Compass bearing (0 = north, positive toward east) relative to the datum.
    // Taking it relative to the start makes this work identically whether the
    // estimator zeroes its yaw (VectorNav path) or reports true world yaw
    // (cheater path) - the two differ by exactly this constant.
    const float bearing = -(est.rpy[2] - yaw_ref);
    const float spd = sqrtf(est.vBody[0] * est.vBody[0] + est.vBody[1] * est.vBody[1]);

    float nv = 0.f, nw = 0.f;
    // NAV owns mission state (arrival tests, waypoint advance, completion); the
    // planner only supplies the COMMANDS. Letting the planner's end-of-path
    // decide completion would end the mission wherever the smoothed path ran
    // out, which is not where the waypoints are.
    bool running = nav.update(N, E, bearing, spd, dt, &nv, &nw);
    if (use_planner) {
      double pv = 0, pw = 0;
      if (planner.follow(N, E, bearing, &pv, &pw)) { nv = (float)pv; nw = (float)pw; }
    }

    /*
     * PRE-LOAD STANCE HEIGHT FOR THE CORNER AHEAD. The controller's height
     * governor regulates height reactively off the robot's own state; this is
     * the planner telling it, ahead of time, that the ground is about to ask
     * for more. $WP_HBIAS is metres of extra height at full lateral budget
     * (0 = planner stays out of it and the governor runs purely reactive).
     */
    /*
     * READ THE ANNOTATED MISSION. A lookup, not a computation: the segment
     * `lead` metres ahead already carries the gait to be in and the stance
     * height to have pre-loaded by the time the demand arrives.
     */
    const planning::MissionSegment* seg = nullptr;
    if (use_analyzer && use_planner)
      seg = analyzer.segmentAhead(planner.currentS(), analyzer_lead);
    if (seg) setHeightBias(seg->height_bias);
    else if (use_planner && hbias_gain > 0.f)
      setHeightBias(planner.plannedHeightBias(hbias_ahead, hbias_gain));

    /*
     * NO GAIT CHANGES INSIDE A STOP'S BRAKING ZONE. Measured on the oval:
     * the analyzer upgrades trotting -> trotRunning entering the closing
     * straight ~3 s before the loop-closure STOP, so the stop maneuver
     * begins from a barely-engaged flight-phase gait - and that stop was
     * the one physically tipping sideways (~1 in 3, roll 72-88 deg) while
     * the star's and atom's stops (no switch on approach) passed cleanly.
     * Hold any pending change while within R_HOLD of a stop still ahead;
     * once the stop is consumed (interlude done -> dash_pending false and
     * the dog drives away), the held segment gait applies on the next
     * pass - which is exactly how the dash still gets trotRunning, just
     * AFTER standing back up instead of right before lying down.
     */
    bool near_stop = false;
    {
      // Distance alone is ambiguous on a CLOSED course - the dog spawns
      // next to the closure coordinates, and a bare radius test would
      // hold the analyzer's very first upgrade at mission start. Require
      // the mission to be past halfway before a stop can be "ahead".
      const float R_HOLD = 10.f;
      const bool second_half = lastIdx > nav.count() / 2;
      if (dash_pending && second_half &&
          std::hypot(N - hold_stop_n[0], E - hold_stop_e[0]) < R_HOLD)
        near_stop = true;
      if (second_half &&
          std::hypot(N - hold_stop_n[1], E - hold_stop_e[1]) < R_HOLD)
        near_stop = true;
    }

    if (seg && bridge->userParams() && seg->gait != cur_gait && !near_stop) {
      ControlParameterValue cv; cv.d = (double)seg->gait;
      bridge->userParams()->collection.lookup("cmpc_gait")
          .set(cv, ControlParameterValueKind::DOUBLE);
      printf("[mission] gait %d -> %d entering %s at s=%.1f (R=%.2f, cost %+.2f s) t=%.1fs\n",
             cur_gait, seg->gait,
             seg->regime == 2 ? "SUSTAINED" : (seg->regime == 1 ? "transient" : "straight"),
             seg->s0, seg->radius_min, seg->time_cost, elapsed());
      fflush(stdout);
      cur_gait = seg->gait;
    }

    if (gait_decider && !use_analyzer && use_planner && bridge->userParams()) {
      // Slowest planned speed within decide_ahead metres: if a corner is
      // coming, commit to the corner gait NOW, while still able to.
      const double vplan = planner.minPlannedSpeedAhead(decide_ahead);
      int want;
      if (regime_mode) {
        /*
         * DECIDE ON THE DURATION OF THE TURN, NOT ITS SEVERITY.
         *
         * The old rule below keys off planned SPEED, which reads a star vertex
         * and an atom lobe identically - and they want opposite gaits.
         * trotRunning is 32/32 on the star (whose corners are over in a metre)
         * and 3/8 on the atom (whose curvature is held for ten-plus metres).
         * regimeAhead() separates them by how long the turn lasts.
         */
        const int reg = planner.regimeAhead(decide_ahead, regime_kappa, regime_len);
        want = (reg == 2) ? gait_corner : gait_fast;
      }
      else if (vplan >= 2.2)                      want = gait_fast;
      else if (gait_tight > 0 && vplan < v_tight) want = gait_tight;
      else                                        want = gait_corner;
      /*
       * THE GATE IS ASYMMETRIC, because the risk is.
       *
       *   -> trotRunning (flight gait) at 2.63 m/s : FATAL (fell instantly)
       *   -> trotRunning at 1.2 m/s                : fine, 2/2
       *   -> walking  at 2.0 m/s                   : fine, 2/2
       *   -> trotting at 2.0 m/s                   : fine, 1/1
       *
       * Dropping to a MORE STABLE gait is safe at speed; climbing to a flight
       * gait is not. A symmetric gate blocked the drop until the robot had
       * already braked to 1.6 m/s - which on this course is the corner apex,
       * far too late - so it entered every corner still in the fast gait and
       * fell. Going UP still waits for low speed; going DOWN may happen
       * whenever the target gait can hold the current speed.
       */
      // Dropping to the stable gait is UNGATED. The only measured danger is
      // climbing to a flight gait at speed; nothing in the data says a
      // down-switch is unsafe (walking at 2.0: 2/2, trotting at 2.0: 1/1).
      // Gating the drop by the target gait's COMMAND ceiling was wrong twice
      // over: trotRunning at 3.0 commanded actually cruises at 3.6 m/s, so a
      // 3.1 gate blocked the drop entirely and the robot entered every corner
      // still in the flight gait - the exact failure the decider exists to
      // prevent.
      // "Down" means toward the more stable end of the ladder
      // (trotRunning -> trotting -> walking); only climbing toward a flight
      // gait is speed-gated, because that is the only direction measured to
      // fail (fatal at 2.63 m/s, fine at 1.2).
      auto tierOf = [&](int g){ return g == gait_fast ? 2 : (g == gait_tight ? 0 : 1); };
      const bool going_down = tierOf(want) < tierOf(cur_gait);
      const bool may_switch = going_down ? true : (spd < decide_v);
      { static int gdbg = 0;
        if ((++gdbg % 50) == 0)
          printf("[gaitdbg] vplan=%.2f want=%d cur=%d spd=%.2f down=%d may=%d\n",
                 vplan, want, cur_gait, spd, (int)going_down, (int)may_switch),
          fflush(stdout); }
      if (want != cur_gait && may_switch) {
        ControlParameterValue cv; cv.d = (double)want;
        bridge->userParams()->collection.lookup("cmpc_gait")
            .set(cv, ControlParameterValueKind::DOUBLE);
        printf("[gait] %d -> %d at v=%.2f (planned %.2f) t=%.1fs\n",
               cur_gait, want, spd, vplan, elapsed());
        fflush(stdout);
        cur_gait = want;
      }
    }

    if (nav.activeIndex() != lastIdx) {
      lastIdx = nav.activeIndex();
      printf("[nav] -> wp%02d of %d  t=%.1fs\n", lastIdx, nav.count(), elapsed());
      fflush(stdout);
    }

    /*
     * LOOP-TO-DASH INTERLUDE. The loop closed AND the explicit return to
     * wp00 (appendDash()'s first inserted point) is done - nav has just
     * advanced onto the actual dash waypoint - so stop, settle, lie down,
     * then stand back up before sprinting the dash, exactly mirroring the
     * end-of-mission sequence below rather than inventing a second one.
     * This blocks the thread for its duration (a few seconds), which is
     * fine: nothing else needs the stick during a controlled sit-and-stand,
     * and the alternative (driving it from the 50 Hz loop with a state
     * machine) buys nothing but complexity for a one-shot event.
     */
    if (dash_pending && lastIdx == dash_wp_index) {
      dash_pending = false;
      printf("[nav] loop complete at t=%.1fs - stop, lie down, stand back up "
             "before the %s dash\n", elapsed(), mission);
      fflush(stdout);

      // STOP WINDOW OPENS: forward command is being taken to zero on
      // purpose. Suspend the zero-debounce orientation ESTOP for the whole
      // stop/lie-down/stand-up maneuver (re-armed below once driving
      // again) - one transient 28.6 deg tick during the settle otherwise
      // cuts the motors mid-crouch and causes the very fall it polices.
      // The debounced fall detector stays armed as the window's arbiter.
      setOrientTripEnable(false);

      // 1. decelerate from cruise - a stepped-to-zero stick pitches the
      //    robot forward on the way down, same reasoning as end-of-mission.
      //
      //    WRONG TWICE before this, in opposite directions:
      //    - a short (1.5s, fixed) ramp was blamed for "pitching the robot
      //      forward" - but that test also had the (since-fixed) illegal
      //      BALANCE_STAND->STAND_UP transition confounding it, so the real
      //      cause was never isolated;
      //    - lengthening it to hold a gentler ~1.2 m/s^2 (this body's
      //      CORNERING deceleration rate, borrowed from a different
      //      manoeuvre than "come to a dead stop") made it WORSE: the stick
      //      still commands nonzero forward speed, UNSTEERED (yaw zeroed),
      //      for the entire ramp - at 3.5 m/s over a ~2.9 s ramp that is
      //      ~5 m of straight-line coasting PAST wp00 before it even starts
      //      meaningfully slowing, which is the overshoot actually observed
      //      live, not a planner or waypoint-arrival bug.
      //    Short and sharp instead - the goal at a stop is arriving close to
      //    the point, not a gentle deceleration profile - and BALANCE_STAND
      //    is no longer entered on a timer's say-so: the loop below polls
      //    the REAL measured body speed and only proceeds once it is
      //    actually low, with a bounded timeout as a backstop.
      // Ramp from the speed actually being COMMANDED right now, not from
      // cruise: the planner's mid-path stop (addStopXY at the loop closure)
      // has already braked the dog to ~v_min by the time this fires, so the
      // stick is at a creep - ramping "vx down to zero" from here would
      // first SPIKE the command back up to cruise.
      const float v_at_stop = bridge->driverCommand().leftStickAnalog[1];
      for (int k = 15; k >= 0; --k) {
        bridge->driverCommand().leftStickAnalog[1]  = v_at_stop * (float)k / 15.f;
        bridge->driverCommand().rightStickAnalog[0] = 0.f;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      bridge->driverCommand().leftStickAnalog[1]  = 0.f;
      bridge->driverCommand().rightStickAnalog[0] = 0.f;
      {
        const float settle_start = elapsed();
        while (elapsed() - settle_start < 2.0f) {
          if (bridge->robotRunner()) {
            const auto& es = bridge->robotRunner()->getStateEstimate();
            const float spd = std::sqrt(es.vBody[0] * es.vBody[0] +
                                         es.vBody[1] * es.vBody[1]);
            if (spd < 0.15f) break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }

      // 2. settle on its feet. (A 1.3 s trot-in-place settle was inserted
      //    here and REVERTED same-day: with the long zero-vel hold it took
      //    the oval's stop from ~1-in-3 tips to 7-of-8 - see the measured
      //    note at ConvexMPCLocomotion::zeroVelHold.)
      bridge->setControlMode(3);                 // K_BALANCE_STAND
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));

      // 3. lie down - re-enter STAND_UP with a low target (see the
      //    end-of-mission comment for why this reuses that interpolation).
      //    BALANCE_STAND -> STAND_UP is not a legal FSM transition
      //    (FSM_State_BalanceStand::checkTransition()'s switch has no
      //    K_STAND_UP case - every request fell to `default`, printing
      //    "Bad Request: Cannot transition from 3 to 1" and being silently
      //    ignored, so the dog just stood there under BALANCE_STAND's own
      //    control indefinitely - measured on a 3-dog fleet run: over a
      //    hundred rejected requests, then an eventual orientation-safety
      //    fall from prolonged undriven standing, never an actual lie-down).
      //    K_STAND_UP IS legal from K_PASSIVE
      //    (FSM_State_Passive::checkTransition() has that case), which is
      //    also the FSM's OWN normal bootstrap path (PASSIVE -> STAND_UP ->
      //    BALANCE_STAND -> LOCOMOTION), so hop through PASSIVE first.
      // PASSIVE cuts leg torque entirely at the FSM level, but
      // RobotRunner::finalizeStep() applies edampCommand() AFTER the FSM
      // state's own control regardless of which state is active - so
      // damping through this hop is not fighting PASSIVE, it is what
      // actually reaches the legs while PASSIVE would otherwise leave them
      // at true zero. Without this, a still-tall standing robot free-falls
      // for however long PASSIVE holds - measured as a real collapse
      // (roll/pitch/z all dropping) even at a bare 300 ms.
      // The z-collapse fall test cannot tell this COMMANDED crouch from a
      // real collapse (both are a level body under 0.10 m) - it killed the
      // process mid-lie-down on every dash run of 2026-08-24, which also
      // read as "the dog never stands back up". Suspend just the z branch
      // for the crouch; attitude tripping stays armed.
      setFallZEnable(false);
      setEdamp(8.0);
      bridge->setControlMode(0);                 // K_PASSIVE
      // As BRIEF as possible regardless - the FSM only needs ONE control
      // tick (2 ms) to register PASSIVE as current before it will accept
      // the next request, so this only has to outlast scheduling jitter.
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      // Hand off to STAND_UP's own controlled Cartesian interpolation
      // undamped - edamp fighting an active position controller defeats
      // the point of using that interpolation at all.
      setEdamp(0.0);
      setStandUpHeight(0.15);  // 0.07 measured to fall during the interpolation itself (roll/pitch stayed tiny - not a rollover - z sagged fast); trying a less extreme crouch that still counts as "lying down"
      bridge->setControlMode(1);                 // K_STAND_UP
      std::this_thread::sleep_for(std::chrono::milliseconds(2500));
      setEdamp(8.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1200));

      // 4. stand back up - restore the normal stand-up target (0.25, from
      //    FSM_State_StandUp.cpp's g_standUpHeight default) and go back
      //    through the SAME staged entry used at mission start: STAND_UP ->
      //    BALANCE_STAND -> LOCOMOTION. Skipping BALANCE_STAND here would
      //    hand the MPC the same 9 cm height step CLAUDE.md already measured
      //    as a launch (z 0.211 -> 0.342 at 0.32 m/s vertical).
      setEdamp(0.0);
      setStandUpHeight(0.25);
      bridge->setControlMode(1);                 // K_STAND_UP
      std::this_thread::sleep_for(std::chrono::milliseconds(2500));
      bridge->setControlMode(3);                 // K_BALANCE_STAND
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      bridge->setControlMode(4);                 // K_LOCOMOTION
      // Let the gait engage before asking it to go anywhere - the same
      // settle window the initial sequencer holds at mission start.
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      // Standing tall and driving again - stop window closes: re-arm both
      // the z-collapse test and the orientation ESTOP for the dash.
      setFallZEnable(true);
      setOrientTripEnable(true);

      restart_t = elapsed();   // ramp forward speed again from this standstill
      printf("[nav] back up at t=%.1fs - dashing the final leg\n", elapsed());
      fflush(stdout);
    }

    if (!running) {
      printf("[nav] MISSION COMPLETE t=%.1fs  (%d waypoints)\n", elapsed(), nav.count());
      fflush(stdout);
      bridge->driverCommand().leftStickAnalog[1]  = 0.f;
      bridge->driverCommand().rightStickAnalog[0] = 0.f;
      done = true;
      // STOP WINDOW: same suspension as the interlude's (see that comment).
      // Never re-armed - the mission ends inside this window, and the
      // debounced fall detector plus the judge's own attitude criteria
      // cover it to the end.
      setOrientTripEnable(false);
      /*
       * END-OF-MISSION SEQUENCE, and the PASS/FAIL criterion.
       *
       * Reaching the last waypoint is not a completed mission - a robot that
       * arrives and then falls over has not done the job. The mission now ends
       * the way it began, in reverse: decelerate, settle on its feet, then LIE
       * DOWN under control. PASS requires all of it; anything else is FAIL.
       *
       * This also exercises exactly the transitions the hardware QA ladder
       * starts with (stand -> lie down -> stand up -> slow walk), so a mission
       * that passes here has rehearsed the sequence a real dog has to survive.
       */
      // 1. decelerate. Short and sharp, then VERIFY the real measured speed
      //    is actually low before touching the FSM - see the matching
      //    comment and the same fix in the loop-to-dash interlude above,
      //    where a longer "gentler" ramp turned out to just coast the dog
      //    several metres past the stopping point, unsteered, before
      //    slowing down at all.
      for (int k = 15; k >= 0; --k) {
        bridge->driverCommand().leftStickAnalog[1] = nv * (float)k / 15.f;
        bridge->driverCommand().rightStickAnalog[0] = 0.f;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      bridge->driverCommand().leftStickAnalog[1] = 0.f;
      bridge->driverCommand().rightStickAnalog[0] = 0.f;
      {
        const float settle_start = elapsed();
        while (elapsed() - settle_start < 2.0f) {
          if (bridge->robotRunner()) {
            const auto& es = bridge->robotRunner()->getStateEstimate();
            const float spd = std::sqrt(es.vBody[0] * es.vBody[0] +
                                         es.vBody[1] * es.vBody[1]);
            if (spd < 0.15f) break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }

      // 2. settle on its feet (trot-in-place settle tried and reverted -
      //    see the interlude's matching note).
      bridge->setControlMode(3);                 // K_BALANCE_STAND
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      const auto& s1 = bridge->robotRunner()->getStateEstimate();
      const float stand_z = s1.position[2];
      const float stand_roll  = std::fabs(s1.rpy[0]) * 57.2958f;
      const float stand_pitch = std::fabs(s1.rpy[1]) * 57.2958f;

      // 3. LIE DOWN: re-enter STAND_UP with a low target so the same Cartesian
      //    interpolation that stands the robot up lowers it instead.
      //    BALANCE_STAND -> STAND_UP is not a legal FSM transition (see the
      //    matching comment in the loop-to-dash interlude above, where this
      //    was actually caught: FSM_State_BalanceStand::checkTransition()
      //    has no K_STAND_UP case, so this request was silently rejected
      //    every tick and the "lie down" never happened - hop through
      //    K_PASSIVE first, which IS a legal source for K_STAND_UP and is
      //    the FSM's own normal bootstrap path.
      // PASSIVE cuts leg torque entirely at the FSM level, but
      // RobotRunner::finalizeStep() applies edampCommand() AFTER the FSM
      // state's own control regardless of which state is active - so
      // damping through this hop is not fighting PASSIVE, it is what
      // actually reaches the legs while PASSIVE would otherwise leave them
      // at true zero. Without this, a still-tall standing robot free-falls
      // for however long PASSIVE holds - measured as a real collapse
      // (roll/pitch/z all dropping) even at a bare 300 ms.
      // Suspend the z-collapse fall test for this COMMANDED lie-down too -
      // same reason as the interlude's (see that comment): the detector was
      // killing the process mid-crouch, before the judge lines ever printed.
      setFallZEnable(false);
      setEdamp(8.0);
      bridge->setControlMode(0);                 // K_PASSIVE
      // As BRIEF as possible regardless - the FSM only needs ONE control
      // tick (2 ms) to register PASSIVE as current before it will accept
      // the next request, so this only has to outlast scheduling jitter.
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      // Hand off to STAND_UP's own controlled Cartesian interpolation
      // undamped - edamp fighting an active position controller defeats
      // the point of using that interpolation at all.
      setEdamp(0.0);
      setStandUpHeight(0.15);  // 0.07 measured to fall during the interpolation itself (roll/pitch stayed tiny - not a rollover - z sagged fast); trying a less extreme crouch that still counts as "lying down"
      bridge->setControlMode(1);                 // K_STAND_UP
      std::this_thread::sleep_for(std::chrono::milliseconds(2500));
      // 3b. DAMPING HOLD - Unitree's second phase. Their sequence is
      //     StandDown (interpolate the joints down) then edampCommand, so the
      //     robot settles compliant rather than holding a pose stiffly or going
      //     limp. Ported from LegController<T>::edampCommand at 0x1af2c0.
      setEdamp(8.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1200));
      const auto& s2 = bridge->robotRunner()->getStateEstimate();
      const float down_z = s2.position[2];
      const float down_roll  = std::fabs(s2.rpy[0]) * 57.2958f;
      const float down_pitch = std::fabs(s2.rpy[1]) * 57.2958f;

      // 4. judge. Upright while standing, actually lower afterwards, and still
      //    level on the ground - a topple registers as attitude, not height.
      const bool ok_stand = (stand_z > 0.20f && stand_roll < 15.f && stand_pitch < 15.f);
      const bool ok_down  = (down_z < stand_z - 0.06f && down_roll < 20.f && down_pitch < 20.f);
      printf("[mission] settle: z=%.3f roll=%.1f pitch=%.1f -> %s\n",
             stand_z, stand_roll, stand_pitch, ok_stand ? "ok" : "BAD");
      printf("[mission] laydown: z=%.3f roll=%.1f pitch=%.1f -> %s\n",
             down_z, down_roll, down_pitch, ok_down ? "ok" : "BAD");
      printf("[mission] RESULT: %s  (waypoints %d/%d, settle %s, laydown %s)\n",
             (ok_stand && ok_down) ? "PASS" : "FAIL",
             nav.count(), nav.count(), ok_stand ? "ok" : "bad", ok_down ? "ok" : "bad");
      fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      /*
       * _exit, NOT exit - THIS WAS THE "Abort trap: 6".
       *
       * exit() runs static destructors and atexit handlers, while the 500 Hz
       * control loop, the motor task and the MPC worker are all still running.
       * Those threads then touch destroyed objects and the runtime aborts:
       * "libc++abi: terminating". It only ever fired AFTER a mission completed,
       * because this is the only exit() in the program - which is exactly the
       * pattern observed (three sweeps, always on a run that had already
       * printed MISSION COMPLETE, never on a failure).
       *
       * RobotRunner's fall detector already uses _exit() for the same reason.
       * On hardware this matters far more than in sim: tearing down the process
       * while motor threads are mid-command is not something to leave to luck.
       */
      _exit(0);
    }

    /*
     * FORWARD-SPEED RAMP, STEERING AT FULL AUTHORITY. A stepped velocity
     * command from standstill is the exact hazard the old bridge-side ramp
     * existed to avoid ("knocked the trot over"), so that protection is kept
     * here - but only on the SPEED channel. Yaw rate is nav's steering
     * output, recomputed fresh every tick from the actual heading error, not
     * a fixed step; letting it act at full strength from tick one is what
     * fixes the straight-line overshoot, and it is already measured safe at
     * far higher authority than this ever asks for (3.0 rad/s stationary
     * spin, roll < 1.5 deg, nothing falls - see the yaw envelope results).
     */
    const float vscale = std::min(1.f, (elapsed() - restart_t) / std::max(0.1f, vx_ramp_s));
    bridge->driverCommand().leftStickAnalog[1]  = nv * vscale;
    bridge->driverCommand().rightStickAnalog[0] = yaw_sign * nw;

    static int navlog = 0;
    if ((++navlog % 50) == 0) {   // 1 Hz
      printf("[nav] wp%d/%d N=%.2f E=%.2f hdg=%.0f d=%.2f v=%.2f w=%.2f t=%.1fs\n",
             nav.activeIndex(), nav.count(), N, E, bearing * 57.2958f,
             nav.lastDistance(), nv, nw, elapsed());
      fflush(stdout);
    }
  }
}

int main(int argc, char** argv) {
  // Line-buffer stdout. The sweep harness ends runs with SIGTERM (timeout, or
  // the fall detector), and a block-buffered stdout loses everything still in
  // the buffer when that lands - which reads as "the run produced no log".
  setvbuf(stdout, nullptr, _IOLBF, 0);

  g_peer                 = (argc > 1) ? argv[1] : "127.0.0.1";
  std::string robot_yaml = (argc > 2) ? argv[2] : "stm32mp1-defaults.yaml";
  std::string user_yaml  = (argc > 3) ? argv[3] : "mc-mit-ctrl-user-parameters.yaml";

  printf("[mit-sim] Gazebo SITL | peer=%s robot=%s user=%s\n",
         g_peer.c_str(), robot_yaml.c_str(), user_yaml.c_str());

  MIT_Controller* ctrl = new MIT_Controller();
  Stm32mp1HardwareBridge bridge(ctrl, robot_yaml, user_yaml,
                                Stm32mp1HardwareBridge::Backend::GAZEBO);
  GazeboUdpConfig g;
  g.peer_addr   = g_peer.c_str();
  /*
   * PARALLEL INSTANCES. $SIM_INSTANCE shifts this dog's UDP ports so several
   * can share the Mac: instance 0 keeps 9100/9101, instance 1 gets 9110/9111,
   * and so on. Gazebo transport is isolated separately with $GZ_PARTITION -
   * ports alone are not enough, because two servers on the default partition
   * see each other's topics and the bridges cross-feed.
   */
  const int inst = getenv("SIM_INSTANCE") ? atoi(getenv("SIM_INSTANCE")) : 0;
  g.cmd_port    = 9100 + 10 * inst;
  g.sensor_port = 9101 + 10 * inst;
  bridge.setGazebo(g);

  std::thread(navThread, &bridge).detach();

  bridge.run();
  return 0;
}
