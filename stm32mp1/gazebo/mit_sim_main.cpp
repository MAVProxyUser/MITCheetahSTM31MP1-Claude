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
#include "rt/rt_gazebo.h"      // gazebo_get_aux(): GPS, the same data the real dog gets over CAN

//! Defined in FSM_State_StandUp.cpp - lets the mission lower the stance target
//! so re-entering STAND_UP performs a controlled lie-down.
void setStandUpHeight(double h);
//! Defined in RobotRunner.cpp - Unitree-style damping hold (edampCommand).
void setEdamp(double d);

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
  else if (sscanf(mission, "circle:%f:%d", &r, &pts) >= 1) nav.makeCircle(r, pts, vx);
  else if (sscanf(mission, "outback:%f", &d) == 1)         nav.makeOutAndBack(d, vx);
  else                                                     nav.makeStar(5.3f, 5, vx);

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
  planning::BodyPathPlanner planner;
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
    if (getenv("WP_TURN_SOFT")) { auto L=planner.limits(); L.turn_soft=atof(getenv("WP_TURN_SOFT")); planner.setLimits(L); }
    if (getenv("WP_TURN_HARD")) { auto L=planner.limits(); L.turn_hard=atof(getenv("WP_TURN_HARD")); planner.setLimits(L); }
    if (getenv("WP_CSCALE"))    { auto L=planner.limits(); L.corner_scale_min=atof(getenv("WP_CSCALE")); planner.setLimits(L); }
    if (getenv("WP_HAIRPIN")) { auto L = planner.limits(); L.hairpin_rad = atof(getenv("WP_HAIRPIN")); planner.setLimits(L); }
    if (getenv("WP_VPIVOT"))  { auto L = planner.limits(); L.v_pivot     = atof(getenv("WP_VPIVOT"));  planner.setLimits(L); }
    const double corridor = getenv("WP_ACCEPT") ? atof(getenv("WP_ACCEPT")) : 1.0;
    planner.plan(wx, wy, 0.10, false, corridor);
    double kk, vv; planner.tightestCorner(&kk, &vv);
    printf("[plan] %zu pts, %.1f m, tightest R=%.2f m -> %.2f m/s "
           "(cruise %.2f, a_lat %.2f, corridor %.2f)\n",
           planner.path().size(), planner.path().empty() ? 0.0 : planner.path().back().s,
           kk > 1e-6 ? 1.0/kk : 0.0, vv, lim.v_cruise, lim.a_lat_max, corridor);
    fflush(stdout);
  }

  // Wait for the sequencer to finish its velocity ramp before taking the stick.
  while (bridge->driverCommand().leftStickAnalog[1] < vx * 0.99f)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const auto t0 = std::chrono::steady_clock::now();
  auto elapsed = [&t0]() {
    return std::chrono::duration<float>(std::chrono::steady_clock::now() - t0).count();
  };
  printf("[nav] taking the stick at t=%.1fs (mission %s)\n", elapsed(), mission);
  fflush(stdout);

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

    if (gait_decider && use_planner && bridge->userParams()) {
      // Slowest planned speed within decide_ahead metres: if a corner is
      // coming, commit to the corner gait NOW, while still able to.
      const double vplan = planner.minPlannedSpeedAhead(decide_ahead);
      int want;
      if      (vplan >= 2.2)                      want = gait_fast;
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

    if (!running) {
      printf("[nav] MISSION COMPLETE t=%.1fs  (%d waypoints)\n", elapsed(), nav.count());
      fflush(stdout);
      bridge->driverCommand().leftStickAnalog[1]  = 0.f;
      bridge->driverCommand().rightStickAnalog[0] = 0.f;
      done = true;
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
      // 1. decelerate. Zeroing the stick from cruise is a step input - the
      //    robot pitches forward and goes down, which is why every completed
      //    star run used to be followed by a [FALL] three seconds later.
      for (int k = 20; k >= 0; --k) {
        bridge->driverCommand().leftStickAnalog[1] = nv * (float)k / 20.f;
        bridge->driverCommand().rightStickAnalog[0] = 0.f;
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
      }
      bridge->driverCommand().leftStickAnalog[1] = 0.f;

      // 2. settle on its feet
      bridge->setControlMode(3);                 // K_BALANCE_STAND
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      const auto& s1 = bridge->robotRunner()->getStateEstimate();
      const float stand_z = s1.position[2];
      const float stand_roll  = std::fabs(s1.rpy[0]) * 57.2958f;
      const float stand_pitch = std::fabs(s1.rpy[1]) * 57.2958f;

      // 3. LIE DOWN: re-enter STAND_UP with a low target so the same Cartesian
      //    interpolation that stands the robot up lowers it instead.
      setStandUpHeight(0.07);
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

    bridge->driverCommand().leftStickAnalog[1]  = nv;
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
  g.cmd_port    = 9100;
  g.sensor_port = 9101;
  bridge.setGazebo(g);

  std::thread(navThread, &bridge).detach();

  bridge.run();
  return 0;
}
