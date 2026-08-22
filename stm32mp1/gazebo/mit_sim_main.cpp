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

#include "Stm32mp1HardwareBridge.h"
#include "MIT_Controller.hpp"
#include "WaypointNav.hpp"
#include "Planning/BodyPathPlanner.h"
#include "rt/rt_gazebo.h"      // gazebo_get_aux(): GPS, the same data the real dog gets over CAN

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
    lim.a_lon_max = getenv("WP_ALON") ? atof(getenv("WP_ALON")) : 1.5;
    lim.yaw_rate_max = nav.max_yawrate;
    planner.setLimits(lim);
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
      // Let the robot come to a stop on its feet before the process ends, so
      // the run scores as a completed mission rather than as a fall.
      std::this_thread::sleep_for(std::chrono::seconds(3));
      exit(0);
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
