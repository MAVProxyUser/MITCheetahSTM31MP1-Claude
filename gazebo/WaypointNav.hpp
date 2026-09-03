/*!
 * @file WaypointNav.hpp
 * @brief OpenPilot-style waypoint follower for the Cheetah/Go1 port.
 *
 * Ported from NinjaPilot's PathPlanner/PathFollower (see
 * flight/modules/PathPlanner/pathplanner.c), keeping the parts that matter for
 * a ground vehicle and dropping the airframe-specific ones:
 *
 *   - waypoints are (North, East) in metres, local tangent plane, plus a
 *     per-leg cruise speed - the same Waypoint object OpenPilot uses minus
 *     Down (the dog cannot choose its altitude);
 *   - ARRIVAL is the OpenPilot rule: inside the acceptance radius, OR past the
 *     half-plane through the waypoint perpendicular to the inbound leg (and
 *     within a corridor either side). The half-plane test is what stops a
 *     vehicle that overshoots by a few cm from turning around and hooking back
 *     to the point - the single biggest source of ugly ground tracks;
 *   - optional CONFIRM ARRIVAL (speed below a threshold for a dwell time), so
 *     a mission can ask for a precise stop on a corner instead of flying
 *     through it;
 *   - steering is pure pursuit toward the active waypoint with a cross-track
 *     term, converted to a yaw RATE because that is what the gaits consume.
 *
 * Position comes from GPS (lat/lon -> local NE via an equirectangular
 * projection about the first fix), which is exactly what the real robot will
 * have over CAN. Heading comes from the state estimator's yaw.
 */
#ifndef WAYPOINT_NAV_H
#define WAYPOINT_NAV_H

#include <cstddef>

struct NavWaypoint {
  float north;      //!< m, local tangent plane
  float east;       //!< m
  float speed;      //!< m/s cruise for the leg ENDING at this waypoint
};

class WaypointNav {
 public:
  //! Build a closed circle of breadcrumbs (the mission we fly first).
  void makeCircle(float radius_m, int points, float speed);
  //! Straight there-and-back, for speed measurement runs.
  //! Straight sprint of `distance_m` due north, ending at that waypoint.
  void makeDash(float distance_m, float speed);
  void makeOutAndBack(float distance_m, float speed);
  /*! ONE isolated corner: a straight approach leg of `leg_m`, due north from
   *  spawn (so nav takes the stick already aimed correctly, same convention
   *  as makeDash/makeStar), then a single direction change of `angle_deg`
   *  (0 = straight through, 180 = full reversal) into a straight exit leg of
   *  the same length. Built for the per-gait/per-angle cornering envelope:
   *  a controlled, isolated corner at a known angle and a known approach/exit
   *  speed, with no other course geometry to confound the measurement. Angles
   *  above ~150 deg fall into BodyPathPlanner's existing reversal-registered-
   *  as-a-stop path (see CLAUDE.md) - a different maneuver, not a failure of
   *  this mission type. */
  void makeCorner(float leg_m, float angle_deg, float speed);
  /*! Append ONE more waypoint after whatever mission is already built, so a
   *  closed loop (star/oval/atom) gets a straight finishing sprint instead of
   *  ending back where it started. Direction continues along the FINAL leg's
   *  own heading (the vector into the last waypoint), so it needs no idea
   *  what course it is finishing - it just keeps going the way the dog is
   *  already pointed when the loop closes. No-op on an empty or 1-point
   *  mission (nothing to take a heading from). */
  void appendDash(float distance_m, float speed);
  //! N-pointed star (visit every k-th vertex), the OpenPilot demo mission.
  void makeStar(float radius_m, int points, float speed);
  /*! Atom-logo rosette: ONE closed stroke with `lobes` petals through a common
   *  nucleus, and curvature that varies smoothly instead of stepping at
   *  vertices. See the .cpp for the curve and why this is the gentler course.
   *  @param outer_radius_m  distance from the nucleus to a lobe tip
   *  @param lobes           petals (6 = the atom logo, 3 = a trefoil)
   *  @param depth           0..1, how close the stroke comes to the nucleus
   *  @param spacing_m       waypoint spacing along the arc
   *  @param speed           cruise for every leg */
  void makeAtom(float outer_radius_m, int lobes, float depth,
                float spacing_m, float speed);
  /*! Stadium: two long straights joined by two constant-radius 180s. The only
   *  course here with BOTH a flight-gait regime and a sustained-curve regime,
   *  which is what a gait decider needs to have anything to decide. */
  void makeOval(float straight_m, float radius_m, float spacing_m, float speed);
  /*! Four canonical SAR patterns (International Aeronautical and Maritime
   *  Search and Rescue Manual, via Steckenrider et al. 2024 Fig. 1/Eqs 1-7).
   *  Circle search is makeCircle() above. */
  //! Sector search: alternating full/half legs at 120 deg, "reps" six-leg
  //! cycles through a common centre, rotated slightly between cycles.
  void makeSectorSearch(float leg_m, int reps, float speed);
  //! Parallel track (lawnmower) search: "passes" legs of length width_m,
  //! stepped height_m apart.
  void makeParallelTrack(float width_m, float height_m, int passes, float speed);
  //! Expanding square search: outward spiral, "legs" turns of increasing
  //! length (multiples of step_m).
  void makeExpandingSquare(float step_m, int legs, float speed);
  /*! Lissajous search curve (Steckenrider et al. 2024, Eq. 8): X=A*sin(wx*t+
   *  pi/2), Y=A*sin(wy*t), traced for one full period at integer frequency
   *  ratio wx:wy. Higher-order ratios (5:7, 11:9, ...) sweep the area more
   *  densely; see Fig. 2. */
  void makeLissajous(float amplitude_m, int wx, int wy, float spacing_m, float speed);
  /*! Spirograph rosette: the SAME trochoid formula makeAtom uses, at a
   *  different point in its own parameter space - `lobes` used directly as
   *  the rolling-circle ratio k (makeAtom uses k = lobes-1) and `depth`
   *  pushed near 1.0 (makeAtom clamps well short of it). Those two changes
   *  move the curve from makeAtom's own look (small loops pointing
   *  outward from each lobe) to the classic Spirograph look this was
   *  built to match: petals curving inward, converging through a shared,
   *  densely rewoven centre. Found by direct visual comparison against a
   *  reference image, not derived analytically - see CLAUDE.md for the
   *  parameter search.
   *  @param outer_radius_m  overall size
   *  @param lobes           petals (8 = the reference image)
   *  @param depth           0..1, pen offset as a fraction of the rolling
   *                         circle's own orbit radius - near 1.0 gives the
   *                         cusped, densely-woven-centre look; lower values
   *                         open the centre back up and round the lobes
   *  @param spacing_m       waypoint spacing along the arc
   *  @param speed           cruise for every leg */
  void makeSpirograph(float outer_radius_m, int lobes, float depth,
                       float spacing_m, float speed);
  //! Read back a waypoint (for drawing the planned track).
  const NavWaypoint& waypoint(int i) const { return _wp[i]; }

  //! Set the origin of the local tangent plane from the first GPS fix.
  void setOrigin(double lat_deg, double lon_deg);
  bool originSet() const { return _originSet; }
  //! Convert a GPS fix to local north/east metres.
  void toLocal(double lat_deg, double lon_deg, float* north, float* east) const;

  /*! Advance the mission and produce drive commands.
   *  @param n,e        current position, local metres
   *  @param yaw        current heading, rad (0 = +north, CCW positive)
   *  @param speed      current ground speed, m/s (for arrival confirmation)
   *  @param dt         seconds since the last call
   *  @param v_cmd      out: forward speed command m/s
   *  @param yawrate    out: yaw rate command rad/s
   *  @return true while the mission is running, false once it is complete */
  bool update(float n, float e, float yaw, float speed, float dt,
              float* v_cmd, float* yawrate);

  int   activeIndex() const { return _idx; }
  int   count() const { return _n; }
  float lastDistance() const { return _lastDist; }
  bool  complete() const { return _complete; }

  // --- tunables (OpenPilot ConditionParameters equivalents) ---
  float accept_radius = 0.25f;   //!< m, acceptance sphere (hit the point, not a buffer)
  float corridor      = 2.0f;    //!< half-plane lateral bound = corridor*accept
  /*!
   * OPT-IN tighter acceptance radius for the FINAL waypoint only (mission
   * end / loop-closure back to the start point), in place of accept_radius.
   * -1 (default) = off, unchanged behaviour: the final waypoint uses the
   * same accept_radius as every other one.
   *
   * accept_radius does double duty - it is ALSO the corridor width
   * BodyPathPlanner fillets corners with (see mit_sim_main.cpp's
   * `corridor = WP_ACCEPT`), so tuning it for tight cornering (e.g.
   * sector's WP_ACCEPT=1.5) makes every waypoint, INCLUDING the final one,
   * "arrived" a full 1.5 m short of its literal coordinate. That is
   * invisible at an intermediate waypoint (the dog just continues onto the
   * next leg regardless), but at the FINAL one there is no next leg - the
   * mission simply stops there, which is exactly what a closed course's own
   * flown-trail plot shows as a visible gap back to the start point.
   * final_accept_radius decouples "how wide can a fillet be" from "how
   * precisely must the mission actually return to its nominal end point" -
   * the two were never actually the same question.
   */
  float final_accept_radius = -1.f;
  float confirm_speed = 0.f;     //!< m/s; 0 = fly-through waypoints
  float confirm_dwell = 0.f;     //!< s to hold below confirm_speed
  float kp_heading    = 2.2f;    //!< yaw rate per rad of heading error
  float max_yawrate   = 0.7f;    //!< rad/s - what the crawl can actually deliver
  float turn_speed_floor = 0.22f; //!< min speed fraction while turning; raise
                                  //!< for dynamic gaits so they arc, not pivot
  float slow_radius   = 0.8f;    //!< m; ease off speed inside this
  bool  loop          = false;   //!< repeat the mission forever

 private:
  // Raised from 64 for makeAtom (~106 points at 1.2 m spacing), then from
  // 256 for makeLissajous: an integer wx:wy ratio closes in exactly one
  // sweep of t (0..2*pi - sin(wx*t) and sin(wy*t) both return to their
  // start value/phase there, no need for the wx*wy-cycle LCM closure a
  // naive reading of Eq. 8 suggests), but that one sweep still packs in
  // wx+wy oscillations - measured 604 waypoints for the 11:9 ratio at a
  // 1.5 m spacing, comfortably under this with margin for denser ratios.
  static const int MAXWP = 768;
  NavWaypoint _wp[MAXWP];
  /*!
   * Translate the whole course so _wp[0] becomes (0,0) - i.e. the robot's
   * own local origin/true spawn point, per direct instruction: EVERY
   * mission should have the robot standing ON its own first waypoint,
   * always and forever, not travelling there from a separate reference
   * point.
   *
   * UNIVERSAL as of the second time this was asked for: every make*()
   * generator calls this once, at the end of its own body -
   * makeCircle/makeSectorSearch/makeParallelTrack/makeExpandingSquare
   * originally (SAR search patterns only), then makeStar/makeOval/
   * makeAtom/makeSpirograph/makeLissajous extended to match. makeDash is
   * the one deliberate exception: a dash's "first waypoint" IS the far
   * end of the sprint by definition, so there is no separate "pattern
   * centre" to shift away from - shifting it would just move the sprint
   * itself, not fix a spawn-to-wp0 walk that does not exist there.
   *
   * The earlier, narrower scoping here warned that shifting star "would
   * silently invalidate" BodyPathPlanner.h's "wp00 rotated due north"
   * opening-leg logic. Re-examined when this was extended: that logic
   * hard-floors _path[0].v to v_min because the robot is ALWAYS at rest
   * at the start of plan()'s very first call, regardless of whether wp0
   * sits at the true origin or somewhere else - a property of "the robot
   * just spawned", not of this convention. Verified, not just reasoned
   * about: star's own guard mission was re-run after the shift and holds
   * its established baseline (see CLAUDE.md). The old warning was
   * overcautious, not load-bearing.
   *
   * Pure translation, so every corner's angle, every leg's length, every
   * fillet radius and speed the corridor-grading tuning computes are
   * UNCHANGED - only the (north,east) reference point moves.
   */
  void shiftFirstToOrigin() {
    if (_n < 1) return;
    const float dn = _wp[0].north, de = _wp[0].east;
    for (int i = 0; i < _n; ++i) { _wp[i].north -= dn; _wp[i].east -= de; }
  }

 public:
  /*
   * CLOSE THE FINAL LEG - walk back to where the dog started.
   *
   * Some courses finish where they began and some just stop wherever their
   * own parametric math ran out, which is an inconsistency an operator
   * sees immediately on the panel overlay: a drawn plan whose last leg
   * simply is not there. Measured gap from home at the last waypoint,
   * before this existed:
   *
   *     lissajous 0.00 m   spiro 0.05 m   atom 0.43 m   oval 1.20 m
   *     circle    6.89 m   expsquare 18.03 m   sector 15.00 m
   *     parallel 46.10 m
   *
   * The first four close by construction (their curves are periodic); the
   * last four leave a real, walkable leg undone. makeStar closes itself
   * explicitly for the same reason, and that precedent is what this
   * generalises - "closing belongs to the mission itself".
   *
   * Skipped, deliberately, in three cases:
   *   - fewer than 2 waypoints. A dash IS its final leg; sending it home
   *     would silently double every dash into an out-and-back, which is a
   *     DIFFERENT mission this file already distinguishes by name.
   *   - already within min_gap_m of home, so a periodic course does not
   *     collect a pointless sub-metre stub waypoint that the planner would
   *     then have to brake for.
   *   - no room left in _wp (MAXWP), rather than overrunning the array.
   *
   * Home is the local-frame ORIGIN, which is exactly where the dog was
   * standing when the GPS datum was taken - not wp0, which for a course
   * that never called shiftFirstToOrigin can be somewhere else entirely.
   */
  void closeFinalLeg(float min_gap_m = 2.0f);

 private:
  int   _n = 0;
  int   _idx = 0;
  bool  _complete = false;
  float _dwell = 0.f;
  float _lastDist = 0.f;
  // start of the current leg (for the half-plane test)
  float _legFromN = 0.f, _legFromE = 0.f;
  bool  _legValid = false;
  bool  _originSet = false;
  double _lat0 = 0, _lon0 = 0, _mPerDegLon = 0;
};

#endif
