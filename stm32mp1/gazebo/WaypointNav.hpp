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
  float confirm_speed = 0.f;     //!< m/s; 0 = fly-through waypoints
  float confirm_dwell = 0.f;     //!< s to hold below confirm_speed
  float kp_heading    = 2.2f;    //!< yaw rate per rad of heading error
  float max_yawrate   = 0.7f;    //!< rad/s - what the crawl can actually deliver
  float turn_speed_floor = 0.22f; //!< min speed fraction while turning; raise
                                  //!< for dynamic gaits so they arc, not pivot
  float slow_radius   = 0.8f;    //!< m; ease off speed inside this
  bool  loop          = false;   //!< repeat the mission forever

 private:
  // Raised from 64 for makeAtom: a smooth curve needs waypoints close
  // enough that the planner's fillet arcs reproduce it (~1.2 m -> ~106
  // points for the default rosette). The star missions use 5.
  static const int MAXWP = 256;
  NavWaypoint _wp[MAXWP];
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
