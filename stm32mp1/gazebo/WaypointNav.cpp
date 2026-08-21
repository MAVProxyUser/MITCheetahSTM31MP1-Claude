#include <cmath>
#include <cstdio>
#include "WaypointNav.hpp"

static const double EARTH_R = 6378137.0;
static const double DEG2RAD = M_PI / 180.0;

void WaypointNav::makeCircle(float radius_m, int points, float speed) {
  if (points > MAXWP) points = MAXWP;
  _n = points;
  for (int i = 0; i < points; ++i) {
    // Angles run 1..points, NOT 0..points-1: at a=0 the circle passes through
    // the robot's own start position, so a waypoint there is "reached" on the
    // first tick and the mission skips a leg. Ending at a=2*pi closes the loop
    // back at the start instead.
    float a = 2.f * (float)M_PI * (float)(i + 1) / (float)points;
    _wp[i].north = radius_m * sinf(a);
    _wp[i].east  = radius_m * (1.f - cosf(a));   // circle tangent to start, centre due east
    _wp[i].speed = speed;
  }
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] circle mission: %d waypoints, r=%.1f m, v=%.2f m/s\n",
         _n, radius_m, speed);
  for (int i = 0; i < _n; ++i)
    printf("[nav]   wp%02d  N=%7.2f  E=%7.2f  v=%.2f\n", i, _wp[i].north, _wp[i].east, _wp[i].speed);
  fflush(stdout);
}

void WaypointNav::makeStar(float radius_m, int points, float speed) {
  // Visit every 2nd vertex of a regular n-gon: for odd n that traces a star
  // polygon in one closed stroke (n=5 -> the classic pentagram), which is the
  // mission the OpenPilot build flies.
  if (points > MAXWP) points = MAXWP;
  int step = (points % 2 == 1) ? 2 : 1;   // even n has no single-stroke star
  _n = points;
  // Rotate the whole pattern so the FIRST waypoint sits due NORTH - the dog
  // spawns facing north, so the mission opens with a straight leg instead of a
  // ~140 degree pivot-in-place (the pivot is the crawl's least stable move and
  // has faceplanted live demos). The star's shape is rotation-invariant.
  float a0 = 2.f * (float)M_PI * (float)(step % points) / (float)points;
  for (int i = 0; i < points; ++i) {
    int v = ((i + 1) * step) % points;    // +1 so we do not start on our own spot
    float a = 2.f * (float)M_PI * (float)v / (float)points - a0;
    _wp[i].north = radius_m * cosf(a);
    _wp[i].east  = radius_m * sinf(a);
    _wp[i].speed = speed;
  }
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] star mission: %d points, r=%.1f m, v=%.2f m/s\n", _n, radius_m, speed);
  for (int i = 0; i < _n; ++i)
    printf("[nav]   wp%02d  N=%7.2f  E=%7.2f\n", i, _wp[i].north, _wp[i].east);
  fflush(stdout);
}

void WaypointNav::makeOutAndBack(float distance_m, float speed) {
  _n = 2;
  _wp[0] = {distance_m, 0.f, speed};
  _wp[1] = {0.f, 0.f, speed};
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] out-and-back mission: %.1f m at %.2f m/s\n", distance_m, speed);
  fflush(stdout);
}

void WaypointNav::setOrigin(double lat_deg, double lon_deg) {
  _lat0 = lat_deg;
  _lon0 = lon_deg;
  _mPerDegLon = EARTH_R * DEG2RAD * cos(lat_deg * DEG2RAD);
  _originSet = true;
  printf("[nav] local origin set: %.7f, %.7f\n", lat_deg, lon_deg);
  fflush(stdout);
}

void WaypointNav::toLocal(double lat_deg, double lon_deg, float* north, float* east) const {
  *north = (float)((lat_deg - _lat0) * DEG2RAD * EARTH_R);
  *east  = (float)((lon_deg - _lon0) * _mPerDegLon);
}

bool WaypointNav::update(float n, float e, float yaw, float speed, float dt,
                         float* v_cmd, float* yawrate) {
  *v_cmd = 0.f;
  *yawrate = 0.f;
  if (_n == 0 || _complete) return false;

  const NavWaypoint& wp = _wp[_idx];
  float dn = wp.north - n;
  float de = wp.east - e;
  float dist = sqrtf(dn * dn + de * de);
  _lastDist = dist;

  // ---- ARRIVAL (OpenPilot conditionDistanceToTarget) ----
  bool arrived = false;

  // half-plane: past the waypoint along the inbound leg, within a corridor
  if (_legValid) {
    float legN = wp.north - _legFromN;
    float legE = wp.east - _legFromE;
    float legLen = sqrtf(legN * legN + legE * legE);
    if (legLen > 0.3f) {
      float un = legN / legLen, ue = legE / legLen;
      float relN = n - wp.north, relE = e - wp.east;
      float along = relN * un + relE * ue;            // >0 = past the waypoint
      float cross = fabsf(-relN * ue + relE * un);    // lateral miss
      if (along > 0.f && cross < corridor * accept_radius) arrived = true;
    }
  }
  // acceptance radius, optionally confirmed by speed + dwell
  if (!arrived && dist <= accept_radius) {
    if (confirm_speed > 0.f) {
      if (speed > confirm_speed) {
        _dwell = 0.f;
      } else {
        _dwell += dt;
        if (_dwell >= confirm_dwell) arrived = true;
      }
    } else {
      arrived = true;
    }
  }
  if (dist > accept_radius && !arrived) _dwell = 0.f;

  if (arrived) {
    printf("[nav] reached wp%02d (N=%.2f E=%.2f) dist=%.2f\n", _idx, wp.north, wp.east, dist);
    fflush(stdout);
    _legFromN = wp.north; _legFromE = wp.east; _legValid = true;
    _dwell = 0.f;
    ++_idx;
    if (_idx >= _n) {
      if (loop) {
        _idx = 0;
      } else {
        _complete = true;
        printf("[nav] MISSION COMPLETE\n"); fflush(stdout);
        return false;
      }
    }
    // recompute against the new target this same tick
    dn = _wp[_idx].north - n;
    de = _wp[_idx].east - e;
    dist = sqrtf(dn * dn + de * de);
  }

  // ---- STEER: pure pursuit toward the active waypoint ----
  // yaw convention: 0 = +north, positive = CCW (toward +... west), matching
  // the estimator's rpy[2] about world +z with x=north/y=east handled below.
  float bearing = atan2f(de, dn);            // rad, 0 = north, +ve = east
  float err = bearing - yaw;
  while (err >  (float)M_PI) err -= 2.f * (float)M_PI;
  while (err < -(float)M_PI) err += 2.f * (float)M_PI;

  float wz = kp_heading * err;
  if (wz >  max_yawrate) wz =  max_yawrate;
  if (wz < -max_yawrate) wz = -max_yawrate;

  // ---- SPEED: cruise, easing off near the point and when badly mis-aimed ----
  float v = _wp[_idx].speed;
  // Do NOT taper speed into the point: easing off near the waypoint makes the
  // dog loiter on top of it, and the acceptance test then fires late. Drive at
  // cruise right through the point; only slow for a genuine stop-type waypoint.
  if (confirm_speed > 0.f && dist < slow_radius)
    v *= fmaxf(0.3f, dist / slow_radius);
  // Turn FIRST, then go. Beyond ~35 deg of heading error, stop translating and
  // pivot - arcing toward the target while badly mis-aimed is what drew those
  // long loops instead of straight legs.
  // Slow hard when mis-aimed so the dog turns rather than arcs, but never to
  // zero: a full stop deadlocks the mission if the achievable yaw rate is lower
  // than the commanded one (observed: 90 s pinned at v=0 with yaw saturated).
  float aerr = fabsf(err);
  // Turn-first speed shaping. The 0.22 floor was sized for the statically
  // stable crawl, which can pivot on the spot quite happily. A DYNAMIC gait
  // cannot: slowing to 0.13 m/s while commanding a saturated yaw rate is the
  // worst case for it, because a trot or a walk needs forward momentum to stay
  // up - measured, the MPC walk fell over at the first star corner doing
  // exactly that. A higher floor makes the robot ARC through the corner
  // instead of pivoting in it.
  if (aerr > 0.25f) v *= fmaxf(turn_speed_floor, 1.f - (aerr - 0.25f) / 0.9f);
  *v_cmd = v;
  *yawrate = wz;
  return true;
}
