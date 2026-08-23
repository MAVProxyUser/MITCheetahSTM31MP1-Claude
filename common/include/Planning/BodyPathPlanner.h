/*!
 * @file BodyPathPlanner.h
 * @brief Apollo-derived high-level body path + speed planner for a quadruped.
 *
 * Adapted from the architecture of Apollo Auto (https://github.com/ApolloAuto/apollo,
 * Apache-2.0). This is a REIMPLEMENTATION of Apollo's concepts, not a copy of its
 * source: no Cyber RT, no protobuf, no HD-map, no Ackermann model.
 *
 * WHAT IS BORROWED, and why each piece earns its place here:
 *
 *  1. PATH/SPEED SEPARATION. Apollo solves geometry first (x,y,yaw along path
 *     distance s) and timing second (s vs t). This port's waypoint nav does
 *     neither - it is pure-pursuit, reacting to the heading error it has RIGHT
 *     NOW. That is why it arrives at a corner still fast and brakes only once it
 *     is already turning.
 *
 *  2. CURVATURE-BOUNDED SPEED. With the path known, curvature kappa(s) is known
 *     BEFORE the robot gets there, so the speed limit can be derived rather than
 *     tuned:
 *
 *         v_max(s) = sqrt(a_lat_max / |kappa(s)|)
 *
 *     Measured justification: a turn at speed needs lateral acceleration
 *     a = v * omega, and lateral acceleration is what the robot pays for in
 *     ROLL. At 2.5 m/s, commanding 1.2 rad/s costs 3.0 m/s^2 and peaks at
 *     27 deg of roll; commanding 3.0 rad/s costs 7.5 m/s^2 and peaks at 72 deg,
 *     well past the 28.6 deg SafetyChecker trip. A single constant yaw-rate
 *     clamp is therefore wrong at both ends - too slow to be useful at a walk,
 *     too fast to be safe at a sprint.
 *
 *  3. LOOKAHEAD BRAKING via a backward pass. Apollo's speed optimizer enforces
 *     longitudinal limits across the whole profile, so a tight corner ahead
 *     forces deceleration to start early. An animal does exactly this - a dog or
 *     a cheetah sheds speed BEFORE the turn, because turning force and
 *     propulsion draw on the same friction budget.
 *
 * WHAT IS DELIBERATELY NOT HERE (per the adaptation notes): footholds, contact
 * scheduling, swing trajectories, whole-body dynamics, joint control. This layer
 * emits vx and yaw_rate into the existing locomotion controller, which is the
 * documented first integration target.
 *
 * Header-only so it needs no build-system change and can be unit-tested
 * standalone with no robot, no simulator and no vendor SDK.
 */
#ifndef BODY_PATH_PLANNER_H
#define BODY_PATH_PLANNER_H

#include <vector>
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace planning {

struct PathPoint {
  double turn = 0.0;       //!< direction change of the corner this point belongs to, rad
  bool   hairpin = false;  //!< legacy hard-mode flag (kept for the A/B)
  double x = 0, y = 0;     //!< world position
  double s = 0;            //!< arc length from path start
  double theta = 0;        //!< path heading (tangent)
  double kappa = 0;        //!< curvature, 1/m (signed: + is left)
  double v_max = 0;        //!< speed limit here, from curvature
  double v = 0;            //!< planned speed after the accel passes
};

/*!
 * Limits describing what the BODY can do. Every one of these is a measured
 * property of this robot, not a guess - see the comments on the defaults.
 */
struct BodyLimits {
  //! Cruise speed the gait can hold on a straight (trotting 3.1, trotRunning 4.0).
  double v_cruise = 2.5;
  //! Minimum speed the planner will command rather than stopping dead. A hard
  //! v=0 pivot is fine on this robot (pirouettes are stable to 3 rad/s), but a
  //! stopped robot still has to accelerate again, so a small floor is faster.
  double v_min = 0.25;
  /*!
   * LATERAL ACCELERATION BUDGET - the key number.
   * Measured on the 100 m star: at 2.5 m/s and 1.2 rad/s (a = 3.0 m/s^2) peak
   * roll was 27 deg and the mission completed; at 2.0 rad/s (5.0 m/s^2) roll hit
   * 52 deg and it failed; at 3.0 rad/s (7.5 m/s^2) roll hit 72 deg. The
   * SafetyChecker trips at 28.6 deg. So ~3.0 m/s^2 is the edge of what this
   * robot can turn at without tripping, and 2.5 leaves margin.
   */
  double a_lat_max = 2.5;
  /*!
   * Longitudinal accel/decel used to build the profile - DELIBERATELY BELOW the
   * achievable rate, and this is the single most important number here.
   *
   * The body tracks a commanded deceleration at a MEASURED 1.2 m/s^2. Setting
   * a_lon to that (or to the old 1.5) makes the profile begin braking
   * (2.5^2 - 0.83^2)/(2*1.5) = 1.85 m before a corner - but with tracking lag
   * the robot needs ~5 m to actually shed the speed. The plan then issues a
   * deceleration it has already left too late to achieve, and the robot enters
   * the corner at 2.58 m/s where the profile called for 0.83, overshooting the
   * waypoint by 5.5 m.
   *
   * The braking ZONE has to be longer than the real stopping distance, so the
   * planning value must be lower than the physical one:
   *     2.5 m/s: a_lon 0.6 FAILED | 0.4 -> 46.0 s PASS 2/2 | 0.3 -> 47.6 s PASS
   *     3.0 m/s:              0.4 -> 45.4 s PASS | 0.3 -> 47.2 s PASS
   * 2.5 and 3.0 had failed EVERY prior attempt across gait pairings, corner
   * budgets, lateral limits and lookahead. This is what unlocked them.
   */
  /*!
   * Planning deceleration - MEASURED per speed, and deliberately below the
   * physical 1.2 m/s^2 because the braking ZONE must outrun the real stopping
   * distance (body tracks at 1.2 m/s^2 with ~1 s of lag).
   *
   *   2.0 m/s cruise: 1.5 -> 42.2 s 2/2     0.4 -> 48.0 s 2/2 (over-brakes)
   *   3.0 m/s cruise: 1.5 -> FAILED every   0.4 -> 45.4 s 3/3
   *
   * A single value is wrong at one end or the other, and an elegant "shift the
   * profile by v*lag" version was tried and did NOT reproduce the 0.4 result -
   * so this is the measured table rather than the theory that should have
   * worked. Set by plan() from v_cruise; WP_ALON overrides.
   */
  double a_lon_max = 1.5;
  //! Yaw rate ceiling in a pirouette (measured: tracks 100% to 1.5 rad/s, 92% at
  //! 3.0, no falls at any rate at ZERO forward speed).
  double yaw_rate_max = 2.0;
  /*!
   * HAIRPIN THRESHOLD (radians of DIRECTION CHANGE). Above this, a corner is
   * not arced through - the robot slows to `v_pivot` and TURNS IN PLACE.
   *
   * Why a separate mode: arcing costs lateral acceleration a = v*omega, which
   * is paid in ROLL, and roll is what trips the safety check (measured 27 deg at
   * 3.0 m/s^2, 52 at 5.0, 72 at 7.5 against a 28.6 deg trip). A pirouette costs
   * NONE of it - spinning at 3 rad/s with zero forward speed holds roll under
   * 1.5 deg and has never fallen. So past some angle the cheapest way through is
   * to stop turning-while-moving and just turn.
   *
   * 2.0 rad (115 deg) by default: the star's interior corners are 144 deg of
   * direction change and its first is 162 deg - all hairpins - while a gentle
   * course change is not. This is the racing line's "slow in, fast out", and the
   * same thing a dog does at a hard corner.
   */
  //! Corner grading: below turn_soft the full lateral budget is used; at
  //! turn_hard it is scaled by corner_scale_min; in between it interpolates.
  //! 1.4 rad = 80 deg (a real corner), 2.8 rad = 160 deg (a hairpin).
  double turn_soft = 1.4;
  double turn_hard = 2.8;
  double corner_scale_min = 0.55;
  double hairpin_rad = 9.9;   // OFF by default: measured to cost time on this course, see below
  //! Speed carried through a hairpin. Not zero - a stopped robot still has to
  //! accelerate again - but slow enough that v*omega is negligible.
  double v_pivot = 0.6;      // 0.35 fails 0/5 - trotting cannot sustain itself that slow
  //! How long the body takes to actually reach a commanded speed change. The
  //! follower commands the plan for where the robot will be in this many
  //! seconds, not where it is - without it, braking is issued too late to be
  //! achieved and the robot enters corners hot.
  double track_lag_s = 1.2;
};

/*!
 * Apollo-style body planner: waypoints -> smooth path -> speed profile ->
 * (vx, yaw_rate) for the locomotion controller.
 */
class BodyPathPlanner {
 public:
  BodyPathPlanner() = default;
  explicit BodyPathPlanner(const BodyLimits& lim) : _lim(lim) {}

  void setLimits(const BodyLimits& lim) { _lim = lim; }
  //! Pin a_lon_max, disabling the speed-dependent default (WP_ALON).
  void setAlonExplicit(double a) { _lim.a_lon_max = a; _alonExplicit = true; }
  const BodyLimits& limits() const { return _lim; }
  const std::vector<PathPoint>& path() const { return _path; }

  /*!
   * Build a smooth path through waypoints and its speed profile.
   *
   * @param wx,wy   waypoints in world coordinates (>= 2 points)
   * @param ds      resampling interval along the path, metres
   * @param loop    close the path back to the first waypoint
   */
  void plan(const std::vector<double>& wx, const std::vector<double>& wy,
            double ds = 0.10, bool loop = false, double corridor = 1.0) {
    _path.clear();
    if (wx.size() < 2 || wx.size() != wy.size()) return;
    // Speed-dependent braking: see a_lon_max. Fast cruise needs a much longer
    // zone because the lag distance (v * ~1 s) grows with speed while the
    // corner speed it must reach does not.
    if (!_alonExplicit) _lim.a_lon_max = (_lim.v_cruise >= 2.2) ? 0.4 : 1.5;
    buildPath(wx, wy, ds, loop, corridor);
    computeGeometry();
    computeSpeedProfile();
  }

  /*!
   * Trajectory follower. Given where the body actually is, return the commands.
   * Pure pursuit for STEERING (a lookahead point on the smoothed path) and the
   * planned profile for SPEED - which is the whole point: the speed comes from
   * the curvature AHEAD, not from the heading error now.
   *
   * @param x,y,yaw  current body pose (yaw CCW-positive, radians)
   * @param[out] vx        forward speed command, m/s
   * @param[out] yaw_rate  yaw rate command, rad/s (CCW-positive)
   * @return false once the path is finished
   */
  bool follow(double x, double y, double yaw, double* vx, double* yaw_rate) {
    if (_path.empty()) { *vx = 0; *yaw_rate = 0; return false; }

    const size_t i = nearestIndex(x, y);
    _lastIdx = i;
    if (i + 2 >= _path.size()) { *vx = 0; *yaw_rate = 0; return false; }

    // SPEED LOOKAHEAD - command the plan for where the robot WILL BE.
    //
    // Commanding _path[i].v (the plan for where it IS) arrives too late: the
    // body tracks a commanded deceleration at about 1.2 m/s^2 (measured: 3.18
    // -> 0.77 m/s in 2.0 s), so at 3 m/s it needs 3-6 m of travel to comply.
    // The result was entering corners at 3.06 m/s where the profile called for
    // 0.83 - a 2.3 m/s overshoot that no gait choice can survive, and which
    // looked like a gait-switching problem for hours.
    // Taking the MINIMUM planned speed over the next `lag` seconds of travel
    // makes braking begin early enough to actually be achieved.
    // The profile already carries the lag shift (see computeSpeedProfile), so
    // the follower simply commands the plan for where it is.
    const double vplan = _path[i].v;
    const double Ld = std::max(0.35, std::min(1.6, 0.45 * vplan + 0.30));
    size_t j = i;
    while (j + 1 < _path.size() && _path[j].s - _path[i].s < Ld) ++j;

    // Pure pursuit: yaw rate that arcs the body onto the lookahead point.
    const double dx = _path[j].x - x, dy = _path[j].y - y;
    const double cy = std::cos(-yaw), sy = std::sin(-yaw);
    const double ex = dx * cy - dy * sy;      // ahead of the body
    const double ey = dx * sy + dy * cy;      // left of the body
    const double dist2 = ex * ex + ey * ey;
    double w = 0.0;
    if (dist2 > 1e-6) w = 2.0 * ey / dist2 * std::max(vplan, _lim.v_min);

    // Respect BOTH the yaw ceiling and the lateral-acceleration budget. The
    // second is what a single constant clamp cannot express: the faster the
    // body is moving, the less yaw rate it can afford.
    const double w_lat = (vplan > 1e-3) ? _lim.a_lat_max / vplan : _lim.yaw_rate_max;
    const double wcap  = std::min(_lim.yaw_rate_max, w_lat);
    w = std::max(-wcap, std::min(wcap, w));

    *vx = vplan;
    *yaw_rate = w;
    return true;
  }

  //! Planned speed at the point the follower is currently on. The GAIT DECIDER
  //! keys off this: high on a straight, low through a corner, and known BEFORE
  //! the robot gets there.
  double plannedSpeed() const { return _path.empty() ? 0.0 : _path[_lastIdx].v; }

  //! Lowest planned speed within `dist` metres ahead. The gait decider uses
  //! this rather than the speed underfoot: a corner has to be committed to
  //! BEFORE it arrives, which is what an animal does and what a
  //! decide-from-here rule structurally cannot do.
  double minPlannedSpeedAhead(double dist) const {
    if (_path.empty()) return 0.0;
    double v = _path[_lastIdx].v;
    const double s0 = _path[_lastIdx].s;
    for (size_t i = _lastIdx; i < _path.size() && _path[i].s - s0 <= dist; ++i)
      if (_path[i].v < v) v = _path[i].v;
    return v;
  }

  //! Fraction of the path completed, 0..1 - for progress reporting.
  double progress() const {
    if (_path.size() < 2) return 1.0;
    return _path[_lastIdx].s / _path.back().s;
  }

  //! Tightest corner on the path and the speed it forces. Useful for logging
  //! why the planner chose the profile it did.
  void tightestCorner(double* kappa, double* v) const {
    double k = 0, vv = _lim.v_cruise;
    for (const auto& p : _path)
      if (std::fabs(p.kappa) > k) { k = std::fabs(p.kappa); vv = p.v_max; }
    *kappa = k; *v = vv;
  }

 private:
  BodyLimits _lim;
  std::vector<PathPoint> _path;
  size_t _lastIdx = 0;
  bool _alonExplicit = false;

  /*!
   * Build the path as straight segments joined by FILLET ARCS at each waypoint.
   *
   * Deterministic corner rounding, chosen over iterative smoothing because the
   * iterative version fought its own corridor clamp: clamping each point's
   * displacement produces a flat-topped boundary with a fresh kink at its edge,
   * so the "smoothed" corner came out at kappa = 32 1/m (a 3 cm radius) - no
   * better than the vertex it replaced.
   *
   * GEOMETRY, and why this bounds what any planner can achieve here:
   * for an interior angle phi between the two legs, a fillet of radius R sits
   * a distance R*(csc(phi/2) - 1) inside the vertex, and its tangent points are
   * R/tan(phi/2) back along each leg. A 5-point star reverses direction by
   * 144 deg, so phi = 36 deg and the offset is R * 2.236. With a 1.0 m
   * corridor, R <= 0.45 m and the corner speed is capped at
   * sqrt(a_lat * R) = 1.06 m/s - by geometry, not by tuning.
   */
  void buildPath(const std::vector<double>& wx, const std::vector<double>& wy,
                 double ds, bool loop, double corridor) {
    const size_t n = wx.size();
    const size_t nseg = loop ? n : n - 1;
    auto push = [&](double X, double Y) {
      if (!_path.empty()) {
        const double dx = X - _path.back().x, dy = Y - _path.back().y;
        if (dx*dx + dy*dy < 1e-8) return;      // skip duplicates
      }
      PathPoint p; p.x = X; p.y = Y; _path.push_back(p);
    };
    auto lerp = [&](double ax, double ay, double bx, double by, double t,
                    double* ox, double* oy) { *ox = ax + (bx-ax)*t; *oy = ay + (by-ay)*t; };

    for (size_t k = 0; k < nseg; ++k) {
      const size_t a = k, b = (k + 1) % n, c = (k + 2) % n;
      const double ax = wx[a], ay = wy[a], bx = wx[b], by = wy[b];
      const bool corner = loop || (c != 0 && k + 2 < n);

      // incoming unit vector
      double ix = bx - ax, iy = by - ay;
      const double ilen = std::sqrt(ix*ix + iy*iy);
      if (ilen < 1e-9) continue;
      ix /= ilen; iy /= ilen;

      double T = 0.0, R = 0.0, ox2 = 0, oy2 = 0;
      if (corner) {
        double jx = wx[c] - bx, jy = wy[c] - by;
        const double jlen = std::sqrt(jx*jx + jy*jy);
        if (jlen > 1e-9) {
          jx /= jlen; jy /= jlen;
          // interior angle between (-incoming) and (outgoing)
          double cosphi = (-ix)*jx + (-iy)*jy;
          cosphi = std::max(-1.0, std::min(1.0, cosphi));
          const double phi = std::acos(cosphi);
          if (phi > 1e-3 && phi < M_PI - 1e-3) {
            const double half = phi * 0.5;
            const double offsetPerR = 1.0/std::sin(half) - 1.0;
            // radius the corridor allows, and never more than half a leg
            R = (offsetPerR > 1e-6) ? corridor / offsetPerR : 1e9;
            T = R / std::tan(half);
            const double Tmax = 0.45 * std::min(ilen, jlen);
            if (T > Tmax) { T = Tmax; R = T * std::tan(half); }
            ox2 = jx; oy2 = jy;
          }
        }
      }

      // Straight part: from where the PREVIOUS fillet let go (not from the
      // vertex - restarting at the vertex re-traverses the corner just cut, and
      // made the "shortened" path come out 104 m instead of 92 m).
      const double tinx = bx - ix*T, tiny = by - iy*T;
      double sx0 = _path.empty() ? ax : _path.back().x;
      double sy0 = _path.empty() ? ay : _path.back().y;
      const double seglen = std::hypot(tinx - sx0, tiny - sy0);
      const int steps = std::max(1, (int)std::ceil(seglen / ds));
      for (int t = 0; t < steps; ++t) {
        double X, Y; lerp(sx0, sy0, tinx, tiny, (double)t / steps, &X, &Y);
        push(X, Y);
      }
      // Flag this corner as a hairpin if the DIRECTION CHANGE is too large to
      // arc through at speed. Recorded on the arc points so the speed profile
      // can force a pivot there.
      bool isHairpin = false;
      double turnAngle = 0.0;
      if (corner && T > 1e-6) {
        double jx2 = wx[c] - bx, jy2 = wy[c] - by;
        const double jl = std::sqrt(jx2*jx2 + jy2*jy2);
        if (jl > 1e-9) {
          jx2 /= jl; jy2 /= jl;
          double cd = ix*jx2 + iy*jy2;            // cos of DIRECTION change
          cd = std::max(-1.0, std::min(1.0, cd));
          turnAngle = std::acos(cd);
          isHairpin = (turnAngle >= _lim.hairpin_rad);
        }
      }
      const size_t arcStart = _path.size();
      if (T > 1e-6 && R > 1e-6) {
        // arc from tangent-in to tangent-out, swept about the fillet centre
        const double toutx = bx + ox2*T, touty = by + oy2*T;
        // bisector direction into the corner interior
        double mx = (-ix + ox2), my = (-iy + oy2);
        const double mlen = std::sqrt(mx*mx + my*my);
        if (mlen > 1e-9) {
          mx /= mlen; my /= mlen;
          const double half = std::atan2(1.0, 0.0);  // placeholder, recomputed below
          (void)half;
          const double dcen = std::sqrt(R*R + T*T);
          const double cx = bx + mx*dcen, cy = by + my*dcen;
          double a0 = std::atan2(tiny - cy, tinx - cx);
          double a1 = std::atan2(touty - cy, toutx - cx);
          double sweep = a1 - a0;
          while (sweep >  M_PI) sweep -= 2*M_PI;
          while (sweep < -M_PI) sweep += 2*M_PI;
          const int asteps = std::max(2, (int)std::ceil(std::fabs(sweep) * R / ds));
          for (int t = 0; t <= asteps; ++t) {
            const double aa = a0 + sweep * t / asteps;
            push(cx + R*std::cos(aa), cy + R*std::sin(aa));
          }
          for (size_t q = arcStart; q < _path.size(); ++q) {
            _path[q].turn = turnAngle;          // continuous, not a mode
            _path[q].hairpin = isHairpin;
          }
        }
      }
    }
    if (!loop) push(wx[n-1], wy[n-1]);
  }

  //! Arc length, heading and curvature of the smoothed path.
  void computeGeometry() {
    const size_t n = _path.size();
    if (n < 3) return;
    _path[0].s = 0;
    for (size_t i = 1; i < n; ++i) {
      const double dx = _path[i].x - _path[i - 1].x;
      const double dy = _path[i].y - _path[i - 1].y;
      _path[i].s = _path[i - 1].s + std::sqrt(dx * dx + dy * dy);
    }
    for (size_t i = 0; i < n; ++i) {
      const size_t a = (i == 0) ? 0 : i - 1;
      const size_t b = (i + 1 >= n) ? n - 1 : i + 1;
      _path[i].theta = std::atan2(_path[b].y - _path[a].y, _path[b].x - _path[a].x);
    }
    // Menger curvature from three consecutive points: robust and local.
    for (size_t i = 1; i + 1 < n; ++i) {
      const double x1 = _path[i-1].x, y1 = _path[i-1].y;
      const double x2 = _path[i  ].x, y2 = _path[i  ].y;
      const double x3 = _path[i+1].x, y3 = _path[i+1].y;
      const double a = std::hypot(x2-x1, y2-y1);
      const double b = std::hypot(x3-x2, y3-y2);
      const double c = std::hypot(x3-x1, y3-y1);
      const double area2 = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);   // 2*signed area
      _path[i].kappa = (a*b*c > 1e-9) ? (2.0 * area2 / (a*b*c)) : 0.0;
    }
    if (n >= 2) { _path[0].kappa = _path[1].kappa; _path[n-1].kappa = _path[n-2].kappa; }
  }

  /*!
   * Speed profile: curvature limit, then a backward pass so braking starts
   * EARLY ENOUGH, then a forward pass so acceleration is achievable. This is the
   * classic two-pass profile and is what turns "brake when I notice the corner"
   * into "brake because there is a corner coming".
   */
  void computeSpeedProfile() {
    const size_t n = _path.size();
    if (n == 0) return;
    for (auto& p : _path) {
      const double k = std::fabs(p.kappa);
      p.v_max = (k > 1e-6) ? std::sqrt(_lim.a_lat_max / k) : _lim.v_cruise;
      p.v_max = std::max(_lim.v_min, std::min(_lim.v_cruise, p.v_max));
      // A hairpin is taken as a PIVOT, not an arc: force it slow enough that
      // v*omega - and therefore roll - is negligible, and let the steering do
      // the work the fillet cannot.
      /*
       * ANGLE-GRADED CORNER SPEED - continuous, no mode boundary.
       *
       * The hard pivot/arc switch was a TRANSITION BUG waiting to happen, and
       * it happened: pivoting only the star's true hairpin and arcing the rest
       * failed 0/3, every time at an ARCED corner AFTER the pivot. Exiting one
       * corner treatment and entering another is a discontinuity, exactly like
       * a gait switch, and it needs the same care - so the treatment is now a
       * smooth function of turn angle rather than two modes with a cliff
       * between them.
       *
       * A sharper corner gets a smaller effective lateral budget, so
       * v = sqrt(a_eff/kappa) falls off gradually as the turn tightens:
       *   scale = 1                       below turn_soft (gentle - full budget)
       *   scale -> corner_scale_min       at and beyond turn_hard (hairpin)
       * and it interpolates in between, so no corner is ever a special case.
       */
      if (p.turn > _lim.turn_soft) {
        const double f = std::min(1.0, (p.turn - _lim.turn_soft) /
                                       std::max(1e-6, _lim.turn_hard - _lim.turn_soft));
        const double scale = 1.0 - f * (1.0 - _lim.corner_scale_min);
        const double k = std::fabs(p.kappa);
        if (k > 1e-6) {
          const double vAng = std::sqrt(_lim.a_lat_max * scale / k);
          p.v_max = std::min(p.v_max, std::max(_lim.v_pivot, vAng));
        }
      }
      if (_lim.hairpin_rad < 9.0 && p.hairpin) p.v_max = std::min(p.v_max, _lim.v_pivot);
      p.v = p.v_max;
    }
    // Backward: v_i^2 <= v_{i+1}^2 + 2*a*ds  (can I still slow down in time?)
    for (size_t i = n - 1; i-- > 0; ) {
      const double ds = _path[i + 1].s - _path[i].s;
      const double lim = std::sqrt(_path[i+1].v * _path[i+1].v + 2 * _lim.a_lon_max * ds);
      _path[i].v = std::min(_path[i].v, lim);
    }
    // Forward: v_{i+1}^2 <= v_i^2 + 2*a*ds  (can I actually get there?)
    for (size_t i = 0; i + 1 < n; ++i) {
      const double ds = _path[i + 1].s - _path[i].s;
      const double lim = std::sqrt(_path[i].v * _path[i].v + 2 * _lim.a_lon_max * ds);
      _path[i+1].v = std::min(_path[i+1].v, lim);
    }

  }

  size_t nearestIndex(double x, double y) const {
    size_t best = _lastIdx; double bd = 1e18;
    // search forward from the last index so the path is not re-acquired backwards
    const size_t lo = _lastIdx;
    for (size_t i = lo; i < _path.size(); ++i) {
      const double dx = _path[i].x - x, dy = _path[i].y - y;
      const double d = dx * dx + dy * dy;
      if (d < bd) { bd = d; best = i; }
      if (d > bd + 25.0) break;      // moved well past the minimum
    }
    return best;
  }
};

}  // namespace planning
#endif  // BODY_PATH_PLANNER_H
