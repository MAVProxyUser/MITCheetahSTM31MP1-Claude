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
  double relief = 0;       //!< stride-scale ground mismatch, m (DEM)
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
  /*! TERRAIN AWARENESS (OPEN-7, 2026-08-28). Friction coefficient of the
   *  ground this mission is planned on, -1 when unknown (then nothing
   *  below changes and every previously validated result is bit-identical).
   *  Set from the conductor's own terrain kind via $WP_TERRAIN_MU.
   *
   *  The rule is physics, not a tuned table: a turn needs lateral
   *  acceleration a_lat = v*omega, and the ground can only supply mu*g of
   *  it before the feet slide, so the planner must never PLAN a corner it
   *  cannot be pushed around. It matches the measured terrain sweep
   *  exactly (TERRAIN.md Phase 1, 20 ground-truthed cells): with the
   *  default budget 2.5 m/s^2, mu*g binds only below mu ~= 0.26 - and ice
   *  (mu 0.15 -> 1.32 m/s^2) is the ONE surface that failed, at every tier
   *  that asked for real lateral force, while every surface at mu >= 0.35
   *  passed with no time cost at all. So this predicts the observed
   *  boundary rather than encoding it. */
  double mu_terrain = -1.0;
  double terrain_safety = 0.9;   //!< margin on mu*g (feet are point contacts)
  /*! Hard speed ceiling for terrains whose GEOMETRY (not friction) is the
   *  limit - the procedural rolling/rough heightmaps, where walking is
   *  measured INTERMITTENT and fails silently. -1 = no ceiling. */
  double v_terrain_max = -1.0;
  //! DEM RELIEF RESPONSE (OPEN-7). The conductor samples the heightmap along
  //! THIS mission's planned path and hands over a per-metre profile of the
  //! stride-scale height mismatch - how much the ground moves under one
  //! stride, which is what decides whether four feet can be on a common
  //! plane. That, not average grade, is what separates the two geometry
  //! kinds measured here: `rolling` has the larger PEAK grade (41.5%) and
  //! costs walking nothing, while `rough` is gentler on average (5.4%) and
  //! takes walking's ceiling from 2.5 to ~2.0-2.25. Mean stride mismatch
  //! tells them apart where grade does not: 16 mm vs 9 mm.
  //! v_cap = v_cruise / (1 + relief_k * mismatch / relief_ref)
  //! DEFAULT 0 = INERT. The law's shape is physical but its gain is not yet
  //! measured - there is exactly one anchor point - and this project's own
  //! rule is that a guessed constant in the planner is the failure mode the
  //! whole terrain programme exists to avoid. $WP_RELIEF_K turns it on.
  //!
  //! MEASURED 2026-09-03 (campaign c8, rough/walking, N=10/arm, interleaved,
  //! uncapped): k=0.25 -> 9/10 at 1.48 m/s, k=0.5 -> 9/10 at 1.24, k=1.0 ->
  //! 10/10 at 0.95; the global 2.0 cap -> 7/10 at 1.61 (pooled clean-server
  //! 42/49 for that rung). Parity on pass rate at lower throughput: relief
  //! can only slow, and on uniformly rough ground it is just a worse global
  //! cap. It stays 0. Its case is MIXED terrain, which has not been built.
  double relief_k = 0.0;
  double relief_ref = 0.02;   //!< metres of stride mismatch per unit of k
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
  /*!
   * ACCELERATION OUT of a corner, separate from braking INTO it.
   *
   * An agility dog does not brake and drive at the same rate: it sheds speed
   * progressively into the wrap, plants, and then DRIVES OUT hard. A single
   * a_lon forces the exit to be as lazy as the entry, which on a course with a
   * corner every 20 m is most of the lap spent accelerating gently.
   *
   * Braking must stay conservative because the zone has to outrun the real
   * stopping distance; acceleration has no such constraint - the measured
   * capability is ~1.2 m/s^2 and the gait tolerates ramps far better than it
   * tolerates arriving hot.
   */
  double a_accel_max = 0.0;   // 0 = use a_lon_max (symmetric). Set to drive out harder.
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
  double corner_scale_min = 1.0;   // 1.0 = grading OFF. Measured: 0.55 costs 2 s and 0.4 costs 3 s for NO reliability gain on this course. Kept as a knob for courses with genuinely varied corner angles.
  /*!
   * SAME turn_soft/turn_hard grading, applied to the FILLET RADIUS instead of
   * the lateral-acceleration budget. corner_scale_min trades RELIABILITY for
   * time on a corner the robot can already make; this trades POSITIONAL
   * ACCURACY for time on one it can already make - a fillet of radius R
   * misses the true vertex by R*(csc(phi/2)-1) (see buildPath's own comment),
   * so a fixed corridor cuts every corner the same amount regardless of how
   * sharp it is, missing an acute vertex by a visibly large margin while a
   * gentle one is untouched. 1.0 = OFF (unchanged geometry, current
   * behaviour, still the right default for a course whose corners are all
   * mild). Below 1.0, an acute corner's EFFECTIVE corridor shrinks toward
   * corridor*corridor_scale_min as its direction change grows past
   * turn_soft toward turn_hard - and because computeSpeedProfile derives
   * v_max from the path's OWN measured curvature (buildPath lays the fillet
   * down; the numerical circumradius kappa comes from those exact points),
   * a tighter fillet here automatically commands a slower, near-pivot speed
   * through it too - one geometric fix does both jobs, they were never two
   * separate levers.
   */
  double corridor_scale_min = 1.0;
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

  /*!
   * Load a DEM profile sampled along THIS mission's planned path
   * (conductor/terrain_profile.py writes it; $WP_TERRAIN_PROFILE names it).
   * CSV: s_m,z_m,grade,stride_mismatch_m. Called before plan(); the values
   * are interpolated onto the resampled path by arc length.
   * Returns false if the file is absent - which is the normal case for flat
   * ground and for every surface kind, and must never be an error.
   */
  bool loadTerrainProfile(const char* path) {
    _reliefS.clear(); _reliefV.clear();
    if (!path || !*path) return false;
    FILE* f = fopen(path, "r");
    if (!f) return false;
    char line[256];
    bool first = true;
    double sumv = 0.0, maxv = 0.0;
    while (fgets(line, sizeof(line), f)) {
      if (first) { first = false; if (line[0] == 's') continue; }
      double s_m = 0, z = 0, g = 0, mm = 0;
      if (sscanf(line, "%lf,%lf,%lf,%lf", &s_m, &z, &g, &mm) == 4) {
        _reliefS.push_back(s_m); _reliefV.push_back(mm);
        sumv += mm; if (mm > maxv) maxv = mm;
      }
    }
    fclose(f);
    if (_reliefS.empty()) return false;
    printf("[plan] DEM profile: %zu samples over %.1f m, stride mismatch "
           "mean %.3f m max %.3f m%s\n", _reliefS.size(), _reliefS.back(),
           sumv / _reliefS.size(), maxv,
           _lim.relief_k > 0.0 ? "" : " (relief_k=0: reported, not applied)");
    return true;
  }

 private:
  double reliefAt(double s_m) const {
    if (_reliefS.empty()) return 0.0;
    if (s_m <= _reliefS.front()) return _reliefV.front();
    if (s_m >= _reliefS.back())  return _reliefV.back();
    size_t lo = 0, hi = _reliefS.size() - 1;
    while (hi - lo > 1) {
      size_t mid = (lo + hi) / 2;
      if (_reliefS[mid] <= s_m) lo = mid; else hi = mid;
    }
    const double t = (s_m - _reliefS[lo]) /
                     std::max(1e-9, _reliefS[hi] - _reliefS[lo]);
    return _reliefV[lo] + t * (_reliefV[hi] - _reliefV[lo]);
  }

 public:
  const std::vector<PathPoint>& path() const { return _path; }

  /*!
   * STOP POINTS - the profile must brake for a stop the same way it brakes
   * for a corner.
   *
   * The stop sequences (end-of-mission lie-down, and the loop-to-dash
   * interlude) were built and validated when missions cruised at 2.0 m/s on
   * trotting; the campaign recipes then moved the loops to trotRunning at
   * 3.5 and nobody made the STOP speed-aware. Measured consequence, at the
   * root of every stop-point fall: the dog arrives at the stop waypoint at
   * v=3.50 - the profile never brakes because there is no corner there -
   * then the caller's short stick ramp demands ~4.7 m/s^2 against a body
   * that can do ~1.2, the gait scheduler cuts TROT->STAND on the zeroed
   * command while the body is still moving fast, the braking pitch trips
   * the 28.6 deg orientation check within a tick, ESTOP cuts the motors,
   * and the "fall during lie-down" is really a fall during an unplanned
   * crash-stop (with a visible unsteered brake-yaw right before it).
   *
   * A stop is just a point whose planned speed is v_min: registered before
   * plan(), applied ahead of the backward pass, which then builds the same
   * correctly-sized braking zone corners already get (a_lon tuning
   * included). The forward pass accelerates back out of a mid-path stop
   * automatically, which is exactly what the dash sprint needs.
   *
   * The path END is always a stop (every mission finishes with a
   * settle-and-lie-down; a looping path never ends), unless disabled via
   * setEndStop(false) / $WP_END_BRAKE=0 for A/B against old behaviour.
   */
  void addStopXY(double x, double y) { _stopsXY.emplace_back(x, y); }
  void setEndStop(bool on) { _endStop = on; }

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
    // TERRAIN CAPS, applied before any geometry is built so the whole
    // profile (corner speeds, braking zones, the analyzer's own segment
    // caps) is computed against what this ground can actually deliver.
    if (_lim.mu_terrain > 0.0) {
      const double a_fric = _lim.terrain_safety * _lim.mu_terrain * 9.81;
      if (a_fric < _lim.a_lat_max) {
        printf("[plan] terrain mu=%.2f caps lateral budget %.2f -> %.2f m/s^2\n",
               _lim.mu_terrain, _lim.a_lat_max, a_fric);
        _lim.a_lat_max = a_fric;
      }
    }
    if (_lim.v_terrain_max > 0.0 && _lim.v_terrain_max < _lim.v_cruise) {
      printf("[plan] terrain caps cruise %.2f -> %.2f m/s\n",
             _lim.v_cruise, _lim.v_terrain_max);
      _lim.v_cruise = _lim.v_terrain_max;
    }
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
    // Bails out (returns false - the caller's `if (planner.follow(...))
    // nv = pv;` then skips the assignment) once the tracked index is
    // within 2 points of the path's end - i.e. steering/speed control
    // hands back to whatever the caller does on its own for the final
    // approach. TWO THINGS WERE TRIED to make this margin smaller so the
    // braking-zone tail (which lives in exactly these last few points)
    // stays under the follower's control longer, and both made a
    // DIFFERENT failure instead of fixing this one: `i >= size` (no
    // margin) let the pure-pursuit lookahead run out of path to aim at
    // once `i` reached the literal last point, so the "pivot when behind
    // the nose" logic (correct everywhere else) had nothing finite to
    // converge toward and spun in place forever; `i + 1 >= size` (one
    // point of margin) still bailed out during the critical final braking
    // - the tested course's v_min region is concentrated in only the last
    // handful of points, so even a one-point-earlier handoff loses it.
    // Left at the original, safe margin. The actual fix for "the decel
    // ramp starts from a stale, un-braked nv" lives in the CALLER instead
    // (mit_sim_main.cpp's end-of-mission and dash-interlude decel blocks):
    // seed the ramp from `plannedSpeed()`, which reads `_path[_lastIdx].v`
    // and is valid EVEN ON A TICK WHERE follow() BAILED OUT, since
    // `_lastIdx` is updated unconditionally above, before this check.
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

    double vcmd = vplan;
    /*
     * PIVOT WHEN THE TARGET IS BEHIND THE NOSE. Pure pursuit models an ARC
     * through the lookahead point, and that model is degenerate once the
     * point sits behind the body's nose plane (ex < 0) - which is exactly
     * what happens at a super-acute vertex: the profile brakes correctly
     * (the star's 162 deg opening corner plans ~0.04 m/s), but the
     * lookahead's own 0.35 m floor reaches PAST a fillet only ~0.1 m long,
     * so the steering target lands on the exit leg nearly behind the dog.
     * Arcing toward a point behind you while still creeping forward traces
     * a small forward loop around the vertex - the pigtail measured at the
     * star's top corner (and, smaller, at its 144 deg tips: same geometry,
     * less acute). The right maneuver here is the one the yaw envelope
     * says is this robot's most stable: slow to a creep and YAW IN PLACE
     * toward the exit until the target comes back in front, then let the
     * profile re-accelerate. This also self-handles a big tracking upset
     * (body knocked to face away from the path): pivot back onto it at a
     * creep instead of arcing at cruise.
     */
    /*
     * TARGET BEHIND THE NOSE -> PIVOT, BUT RAMP INTO IT. NEVER SNAP.
     *
     * Two failures got us here, and they are opposite ends of the same
     * mistake. First version snapped the command from cruise to v_min with
     * full yaw the instant ex<0: on the out-and-back's 180 that flipped the
     * dog outright (roll 52 / pitch 69). Gating it to creep speed then made
     * it strictly worse - the gate never opened (plan braked only to 1.96
     * against a 1.0 threshold), the pivot never fired, and because a target
     * DIRECTLY BEHIND has zero cross-track error, pure pursuit commanded
     * w=-0.02 and the dog drove straight past the turnaround forever,
     * 115 m and counting.
     *
     * The lesson: pure pursuit is geometrically INCAPABLE of a reversal -
     * ey ~ 0 when the target is straight back, so there is no error to
     * steer on. The pivot is the only thing that can turn the body around,
     * so it must always be available. What must never happen is the STEP.
     * So: bleed the speed command down toward v_min at a bounded rate, and
     * turn at the lateral-acceleration-limited rate for whatever speed the
     * body is actually being asked for. That spirals into the reversal
     * instead of stamping on it, and it self-tightens as the speed drops.
     */
    if (ex < 0.0) {
      // Bounded bleed-down (~3 m/s^2 at a 50 Hz nav loop) instead of a step.
      if (_pivotV <= 0.0) _pivotV = vplan;
      _pivotV = std::max(_lim.v_min, _pivotV - 0.06);
      vcmd = std::min(vplan, _pivotV);
      // Turn as hard as lateral acceleration allows AT THIS SPEED - the same
      // budget the profile uses everywhere else, so the roll cost is bounded.
      const double w_af = (vcmd > 1e-3) ? _lim.a_lat_max / vcmd : _lim.yaw_rate_max;
      const double w_piv = std::min(_lim.yaw_rate_max, w_af);
      // Commit to a side. ey ~ 0 at a true reversal, so a sign test on ey
      // alone dithers; bias by ey when it is meaningful, else hold the last
      // direction so the dog does not oscillate about the reversal.
      if (std::fabs(ey) > 0.05) _pivotDir = (ey >= 0.0) ? 1.0 : -1.0;
      else if (_pivotDir == 0.0) _pivotDir = 1.0;
      w = _pivotDir * w_piv;
    } else {
      _pivotV = -1.0; _pivotDir = 0.0;   // out of the reversal - rearm
    }

    *vx = vcmd;
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

  /*!
   * PREDICTIVE half of body-height control (the reactive half is
   * HeightGovernor). Metres of extra stance height to pre-load for the
   * hardest corner within `dist` metres ahead.
   *
   * This is deliberately the INVERSE of the corner crouch that was measured
   * here and lost. Crouching into a turn is what the animal appears to do and
   * what the earlier CTRL_CORNER_CROUCH lever implemented, and it came back
   * neutral-to-worse - because on this robot the thing that kills a run is a
   * VERTICAL FORCE DEFICIT, and a corner stacks lateral demand on top of it.
   * The margin wanted going in is therefore more height, not less. Section 4.7
   * of Zhang et al. 2022 points the same way: the cheetah's stance leg sits at
   * its LOWEST manipulability, the posture that "can withstand a greater
   * force", and it does not give that up when it needs the ground most.
   *
   * Pre-loading rather than reacting matters because the height loop is
   * deliberately slow (it must not resonate with the gait cycle), so margin
   * asked for at the apex arrives after the apex.
   *
   * @param dist  metres to look ahead
   * @param gain  metres of bias at full lateral budget (0 disables)
   */
  double plannedHeightBias(double dist, double gain) const {
    if (_path.empty() || gain <= 0.0) return 0.0;
    double a_peak = 0.0;
    const double s0 = _path[_lastIdx].s;
    for (size_t i = _lastIdx; i < _path.size() && _path[i].s - s0 <= dist; ++i) {
      // What the plan will actually pull there, not what the corner could
      // demand at cruise - the profile has already slowed for it.
      const double a = _path[i].v * _path[i].v * std::fabs(_path[i].kappa);
      if (a > a_peak) a_peak = a;
    }
    const double f = std::min(1.0, a_peak / std::max(1e-6, _lim.a_lat_max));
    return gain * f;
  }

  /*!
   * HOW LONG the curvature lasts, not how tight it is - the quantity a gait
   * decider actually needs and the one this planner never published.
   *
   * The decider has always keyed off `minPlannedSpeedAhead`, i.e. the MAGNITUDE
   * of the upcoming demand. Magnitude cannot tell these apart:
   *
   *   a star vertex      very high kappa, over in a metre or two. The planner
   *                      brakes hard, the robot powers through, and a flight
   *                      gait is never actually tested - trotRunning goes
   *                      32/32 on the star across 2.5-3.3 m/s.
   *   an atom lobe       moderate kappa held for ten-plus metres, twenty-odd
   *                      gait cycles with no recovery - and there trotRunning
   *                      collapses to 3/8 where trotting manages 7/8.
   *
   * Both read as "low planned speed ahead". They want opposite gaits. So
   * measure the RUN LENGTH of continuous curvature instead: a flight gait can
   * spend a metre or two anywhere, and cannot spend twenty gait cycles in a
   * turn.
   *
   * @param look       metres to search ahead
   * @param kappa_min  curvature that counts as "turning" (1/R)
   * @return metres of continuous turning found, starting anywhere in `look`
   */
  double curveRunAhead(double look, double kappa_min) const {
    if (_path.empty()) return 0.0;
    const double s0 = _path[_lastIdx].s;
    double best = 0.0, run = 0.0;
    for (size_t i = _lastIdx; i < _path.size(); ++i) {
      const double ds = _path[i].s - s0;
      if (ds > look && run <= 0.0) break;      // nothing open, past the window
      if (std::fabs(_path[i].kappa) >= kappa_min) {
        if (i > _lastIdx) run += _path[i].s - _path[i-1].s;
        if (run > best) best = run;
      } else {
        if (ds > look) break;                  // window closed on a straight
        run = 0.0;
      }
    }
    return best;
  }

  /*! Regime of the road ahead, for a gait decider.
   *  0 = straight, 1 = transient corner (brake and power through),
   *  2 = sustained curve (a flight gait cannot hold it). */
  int regimeAhead(double look, double kappa_min, double sustained_m) const {
    const double run = curveRunAhead(look, kappa_min);
    if (run <= 0.0) return 0;
    return (run >= sustained_m) ? 2 : 1;
  }

  /*!
   * Impose an extra speed ceiling over an arc-length range and rebuild the
   * profile. Used by MissionAnalyzer to apply a limit that curvature cannot
   * express.
   *
   * MEASURED, and it is the reason this exists: the oval fails at 3.0 m/s at
   * EVERY radius tried - 3.0, 5.0 and 7.0 m - while the star is 8/8 at 3.3 and
   * the dash holds 3.0 for 100 m. R=7.0 asks for only 1.29 m/s^2 of lateral
   * acceleration, so this is not a curvature limit. The robot simply cannot
   * hold ~3 m/s through CONTINUOUS turning of any tightness, though it takes
   * transient corners at 3.3 happily. A lateral-acceleration budget cannot
   * represent that, because the budget is per-point and the constraint is
   * about duration.
   */
  void capSpeedOverRange(double s0, double s1, double v_cap) {
    if (_path.empty() || v_cap <= 0.0) return;
    if (_extraCap.size() != _path.size()) _extraCap.assign(_path.size(), 0.0);
    for (size_t i = 0; i < _path.size(); ++i)
      if (_path[i].s >= s0 && _path[i].s <= s1)
        _extraCap[i] = (_extraCap[i] > 0.0) ? std::min(_extraCap[i], v_cap) : v_cap;
  }
  //! Rebuild the speed profile after caps have been applied.
  void recomputeSpeedProfile() { computeSpeedProfile(); }

  //! Arc length the follower is currently at, m - the key into the analysed
  //! mission (MissionAnalyzer::segmentAt).
  double currentS() const { return _path.empty() ? 0.0 : _path[_lastIdx].s; }

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
  //! Extra per-point speed ceiling imposed from outside (MissionAnalyzer).
  //! Empty = none. Curvature alone cannot express "this turn is sustained".
  std::vector<double> _extraCap;
  std::vector<double> _reliefS, _reliefV;  //!< DEM profile (OPEN-7)
  size_t _lastIdx = 0;
  bool _alonExplicit = false;
  //! Pivot state: ramped speed command and committed turn direction while a
  //! steering target sits behind the nose (see follow()). -1 / 0 = rearmed.
  double _pivotV = -1.0;
  double _pivotDir = 0.0;
  //! Mid-path stop points (world x,y) - see addStopXY. Resolved to the
  //! nearest path index at profile time, so they survive a re-plan.
  std::vector<std::pair<double, double>> _stopsXY;
  bool _endStop = true;

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
            // ANGLE-GRADED CORRIDOR - see the field comment on
            // corridor_scale_min. turnAngle here is the DIRECTION CHANGE
            // (0 = straight through, pi = full reversal) - the same
            // quantity and the same turn_soft/turn_hard thresholds the
            // speed grading below uses, so "how acute is too acute" is one
            // shared answer, not two. turnAngle = pi - phi.
            double eff_corridor = corridor;
            if (_lim.corridor_scale_min < 1.0) {
              const double turnAngle = M_PI - phi;
              if (turnAngle > _lim.turn_soft) {
                const double f = std::min(1.0, (turnAngle - _lim.turn_soft) /
                                   std::max(1e-6, _lim.turn_hard - _lim.turn_soft));
                eff_corridor = corridor * (1.0 - f * (1.0 - _lim.corridor_scale_min));
              }
            }
            // radius the (possibly graded) corridor allows, and never more
            // than half a leg
            R = (offsetPerR > 1e-6) ? eff_corridor / offsetPerR : 1e9;
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
    size_t _ci = 0;
    for (auto& p : _path) {
      (void)_ci;
      const double k = std::fabs(p.kappa);
      /*
       * TWO INDEPENDENT constraints on cornering speed, not one - and this
       * only ever enforced the first:
       *   traction/roll:  v <= sqrt(a_lat_max / kappa)   (a physics limit)
       *   steering:       v <= yaw_rate_max / kappa       (an actuation limit -
       *                   the body simply cannot yaw fast enough to track a
       *                   curvature this tight at this speed, regardless of
       *                   how much lateral acceleration it could tolerate)
       * They cross over at ordinary corners (traction binds first, which is
       * why gentle-course tuning never needed the second one) but at the
       * angle-graded corridor's own extreme end they diverge hugely: at the
       * star's tightest fillet (R~0.03 m after corridor_scale_min), traction
       * allows 0.31 m/s while steering (yaw_rate_max 1.2 rad/s) allows only
       * 0.036 m/s - an almost 9x gap. Commanding the traction number sends
       * the body into a turn its own steering rate cannot actually track,
       * which does not fail safe - it overshoots the fillet and loops back,
       * the "elephant foot" shape measured on the star's every corner, not
       * just the hairpin. Verified directly (not assumed): an isolated
       * vx=1.0/wz=1.2 test tracks yaw at ~100% commanded, over 35 s, roll
       * 6-9 deg, never falls - the steering CAN be trusted once it is not
       * being asked to also violate its own rate limit.
       */
      double v_max = _lim.v_cruise;
      if (k > 1e-6) {
        const double v_traction = std::sqrt(_lim.a_lat_max / k);
        const double v_steering = _lim.yaw_rate_max / k;
        v_max = std::min(v_traction, v_steering);
      }
      // NOT floored to v_min here - v_min exists to keep _path[0] off a
      // literal zero (follow()'s nearestIndex lookup would never advance
      // past a point commanding exactly 0, see that assignment below).
      // Flooring EVERY point to it would undo the steering cap above right
      // where it matters most: the star's tightest fillet wants ~0.036 m/s,
      // well under v_min's 0.25 - forcing it back up is exactly the
      // traction-only speed that was measured to shank the corner.
      p.v_max = std::min(_lim.v_cruise, v_max);
      // DEM RELIEF CAP (OPEN-7). The conductor sampled the heightmap along
      // this exact path; p.relief is the stride-scale height mismatch here.
      // Slow down where the ground moves under a stride, because that is
      // what stops four feet sharing a plane - the thing measured to cost
      // `rough` a speed rung while `rolling`, with a LARGER peak grade,
      // costs nothing. Recorded on every point regardless, so a run's
      // profile is in the record even when the cap is inert.
      p.relief = reliefAt(p.s);
      if (_lim.relief_k > 0.0 && p.relief > 0.0) {
        const double f = 1.0 + _lim.relief_k * p.relief /
                                std::max(1e-6, _lim.relief_ref);
        p.v_max = std::min(p.v_max, _lim.v_cruise / f);
      }
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
      if (!_extraCap.empty()) {
        const size_t idx = (size_t)(&p - &_path[0]);
        if (idx < _extraCap.size() && _extraCap[idx] > 0.0)
          p.v_max = std::min(p.v_max, _extraCap[idx]);
      }
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
    /*
     * STOP POINTS (see addStopXY's comment): force v down to v_min at the
     * path's end and at every registered mid-path stop BEFORE the backward
     * pass, so the pass builds a real braking zone into each one - the same
     * machinery, and the same already-tuned a_lon, that corners get. v_min
     * rather than a literal 0 for the same reason as _path[0] below: the
     * arrival test needs the dog to actually creep across the waypoint's
     * half-plane, and follow() reads .v with no lookahead - an exact 0 at
     * the nearest point would park it just short of the line forever.
     * Mid-path stops resolve to the nearest path point; the fillet means
     * the path passes within ~corridor of the waypoint itself, which only
     * makes the braking end a metre early - harmless.
     */
    if (_endStop && n >= 2) _path[n - 1].v = std::min(_path[n - 1].v, _lim.v_min);
    /*
     * EVERY PASS OVER A STOP POINT GETS BRAKED, not just the globally
     * nearest path index. The global-nearest version broke the moment a
     * course genuinely visited a stop's coordinates twice: after
     * shiftFirstToOrigin, a closed loop's CLOSURE waypoint sits at the
     * exact coordinates of the path START (0,0), the `<` scan from index 0
     * resolved the tie to s=0 - whose v is already forced to 0 by the
     * at-rest rule, a no-op - and the real closure at the far end of the
     * lap got NO braking at all. Measured (run602 SHM trace): the dog
     * arrived at its own loop closure at vx=+2.98 - full sprint - and the
     * stop sequence became a ~5 m/s^2 crash-stop, pitch -0.09 -> -1.12 rad
     * in 800 ms, face-plant. Star+dash had been validated only BEFORE the
     * shift landed, and the suite's star case ran dash-less, so this
     * regression lived silently until an operator UI run hit it.
     *
     * Fix: clamp v_min at every LOCAL minimum of squared distance that
     * comes within stop_gate of the point. A twice-visited coordinate is
     * braked on both visits (the s=0 one harmlessly - it is already 0); a
     * uniquely-visited stop resolves to its single local minimum, exactly
     * the old behaviour. The 2 m gate comfortably covers every accept
     * radius/corridor in use while rejecting far-side passes of a course
     * that merely comes NEAR a stop's coordinates.
     */
    for (const auto& st : _stopsXY) {
      const double stop_gate = 2.0 * 2.0;   // m^2
      auto d2 = [&](size_t i) {
        const double dx = _path[i].x - st.first, dy = _path[i].y - st.second;
        return dx * dx + dy * dy;
      };
      for (size_t i = 0; i < n; ++i) {
        const double d = d2(i);
        if (d > stop_gate) continue;
        const bool leftOk  = (i == 0)     || d <= d2(i - 1);
        const bool rightOk = (i + 1 >= n) || d <= d2(i + 1);
        if (leftOk && rightOk)
          _path[i].v = std::min(_path[i].v, _lim.v_min);
      }
    }
    // Backward: v_i^2 <= v_{i+1}^2 + 2*a*ds  (can I still slow down in time?)
    for (size_t i = n - 1; i-- > 0; ) {
      const double ds = _path[i + 1].s - _path[i].s;
      const double lim = std::sqrt(_path[i+1].v * _path[i+1].v + 2 * _lim.a_lon_max * ds);
      _path[i].v = std::min(_path[i].v, lim);
    }
    /*
     * THE ROBOT IS AT REST AT s=0, ALWAYS - plan() is only ever called once,
     * at mission setup, before nav takes the stick (mit_sim_main.cpp calls it
     * exactly once; a later mid-mission restart, e.g. after the dash
     * interlude's stand-back-up, reads further into an ALREADY-computed
     * profile and is handled separately, by the caller ramping in from
     * wherever _path[i].v already is - it does not call plan() again). Up to
     * here, _path[0].v is whatever the curvature limit allowed - v_cruise
     * outright on a straight opening tangent, which is exactly the star's
     * case (wp00 is rotated due north to avoid an opening pivot). Left
     * uncorrected, that tells the caller to command near-cruise speed on
     * tick one, while the caller ALSO safety-ramps its own output up from a
     * true standstill (a stepped velocity command was measured to knock the
     * trot over) - two independent ramps, on the same channel, disagreeing
     * about where the robot actually is on the profile. Every corner after
     * the first inherits that arc-length error for the rest of the mission.
     * Measured effect on the star specifically (the only course of the three
     * whose first leg starts at v_max already): 42.4 s against a validated
     * 38.25 s baseline, and a visibly wide, arcing entry into wp00 instead of
     * a clean pivot. Zeroing it here lets the SAME forward accel-limited pass
     * below produce the ramp, arc-length-consistent with the braking-zone
     * math the corners already depend on, so the caller no longer needs (or
     * should apply) a second, independent ramp on the initial takeover.
     *
     * NOT literally 0.0: follow() reads _path[nearestIndex(x,y)].v directly,
     * with no lookahead on the SPEED value (only the steering target point
     * gets a lookahead). The robot starts at essentially _path[0]'s own
     * coordinates, so nearestIndex()==0 on the very first tick, commanding
     * exactly this value - a literal 0.0 here is *vx=0 forever: the robot
     * never moves, so nearestIndex never advances past 0, so vplan never
     * leaves 0. Measured: v pinned at 0.00 for 70+ s, standing in place.
     * _lim.v_min (0.25 m/s default, the same floor computeSpeedProfile
     * already enforces everywhere else in this function) guarantees actual
     * progress from tick one while still being far below the v_max this was
     * fixing - the forward accel-limited pass below still ramps normally
     * from there.
     */
    _path[0].v = _lim.v_min;
    // Forward pass uses the ACCELERATION limit, not the braking one - slow in,
    // fast out. The backward pass above already guaranteed the robot can shed
    // the speed; there is no reason to make it regain it just as slowly.
    for (size_t i = 0; i + 1 < n; ++i) {
      const double ds = _path[i + 1].s - _path[i].s;
      const double aacc = (_lim.a_accel_max > 1e-6) ? _lim.a_accel_max : _lim.a_lon_max;
      const double lim = std::sqrt(_path[i].v * _path[i].v + 2 * aacc * ds);
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
