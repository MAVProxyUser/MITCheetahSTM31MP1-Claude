#ifndef CHEETAH_MISSION_ANALYZER_H
#define CHEETAH_MISSION_ANALYZER_H

/*!
 * @file MissionAnalyzer.h
 * @brief Turn a bare waypoint list into an ANNOTATED mission the robot reads.
 *
 * ---------------------------------------------------------------------------
 * WHY
 * ---------------------------------------------------------------------------
 * Everything about a mission that can be known, is known before the dog takes a
 * step. The waypoints are fixed, the geometry is fixed, the curvature and the
 * speed each point allows are fixed. Yet this port has kept re-deriving that
 * information at 50 Hz, badly, from filtered signals:
 *
 *   - the gait decider asked `minPlannedSpeedAhead()` every tick and could not
 *     tell a star vertex from an atom lobe, because both read as "slow ahead".
 *     They want opposite gaits. That single ambiguity is why every gait-switch
 *     test in this project came back neutral.
 *   - the height governor learned the gait's vertical bob from a live envelope
 *     follower, having first got it wrong with a mean and before that with a
 *     constant tuned on one gait. The gait is known in advance.
 *   - three separate "context-blind constant" bugs, each one a number measured
 *     on one gait at one speed and applied as universal.
 *
 * This is the Apollo split the port already borrowed for the speed profile,
 * applied one level up: the PLANNER decides, once, with the whole route in
 * front of it; the controller executes. A decision that can be made with more
 * information, earlier, and only once, should be.
 *
 * ---------------------------------------------------------------------------
 * WHAT IT PRODUCES
 * ---------------------------------------------------------------------------
 * The route is cut into SEGMENTS, each carrying what the robot needs there:
 *
 *   regime        straight / transient corner / sustained curve
 *   radius_min    tightest point in the segment
 *   v_cap         what the curvature allows
 *   gait          which gait to be in, decided from the regime's DURATION
 *   height_bias   how much stance-height margin to pre-load before it
 *   time_cost     seconds this segment loses against free cruising
 *
 * `time_cost` is the map of where the gains and losses actually are. Optimising
 * a segment that costs 0.1 s is wasted effort no matter how bad it looks; this
 * says which ones are worth attacking. It is also the honest way to compare two
 * courses, or the same course under two configurations.
 *
 * At runtime the robot does a lookup - `segmentAt(s)` - instead of evaluating
 * thresholds against a noisy estimate. Nothing is re-derived in the loop.
 */

#include "BodyPathPlanner.h"
#include <cstdio>
#include <string>
#include <vector>
#include "../../../stm32mp1/gazebo/ShmTrace.h"   // per-tick/text SHM tracing - see that file's own header

namespace planning {

enum Regime { REGIME_STRAIGHT = 0, REGIME_TRANSIENT = 1, REGIME_SUSTAINED = 2 };

struct MissionSegment {
  double s0 = 0, s1 = 0;      //!< arc-length span, m
  int    regime = REGIME_STRAIGHT;
  double radius_min = 1e9;    //!< tightest radius inside the segment, m
  double v_cap = 0;           //!< slowest curvature-limited speed inside, m/s
  double v_plan_min = 0;      //!< slowest PLANNED speed inside, m/s
  double a_lat_peak = 0;      //!< peak v^2 * kappa the plan will actually pull
  int    gait = -1;           //!< recommended gait number
  double height_bias = 0;     //!< recommended stance-height pre-load, m
  double time_cost = 0;       //!< s lost INSIDE this segment vs cruising
  /*! s lost by this TURN including the braking and acceleration it forces on
   *  the neighbouring straights. The plain time_cost blames a straight for
   *  slowing down, which is backwards - on the star it reported the worst
   *  segment as a STRAIGHT costing 3.48 s, when what costs 3.48 s is the
   *  corner that straight is braking for. Straights carry 0 here; the whole
   *  deficit is attributed to whatever forced it. */
  double blame_cost = 0;
  double length() const { return s1 - s0; }
};

struct MissionPolicy {
  //! Curvature that counts as "turning". 0.15 = a 6.7 m radius.
  double kappa_min = 0.15;
  /*! Run length above which a flight gait is judged unable to hold on.
   *  A flight gait can spend a metre or two in a turn; it cannot spend twenty
   *  gait cycles there. Measured: trotRunning is 32/32 on the star, whose
   *  corners are over in a metre, and 3/8 on the atom, whose curvature is held
   *  for ten-plus metres. */
  double sustained_m = 3.0;
  /*! Do not switch gait for a segment shorter than this. A gait change costs a
   *  ~500 ms settling window, so a switch that cannot be held for longer than
   *  it takes to make is a loss - this is the decision Apollo would call
   *  "is the manoeuvre worth it". */
  double min_switch_m = 2.0;
  int    gait_fast = 5;       //!< flight gait for straights and transients
  int    gait_sustained = 9;  //!< for curvature held long enough to matter
  double hbias_max = 0.04;    //!< m of pre-load at full lateral budget
  /*!
   * Hard speed ceiling for SUSTAINED-curve segments, m/s, independent of how
   * gentle the curve is. Measured on the oval: every radius tried - 3.0, 5.0,
   * 7.0 m - fails at a 3.0 m/s cruise, while the same robot takes the star's
   * transient corners at 3.3 (8/8) and runs 100 m straight at 3.0. R=7.0 is
   * only 1.29 m/s^2 of lateral load, so curvature is not what is binding.
   * Continuous turning has its own, lower speed envelope than either straight
   * running or transient cornering, and nothing in a per-point
   * lateral-acceleration budget can express that. 0 disables.
   */
  double v_sustained_max = 2.5;
};

class MissionAnalyzer {
 public:
  /*! Analyse a planned path. Call once, after BodyPathPlanner::plan(). */
  void analyze(const BodyPathPlanner& planner, const MissionPolicy& pol) {
    _pol = pol;
    _seg.clear();
    const auto& p = planner.path();
    if (p.size() < 2) return;
    const double v_cruise = planner.limits().v_cruise;
    const double a_lat    = planner.limits().a_lat_max;

    // ---- 1. cut the route into turning and non-turning runs ----
    std::vector<std::pair<size_t, size_t>> runs;   // [i0, i1) of one regime
    bool turning = std::fabs(p[0].kappa) >= pol.kappa_min;
    size_t start = 0;
    for (size_t i = 1; i < p.size(); ++i) {
      const bool t = std::fabs(p[i].kappa) >= pol.kappa_min;
      if (t != turning) { runs.push_back({start, i}); start = i; turning = t; }
    }
    runs.push_back({start, p.size()});

    // ---- 2. absorb straights too short to be worth a switch ----
    // Two corners separated by a metre of straight are one corner as far as the
    // gait is concerned; leaving the gap in produces switch chatter.
    for (size_t k = 1; k + 1 < runs.size(); ++k) {
      const double len = p[runs[k].second - 1].s - p[runs[k].first].s;
      const bool isStraight = std::fabs(p[runs[k].first].kappa) < pol.kappa_min;
      if (isStraight && len < pol.min_switch_m) {
        runs[k - 1].second = runs[k + 1].second;
        runs.erase(runs.begin() + k, runs.begin() + k + 2);
        --k;
      }
    }

    // ---- 3. annotate ----
    for (const auto& r : runs) {
      MissionSegment g;
      g.s0 = p[r.first].s;
      g.s1 = p[r.second - 1].s;
      double kmax = 0, vplan_min = 1e9, vcap = 1e9, alat = 0, t_actual = 0;
      for (size_t i = r.first; i < r.second; ++i) {
        const double k = std::fabs(p[i].kappa);
        if (k > kmax) kmax = k;
        if (p[i].v < vplan_min) vplan_min = p[i].v;
        if (p[i].v_max < vcap) vcap = p[i].v_max;
        const double al = p[i].v * p[i].v * k;
        if (al > alat) alat = al;
        if (i > r.first) {
          const double ds = p[i].s - p[i-1].s;
          const double v  = std::max(0.05, 0.5 * (p[i].v + p[i-1].v));
          t_actual += ds / v;
        }
      }
      g.radius_min = (kmax > 1e-6) ? 1.0 / kmax : 1e9;
      g.v_cap      = (vcap  < 1e8) ? vcap : v_cruise;
      g.v_plan_min = (vplan_min < 1e8) ? vplan_min : v_cruise;
      g.a_lat_peak = alat;

      const bool isTurn = kmax >= pol.kappa_min;
      if (!isTurn)                          g.regime = REGIME_STRAIGHT;
      else if (g.length() >= pol.sustained_m) g.regime = REGIME_SUSTAINED;
      else                                   g.regime = REGIME_TRANSIENT;

      // A switch that cannot be held longer than it takes to make is a loss.
      const bool worthSwitching = g.length() >= pol.min_switch_m;
      g.gait = (g.regime == REGIME_SUSTAINED && worthSwitching)
               ? pol.gait_sustained : pol.gait_fast;

      // Pre-load height in proportion to the lateral load the plan will pull.
      g.height_bias = pol.hbias_max * std::min(1.0, alat / std::max(1e-6, a_lat));

      // What this segment costs against simply cruising it.
      g.time_cost = t_actual - g.length() / std::max(0.05, v_cruise);
      _seg.push_back(g);
    }

    /*
     * BLAME THE CAUSE, NOT THE PLACE. A straight where the plan runs below
     * cruise is not slow because it is a straight - it is braking for the next
     * turn, or still recovering from the last one. Walk the whole profile and
     * hand every metre of deficit to the turn responsible: decelerating means
     * the turn ahead, accelerating means the turn behind. Turns keep their own
     * internal cost too. This is the map worth optimising against; the
     * per-segment column is not.
     */
    for (auto& g : _seg) g.blame_cost = (g.regime == REGIME_STRAIGHT) ? 0.0 : g.time_cost;
    auto turnFor = [&](size_t i, bool ahead) -> MissionSegment* {
      // nearest turn segment ahead of / behind path index i
      const double s = p[i].s;
      MissionSegment* best = nullptr;
      for (auto& g : _seg) {
        if (g.regime == REGIME_STRAIGHT) continue;
        if (ahead ? (g.s0 >= s) : (g.s1 <= s)) {
          if (!best) best = &g;
          else if (ahead ? (g.s0 < best->s0) : (g.s1 > best->s1)) best = &g;
        }
      }
      return best;
    };
    for (size_t i = 1; i < p.size(); ++i) {
      if (std::fabs(p[i].kappa) >= pol.kappa_min) continue;   // turns own theirs
      const double ds = p[i].s - p[i-1].s;
      const double v  = std::max(0.05, 0.5 * (p[i].v + p[i-1].v));
      const double deficit = ds / v - ds / std::max(0.05, v_cruise);
      if (deficit <= 0.0) continue;
      const bool decel = p[i].v < p[i-1].v;
      MissionSegment* t = turnFor(i, decel);
      if (!t) t = turnFor(i, !decel);
      if (t) t->blame_cost += deficit;
    }
    _total = p.back().s;
  }

  /*! Push the analysis back into the planner: apply the sustained-curve speed
   *  ceiling and rebuild the profile. Call right after analyze(), then re-run
   *  analyze() so the segment table reflects the plan the robot will fly. */
  void applyTo(BodyPathPlanner& planner, const MissionPolicy& pol) {
    if (pol.v_sustained_max <= 0.0) return;
    bool any = false;
    for (const auto& g : _seg)
      if (g.regime == REGIME_SUSTAINED) {
        planner.capSpeedOverRange(g.s0, g.s1, pol.v_sustained_max);
        any = true;
      }
    if (any) planner.recomputeSpeedProfile();
  }

  //! Runtime lookup: which segment is arc-length `s` in?
  const MissionSegment* segmentAt(double s) const {
    for (const auto& g : _seg) if (s >= g.s0 && s <= g.s1) return &g;
    return _seg.empty() ? nullptr : &_seg.back();
  }

  /*! Lookup for the segment `lead` metres ahead - what the robot should be
   *  preparing for, rather than what it is standing on. Gait changes and
   *  height pre-load both need to happen BEFORE the demand arrives. */
  const MissionSegment* segmentAhead(double s, double lead) const {
    return segmentAt(s + lead);
  }

  const std::vector<MissionSegment>& segments() const { return _seg; }

  //! Human-readable mission brief - the preplanned map of gains and losses.
  void print() const {
    if (_seg.empty()) { shmtrace::logf(0.0, "[mission] no segments"); return; }
    static const char* NAME[3] = {"straight", "transient", "SUSTAINED"};
    double lost = 0;
    shmtrace::logf(0.0, "[mission] %zu segments over %.1f m", _seg.size(), _total);
    shmtrace::logf(0.0, "[mission] %6s %7s  %-10s %8s %7s %7s %6s %7s %8s %8s",
           "s0", "len", "regime", "Rmin", "v_cap", "v_plan", "gait", "h_bias",
           "cost_s", "blame_s");
    char rb[16];
    for (const auto& g : _seg) {
      lost += g.time_cost;
      if (g.radius_min > 999.0) snprintf(rb, sizeof rb, "%8s", "straight");
      else                      snprintf(rb, sizeof rb, "%8.2f", g.radius_min);
      shmtrace::logf(0.0, "[mission] %6.1f %7.1f  %-10s %8s %7.2f %7.2f %6d %7.3f %+8.2f %+8.2f",
             g.s0, g.length(), NAME[g.regime], rb,
             g.v_cap, g.v_plan_min, g.gait, g.height_bias, g.time_cost, g.blame_cost);
    }
    // Where the time actually goes, which is the point of the whole table.
    const MissionSegment* worst = &_seg[0];
    for (const auto& g : _seg) if (g.blame_cost > worst->blame_cost) worst = &g;
    int nsus = 0, nsw = 0; int prev = -1;
    for (const auto& g : _seg) {
      if (g.regime == REGIME_SUSTAINED) ++nsus;
      if (prev >= 0 && g.gait != prev) ++nsw;
      prev = g.gait;
    }
    shmtrace::logf(0.0, "[mission] %.2f s lost to constraints; costliest FEATURE %+.2f s "
           "at s=%.1f (%s, R=%.2f m) once its braking zone is charged to it",
           lost, worst->blame_cost, worst->s0,
           NAME[worst->regime], worst->radius_min);
    shmtrace::logf(0.0, "[mission] %d sustained-curve segments, %d gait changes planned",
           nsus, nsw);
  }

 private:
  MissionPolicy _pol;
  std::vector<MissionSegment> _seg;
  double _total = 0;
};

}  // namespace planning
#endif  // CHEETAH_MISSION_ANALYZER_H
