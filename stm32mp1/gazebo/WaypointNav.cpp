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
  shiftFirstToOrigin();   // per direct instruction: robot spawns ON wp0
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
  if (points > MAXWP - 1) points = MAXWP - 1;   // one slot reserved for the closing waypoint
  int step = (points % 2 == 1) ? 2 : 1;   // even n has no single-stroke star
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
  /*
   * CLOSE THE LOOP. A pentagram is drawn 0->1->2->3->4->0 - the final stroke
   * back to the first vertex is part of the SHAPE, not an optional extra. This
   * list used to end at the last vertex, and the closing leg only ever existed
   * as a side effect of appendDash()'s return-to-wp00 insert - so a star with
   * the dash disabled visibly stopped one stroke short of the drawn plan
   * (which the UI renders closed). Closing belongs to the mission itself;
   * appendDash() now detects an already-closed course and only appends the
   * sprint. (The oval and atom already close by construction - their waypoint
   * lists trace the full stroke back to the start.)
   */
  _wp[points].north = _wp[0].north;
  _wp[points].east  = _wp[0].east;
  _wp[points].speed = speed;
  _n = points + 1;
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] star mission: %d points, r=%.1f m, v=%.2f m/s\n", _n, radius_m, speed);
  for (int i = 0; i < _n; ++i)
    printf("[nav]   wp%02d  N=%7.2f  E=%7.2f\n", i, _wp[i].north, _wp[i].east);
  fflush(stdout);
}

/*
 * ATOM ROSETTE - a course with CURVES instead of corners.
 *
 * The pentagram star is a polygon: five 36-degree vertices joined by straight
 * legs, so curvature is zero almost everywhere and then effectively infinite
 * for an instant. The planner has to brake to a crawl for each vertex and
 * accelerate out, and the interesting part of the course is five discrete
 * events. Everything this port has learned about cornering came from those
 * five points, which is a narrow diet.
 *
 * This mission is the opposite: one continuous closed stroke whose curvature
 * varies smoothly and never stops changing. The dog is always turning, never
 * hard, and the radius sweeps a 3:1 range every lobe.
 *
 *     x(t) = cos t + A cos(kt)
 *     y(t) = sin t - A sin(kt)          k = lobes - 1,  t in [0, 2pi)
 *
 * This is an epitrochoid-family curve with (k+1)-fold symmetry. With A near 1
 * the radius sweeps from (1+A) at a lobe tip down to (1-A) at the nucleus, so
 * the stroke runs out to a tip, back through the middle, out to the next tip,
 * (k+1) times, and closes - which is exactly the look of the atom logo, drawn
 * without lifting the pen. A < 1 leaves a small nucleus hole rather than
 * passing through the exact centre.
 *
 * Speed is never zero (|1 - kA| > 0 for the defaults) so there are no cusps:
 * every point of the path has finite curvature and the dog never has to stop
 * and pivot. For the default atom:9:6 with A=0.8:
 *
 *     length            127.6 m        (comparable to the 100 m star)
 *     turn radius       2.14 - 6.58 m  (3.1:1, continuously varying)
 *     tightest allows   2.31 m/s at a_lat 2.5 - no braking zone needed
 *     nucleus hole      1.00 m
 *
 * Compare the star, whose 36-degree vertices force a near-stop. This is the
 * gentler course by construction.
 *
 * WAYPOINT SPACING matters here in a way it does not for a 5-point star. The
 * planner rounds the waypoint polyline into fillet arcs, and the fillet is
 * capped at 0.45 x the shorter adjacent leg - so the spacing sets how well the
 * arcs reproduce the true curve. 1.2 m gives ~32 degrees of turn per step at
 * the tightest lobe tip, which reproduces the tip radius to within a few
 * percent. It also has to stay comfortably ABOVE the acceptance radius or nav
 * chews through waypoints faster than the robot moves, so this sets a matching
 * accept_radius unless $WP_ACCEPT overrides it.
 */
void WaypointNav::makeAtom(float outer_radius_m, int lobes, float depth,
                           float spacing_m, float speed) {
  if (lobes < 3) lobes = 3;
  if (depth < 0.05f) depth = 0.05f;
  if (depth > 0.98f) depth = 0.98f;      // 1.0 puts the stroke through the
                                         // exact centre; leave a nucleus
  if (spacing_m < 0.2f) spacing_m = 0.2f;

  const int   k = lobes - 1;
  const float A = depth;
  const float S = outer_radius_m / (1.f + A);
  auto rx = [&](float t) { return S * (cosf(t) + A * cosf(k * t)); };
  auto ry = [&](float t) { return S * (sinf(t) - A * sinf(k * t)); };
  auto vx = [&](float t) { return S * (-sinf(t) - k * A * sinf(k * t)); };
  auto vy = [&](float t) { return S * ( cosf(t) - k * A * cosf(k * t)); };

  /*
   * WHERE TO JOIN THE CURVE. The dog starts at the nucleus, so it has to get
   * out to the stroke somehow, and the obvious choice - start at a lobe tip -
   * is the worst one: at a tip the tangent is perpendicular to the radius, so
   * a radial run-out meets the curve at 90 degrees. That put a hard corner at
   * the entry of a course whose entire point is that it has none, and the
   * planner duly reported a 0.27 m tightest radius on a curve whose true
   * minimum is 2.14 m, braking to 0.82 m/s for an artefact of the approach.
   *
   * So join where the tangent is RADIAL - where the curve is already heading
   * straight out from the nucleus. Those points are the roots of p x v = 0
   * with p . v > 0; there are `lobes` of them, all at the same radius (3.67 m
   * for the default rosette). Rotating the whole figure to put one of them due
   * north means the dog runs straight out of the nucleus on its spawn heading
   * and merges onto the stroke with ZERO turn.
   */
  float t0 = 0.f;
  {
    const int SCAN = 20000;
    float prev = rx(0.f) * vy(0.f) - ry(0.f) * vx(0.f);
    for (int i = 1; i <= SCAN; ++i) {
      const float t = 2.f * (float)M_PI * (float)i / (float)SCAN;
      const float cr = rx(t) * vy(t) - ry(t) * vx(t);
      if ((cr < 0.f) != (prev < 0.f) &&
          rx(t) * vx(t) + ry(t) * vy(t) > 0.f) { t0 = t; break; }
      prev = cr;
    }
  }
  // Rotate so the join lies due north (bearing 0), same reasoning as
  // makeStar's rotation: never open a mission with a pivot in place.
  const float rot = -atan2f(ry(t0), rx(t0));
  const float cr_ = cosf(rot), sr_ = sinf(rot);
  auto px = [&](float t) { return rx(t) * cr_ - ry(t) * sr_; };   // north
  auto py = [&](float t) { return rx(t) * sr_ + ry(t) * cr_; };   // east

  // Walk the curve finely and drop a waypoint every `spacing_m` of ARC length,
  // so the tight lobe tips get the same linear resolution as the open sweeps.
  const int SUB = 40000;
  const float dt = 2.f * (float)M_PI / (float)SUB;
  float prevN = px(t0), prevE = py(t0), acc = 0.f, total = 0.f;
  _n = 0;
  // wp00 IS the join point, so the opening leg is exactly the tangent the dog
  // is already running along.
  _wp[_n].north = prevN; _wp[_n].east = prevE; _wp[_n].speed = speed; ++_n;
  for (int i = 1; i <= SUB && _n < MAXWP; ++i) {
    const float t = t0 + dt * (float)i;
    const float n = px(t), e = py(t);
    const float d = sqrtf((n - prevN) * (n - prevN) + (e - prevE) * (e - prevE));
    acc += d; total += d;
    prevN = n; prevE = e;
    if (acc >= spacing_m) {
      acc -= spacing_m;
      _wp[_n].north = n; _wp[_n].east = e; _wp[_n].speed = speed;
      ++_n;
    }
  }
  // Close the stroke on the starting tip rather than wherever the sampler ran
  // out, so the figure is finished.
  if (_n < MAXWP) {
    _wp[_n].north = px(t0); _wp[_n].east = py(t0); _wp[_n].speed = speed;
    ++_n;
  }

  // Must stay well under the waypoint spacing - see the comment above.
  accept_radius = 0.45f * spacing_m;

  // Report the curvature extremes, which is what makes this course different
  // from the star and the number to check before blaming the controller.
  float rmin = 1e9f, rmax = 0.f;
  for (int i = 0; i < SUB; ++i) {
    const float t  = dt * (float)i;
    const float dx = vx(t), dy = vy(t);
    const float ax = S * (-cosf(t) - k * k * A * cosf(k * t));
    const float ay = S * (-sinf(t) + k * k * A * sinf(k * t));
    const float sp = sqrtf(dx * dx + dy * dy);
    const float kap = fabsf(dx * ay - dy * ax) / (sp * sp * sp);
    if (kap > 1e-9f) { const float r = 1.f / kap;
                       if (r < rmin) rmin = r; if (r > rmax) rmax = r; }
  }
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] atom mission: %d lobes, outer r=%.1f m, depth=%.2f -> "
         "%.1f m single closed stroke, %d waypoints @ %.2f m (accept %.2f)\n",
         lobes, outer_radius_m, A, total, _n, spacing_m, accept_radius);
  printf("[nav]   turn radius %.2f - %.2f m  (a_lat 2.5 allows %.2f - %.2f m/s)"
         "  nucleus hole %.2f m  join at %.2f m due north\n",
         rmin, rmax, sqrtf(2.5f * rmin), sqrtf(2.5f * rmax), S * (1.f - A),
         sqrtf(px(t0) * px(t0) + py(t0) * py(t0)));
  fflush(stdout);
}

/*
 * OVAL / STADIUM - the course that makes a gait SWITCH worth making.
 *
 * Neither existing course rewards switching, and for opposite reasons:
 *
 *   star  long straights joined by five INSTANTANEOUS vertices. The planner
 *         brakes to a crawl for each one, so a flight gait is never actually
 *         tested in a corner - trotRunning goes 32/32 across 2.5-3.3 m/s and
 *         there is nothing to switch away from.
 *   atom  continuous curvature with no straights and no recovery, so there is
 *         nothing to switch TO - trotting wins outright (7/8 vs 3/8 at 2.1).
 *
 * A decider needs a course with BOTH regimes present and each lasting long
 * enough to matter. That is a stadium: two long straights joined by two
 * constant-radius 180-degree ends.
 *
 *   `oval:<straight_m>:<radius_m>`, default oval:40:3.0
 *      straights   2 x 40 m     - long enough for a flight gait to reach speed
 *      ends        2 x pi*3.0 = 9.4 m each of CONTINUOUS R=3.0 m curvature,
 *                                about 15 gait cycles at 2.5 m/s, and capped
 *                                at sqrt(a_lat * R) = 2.74 m/s
 *      total       98.8 m       - directly comparable to the 100 m star
 *
 * The radius is the knob that matters: it sets how hard the sustained regime
 * is, independently of how long the straights are. Unlike the star's fillet
 * corners, this curvature is a property of the COURSE, not of how the planner
 * chose to round a polyline.
 *
 * Starts at the origin heading due north along the first straight, so the dog
 * opens on its spawn heading - same rule as makeStar and makeAtom.
 */
void WaypointNav::makeOval(float straight_m, float radius_m,
                           float spacing_m, float speed) {
  if (radius_m < 0.5f) radius_m = 0.5f;
  if (straight_m < 1.f) straight_m = 1.f;
  if (spacing_m < 0.2f) spacing_m = 0.2f;
  const float R = radius_m, S = straight_m;
  const float arc = (float)M_PI * R;
  const float total = 2.f * S + 2.f * arc;

  _n = 0;
  auto put = [&](float n, float e) {
    if (_n >= MAXWP) return;
    _wp[_n].north = n; _wp[_n].east = e; _wp[_n].speed = speed; ++_n;
  };
  // Walk the perimeter at constant arc spacing. North straight, right-hand
  // 180 at the far end, south straight, right-hand 180 back to the start.
  for (float d = spacing_m; d <= total && _n < MAXWP; d += spacing_m) {
    if (d <= S) {                                   // up the near straight
      put(d, 0.f);
    } else if (d <= S + arc) {                      // far end, centre (S, R)
      const float a = (d - S) / R;                  // 0 .. pi
      put(S + R * sinf(a), R - R * cosf(a));
    } else if (d <= 2.f * S + arc) {                // down the far straight
      put(S - (d - S - arc), 2.f * R);
    } else {                                        // near end, centre (0, R)
      const float a = (d - 2.f * S - arc) / R;
      put(-R * sinf(a), R + R * cosf(a));
    }
  }
  // Close the lap - but only if the sampler did not already finish near the
  // start. A leftover stub leg (0.36 m against a 1.2 m spacing) makes the
  // planner's fillet clamp T to 0.45 x 0.36 and report a 1.46 m radius on an
  // arc that is actually 3.0 m, so the two ends of a symmetric course come out
  // asymmetric and every per-end number after that is wrong.
  if (_n == 0 ||
      std::hypot(_wp[_n-1].north, _wp[_n-1].east) > 0.6f * spacing_m)
    put(0.f, 0.f);

  accept_radius = 0.45f * spacing_m;
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] oval mission: 2 x %.1f m straight + 2 x %.1f m of continuous "
         "R=%.1f m -> %.1f m lap, %d waypoints @ %.2f m (accept %.2f)\n",
         S, arc, R, total, _n, spacing_m, accept_radius);
  printf("[nav]   sustained-curve regime is %.0f%% of the lap and caps at "
         "%.2f m/s (a_lat 2.5); straights are the other %.0f%%\n",
         200.f * arc / total, sqrtf(2.5f * R), 200.f * S / total);
  fflush(stdout);
}

/*
 * THE 100 m DASH: a STRAIGHT SPRINT THAT ENDS AT ITS FINAL WAYPOINT.
 *
 * The panel's "dash" used to be wired to makeOutAndBack, i.e. 100 m out
 * and 100 m BACK - 200 m with a 180 degree reversal in the middle. That is
 * a different course, and a nasty one: pure pursuit is geometrically
 * incapable of a reversal (a target directly behind has zero cross-track
 * error, so there is nothing to steer on), and the reversal is invisible
 * to curvature as well (three collinear points give kappa ~ 0). Every
 * dash failure chased for hours was that phantom turnaround, not the
 * sprint.
 *
 * A dash is one leg, due north, ending where it ends - the same thing
 * appendDash() tacks onto the star/oval/atom, just standing alone. The
 * end-of-mission stop then brakes into the final waypoint exactly as it
 * does on every other course.
 */
void WaypointNav::makeDash(float distance_m, float speed) {
  _n = 1;
  _wp[0] = {distance_m, 0.f, speed};
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] dash mission: %.1f m straight, ending at the final waypoint, "
         "at %.2f m/s\n", distance_m, speed);
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

void WaypointNav::appendDash(float distance_m, float speed) {
  if (_n < 2 || _n + 1 >= MAXWP) return;
  /*
   * TWO waypoints, not one: an explicit return to wp00 (closing the shape
   * for real, not just pointing at it), THEN the dash outward from there.
   * Per direct instruction - "it never went back to the first waypoint
   * before doing the dash" - extrapolating wp[last]'s heading toward wp00
   * (the previous version) got the DIRECTION right but skipped the point
   * itself: the dog would peel off toward wp00 without ever actually
   * arriving, which reads as leaving the course unfinished before sprinting
   * off. Now it genuinely closes the loop first.
   *
   * Heading for the final leg is wp[last] -> wp[0], same reasoning as
   * before: a closed course's last waypoint is usually a VERTEX at some
   * oblique angle (a star tip, an atom lobe), and extrapolating THAT leg's
   * own direction sends the dash further along whatever angle the vertex
   * happened to leave the dog on - a star tip shoots it into open ground at
   * a spike angle. Aiming at wp0 instead is the course's own closing
   * tangent, so the dash continues the SHAPE. With wp0 now a real
   * intermediate stop, the final leg's direction is identical either way
   * (same two points), just computed before appending wp0 rather than after.
   */
  float dn = _wp[0].north - _wp[_n - 1].north;
  float de = _wp[0].east  - _wp[_n - 1].east;
  const float len = sqrtf(dn * dn + de * de);
  if (len < 1.0f) {
    /*
     * The course ALREADY closes - its last waypoint is (at or within a
     * metre of) wp00. makeStar now appends its own closing waypoint, and
     * the oval/atom have always traced their stroke back to the start, so
     * inserting another return point would add a degenerate metre-long leg
     * the dog has to "navigate" at the seam. Just append the sprint,
     * continuing the course's own closing tangent (the final leg's heading,
     * which by construction ends at wp00 - the same direction the old
     * two-point insert produced, minus the duplicate point).
     */
    dn = _wp[_n - 1].north - _wp[_n - 2].north;
    de = _wp[_n - 1].east  - _wp[_n - 2].east;
    const float l2 = sqrtf(dn * dn + de * de);
    if (l2 < 1e-3f) return;
    dn /= l2; de /= l2;
    _wp[_n].north = _wp[_n - 1].north + dn * distance_m;
    _wp[_n].east  = _wp[_n - 1].east  + de * distance_m;
    _wp[_n].speed = speed;
    ++_n;
    printf("[nav] dash finish appended: course already closes at wp%02d, "
           "%.1f m sprint onward to wp%02d  N=%7.2f  E=%7.2f\n",
           _n - 2, distance_m, _n - 1, _wp[_n - 1].north, _wp[_n - 1].east);
  } else {
    // OPEN course (e.g. outback): insert the explicit return to wp00 first,
    // then sprint onward along the closing heading - the original two-point
    // design, still the right one when the loop does not close itself.
    dn /= len; de /= len;

    /*
     * NEAR-CLOSED course - the oval: its closing arc ends 1.2 m short of
     * wp00, so the pre-dash STOP used to sit right off the exit of a
     * continuous R=5 turn, where the dog stops mid-straightening - the
     * residual sideways-tip cell (measured ~1-in-6 even with steered
     * deceleration). The star's stop never shows it because the star
     * arrives down a 20 m straight. Give the oval the SAME end logic:
     * place the stop a few metres PAST wp00 along the dash heading, so
     * the dog closes its loop at the arc exit and then gets a genuine
     * straight run-in - retracing its own opening straight, so the drawn
     * shape is unchanged - before it is asked to stop. A genuinely open
     * course (len >= 3, e.g. outback's 100 m gap) keeps run_in = 0 and
     * its original "return exactly to wp00" semantics.
     */
    // MEASURED AND REVERTED same-day: run_in = 6 m took the oval's stop
    // from 5-of-6 to 0-of-8 - four tips with the pre-stop gait switch
    // accidentally un-held, then four MORE with the switch verifiably held
    // and clean 2.99 ms loops. The straight-run-in theory (match the
    // star's approach) fails its own A/B; stopping right where the arc's
    // speed cap still binds measures BETTER than stopping after a brief
    // re-acceleration up the open straight, for a reason not yet
    // identified. Do not raise this from 0 again without interleaved reps.
    const float run_in = 0.f;
    (void)run_in;

    const int return_idx = _n;
    _wp[_n].north = _wp[0].north + dn * run_in;
    _wp[_n].east  = _wp[0].east  + de * run_in;
    _wp[_n].speed = speed;
    ++_n;

    _wp[_n].north = _wp[return_idx].north + dn * distance_m;
    _wp[_n].east  = _wp[return_idx].east  + de * distance_m;
    _wp[_n].speed = speed;
    ++_n;
    printf("[nav] dash finish appended: return to wp00 (wp%02d N=%7.2f E=%7.2f), "
           "then %.1f m onward to wp%02d  N=%7.2f  E=%7.2f\n",
           return_idx, _wp[return_idx].north, _wp[return_idx].east, distance_m,
           _n - 1, _wp[_n - 1].north, _wp[_n - 1].east);
  }
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

/*
 * FOUR CANONICAL SEARCH-AND-RESCUE PATTERNS, per the International
 * Aeronautical and Maritime Search and Rescue Manual (as reproduced in
 * Steckenrider et al., "Lissajous curves as aerial search patterns",
 * Sci Rep 14:11144, 2024, Fig. 1 and Eqs. 1-7). Circle search is already
 * `makeCircle()` above (same closed-loop breadcrumb trail this port
 * already flies); the other three are new. All four use the SAME
 * (north, east) = (cos, sin) compass-bearing convention as makeStar and
 * open on a bearing of 0 (due north) for the usual reason - the dog
 * spawns facing north, so the mission's first leg is a straight run
 * instead of a pivot in place.
 */

/*
 * SECTOR SEARCH: a "flower" of alternating long/short legs through a
 * common centre, turning 120 deg each leg (Eq. 2-3). D_{k-1} = D on even
 * (k-1), D/2 on odd - six legs bring the heading back to where it
 * started (6 * 120 = 720 = 2 full turns) AND the position back to the
 * centre exactly (verified by hand: the six (D, D/2) legs at 120 deg
 * apart sum to zero). Successive six-leg cycles are rotated by a fixed
 * offset - "an angular offset is often added" per the source - so
 * repeated sweeps interrogate new sectors instead of retracing the same
 * six legs.
 */
void WaypointNav::makeSectorSearch(float leg_m, int reps, float speed) {
  if (leg_m < 1.f) leg_m = 1.f;
  if (reps < 1) reps = 1;
  const float offset = (float)M_PI / 9.f;   // 20 deg between six-leg cycles
  // Bearing convention throughout this file: (north,east) = (cos,sin) of
  // the bearing, bearing 0 = due north - same as makeStar's `a`.
  float bearing = 0.f;
  float n = 0.f, e = 0.f;
  _n = 0;
  for (int r = 0; r < reps && _n < MAXWP; ++r) {
    for (int k = 1; k <= 6 && _n < MAXWP; ++k) {
      const float D = ((k - 1) % 2 == 0) ? leg_m : 0.5f * leg_m;
      n += D * cosf(bearing);
      e += D * sinf(bearing);
      /*
       * SKIP THE CYCLE-CLOSING WAYPOINT, except on the true last leg. The
       * six-leg pattern returns to the EXACT same physical point (the
       * centre) every cycle by construction (D, D/2 legs at 120 deg apart
       * sum to zero - verified by hand and numerically). With reps>1 that
       * means multiple DISTINCT waypoint indices sit on the identical
       * (north, east) coordinate - a follower doing nearest-point-on-path
       * or pure-pursuit target selection cannot tell "arrived at cycle 1's
       * centre" from "cycle 2's" or "cycle 3's" when they are the same
       * point, and measured live it produced exactly the failure mode a
       * self-intersecting path is known for: the flown track cut a smooth
       * blob through the centre instead of tracing the six-leg zigzag,
       * and the mission failed (orientation trip) within one cycle. Only
       * the FINAL cycle's return-to-centre becomes a real waypoint; every
       * intermediate one is skipped so the dog flows straight from one
       * cycle's fifth leg into the next cycle's first, through the centre
       * region without a duplicate point to get confused by.
       */
      const bool cycleClose = (k == 6);
      const bool lastLeg = (r == reps - 1) && (k == 6);
      if (!cycleClose || lastLeg) _wp[_n++] = {n, e, speed};
      bearing += 2.f * (float)M_PI / 3.f;
    }
    bearing += offset;
  }
  shiftFirstToOrigin();   // per direct instruction: robot spawns ON wp0
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] sector search mission: %d legs (%d cycles of 6), leg=%.1f/%.1f m, "
         "v=%.2f m/s\n", _n, reps, leg_m, 0.5f * leg_m, speed);
  fflush(stdout);
}

/*
 * PARALLEL TRACK (lawnmower / boustrophedon) SEARCH: sweep a rectangle
 * of length `width_m` in `passes` legs, stepping `height_m` EAST between
 * each pass (Eq. 4-5, re-oriented so the SWEEP runs north/south and the
 * STEP runs east/west, rather than the paper's own axes - alternate
 * north/south legs of length `width_m`, joined by short east `height_m`
 * legs, covering the whole rectangle exactly once). The re-orientation
 * is only so the first leg opens due north like every other mission
 * here; the pattern traced is identical either way.
 */
void WaypointNav::makeParallelTrack(float width_m, float height_m, int passes,
                                     float speed) {
  if (width_m < 1.f) width_m = 1.f;
  if (height_m < 0.5f) height_m = 0.5f;
  if (passes < 1) passes = 1;
  if (passes > MAXWP / 2 - 1) passes = MAXWP / 2 - 1;
  float n = 0.f, e = 0.f;
  bool north = true;   // first pass heads north; alternates thereafter
  _n = 0;
  for (int p = 0; p < passes && _n < MAXWP; ++p) {
    n += north ? width_m : -width_m;
    _wp[_n++] = {n, e, speed};
    if (p + 1 < passes && _n < MAXWP) {
      e += height_m;
      _wp[_n++] = {n, e, speed};
    }
    north = !north;
  }
  shiftFirstToOrigin();   // per direct instruction: robot spawns ON wp0
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] parallel track mission: %d passes, width=%.1f m, step=%.1f m, "
         "%d waypoints, v=%.2f m/s\n", passes, width_m, height_m, _n, speed);
  fflush(stdout);
}

/*
 * EXPANDING SQUARE SEARCH: an outward spiral from the centre, turning
 * 90 deg each leg with length l*floor((k-1)/2) (Eq. 6-7) - two legs of
 * length l, two of 2l, two of 3l, and so on, exactly like tracing a
 * square spiral by hand.
 */
void WaypointNav::makeExpandingSquare(float step_m, int legs, float speed) {
  if (step_m < 1.f) step_m = 1.f;
  if (legs < 2) legs = 2;
  if (legs > MAXWP - 1) legs = MAXWP - 1;
  float bearing = 0.f;   // due north first leg; see makeSectorSearch's convention note
  float n = 0.f, e = 0.f;
  _n = 0;
  for (int k = 1; k <= legs && _n < MAXWP; ++k) {
    const float len = step_m * (float)((k - 1) / 2);
    // k=1,2 both carry floor(0/2)=0 in Eq. 7 (a zero-length "leg", i.e. no
    // motion) - skip it rather than emit a duplicate waypoint at the origin.
    if (len > 1e-3f) {
      n += len * cosf(bearing);
      e += len * sinf(bearing);
      _wp[_n++] = {n, e, speed};
    }
    bearing += (float)M_PI / 2.f;
  }
  shiftFirstToOrigin();   // per direct instruction: robot spawns ON wp0
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] expanding square mission: %d legs, step=%.1f m, %d waypoints, "
         "v=%.2f m/s\n", legs, step_m, _n, speed);
  fflush(stdout);
}

/*
 * LISSAJOUS SEARCH CURVE (Eq. 8): X(t) = A*sin(wx*t + phix),
 * Y(t) = A*sin(wy*t + phiy), traced for one full period at integer
 * frequency ratio wx:wy - the higher the ratio's terms, the denser the
 * coverage (compare Fig. 2's 1:2, 5:7 and 11:9 examples). Sampled at
 * constant ARC-length spacing, same technique as makeAtom/makeOval,
 * since a Lissajous curve's parametric speed varies a great deal more
 * than its position does. phix=pi/2, phiy=0 (a quarter-period offset
 * between axes) is the standard Lissajous starting phase and happens to
 * open the curve heading due north from the origin - the usual "no pivot
 * to start" rule, for free.
 */
void WaypointNav::makeLissajous(float amplitude_m, int wx, int wy,
                                 float spacing_m, float speed) {
  if (amplitude_m < 1.f) amplitude_m = 1.f;
  if (wx < 1) wx = 1;
  if (wy < 1) wy = 1;
  if (spacing_m < 0.2f) spacing_m = 0.2f;
  const float A = amplitude_m;
  const float phix = (float)M_PI / 2.f, phiy = 0.f;
  // wx and wy are INTEGERS, so sin(wx*t) and sin(wy*t) each complete a
  // whole number of cycles (wx and wy respectively) over t in [0, 2*pi) -
  // that alone closes the curve exactly, regardless of gcd/lcm. (A first
  // version of this used tmax = 2*pi*lcm(wx,wy), reasoning that BOTH axes
  // need to return to their start PHASE - true, but t=2*pi already does
  // that for any integer wx,wy; lcm(wx,wy) cycles of t is only needed if
  // wx,wy were rational non-integers. Measured the difference the hard
  // way: at lcm(11,9)=99 the "closed" curve was 90 km long before this
  // was caught by estimating waypoint counts up front.) One sweep of
  // t:0..2*pi still packs in wx and wy oscillations respectively, which
  // is what makes the higher ratios (5:7, 11:9) visually dense.
  auto px = [&](float t) { return A * sinf((float)wx * t + phix); };  // north
  auto py = [&](float t) { return A * sinf((float)wy * t + phiy); };  // east

  const int SUB = 40000;
  const float tmax = 2.f * (float)M_PI;
  const float dt = tmax / (float)SUB;
  float prevN = px(0.f), prevE = py(0.f), acc = 0.f;
  _n = 0;
  _wp[_n].north = prevN; _wp[_n].east = prevE; _wp[_n].speed = speed; ++_n;
  for (int i = 1; i <= SUB && _n < MAXWP; ++i) {
    const float t = dt * (float)i;
    const float n = px(t), e = py(t);
    acc += sqrtf((n - prevN) * (n - prevN) + (e - prevE) * (e - prevE));
    prevN = n; prevE = e;
    if (acc >= spacing_m) {
      acc -= spacing_m;
      _wp[_n].north = n; _wp[_n].east = e; _wp[_n].speed = speed;
      ++_n;
    }
  }
  if (_n < MAXWP) {   // close the curve back on its own start
    _wp[_n].north = px(0.f); _wp[_n].east = py(0.f); _wp[_n].speed = speed;
    ++_n;
  }
  accept_radius = 0.45f * spacing_m;
  _idx = 0; _complete = false; _legValid = false; _dwell = 0.f;
  printf("[nav] lissajous mission: %d:%d ratio, A=%.1f m, %d waypoints @ "
         "%.2f m (accept %.2f), v=%.2f m/s\n",
         wx, wy, A, _n, spacing_m, accept_radius, speed);
  fflush(stdout);
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

  // FINAL WAYPOINT gets its own, tighter acceptance radius when opted in
  // (see final_accept_radius's field comment): accept_radius also sizes
  // BodyPathPlanner's cornering corridor, so a course tuned for tight
  // corners "arrives" everywhere including the mission-end/loop-closure
  // point a full accept_radius short of it - invisible at an intermediate
  // waypoint (the next leg starts regardless) but a visible gap on a
  // closed course's own flown-trail plot, since there is no next leg to
  // cover it.
  const bool isFinalWp = (_idx == _n - 1) && !loop;
  const float eff_accept = (isFinalWp && final_accept_radius >= 0.f)
                                ? final_accept_radius : accept_radius;

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
      if (along > 0.f && cross < corridor * eff_accept) arrived = true;
    }
  }
  // acceptance radius, optionally confirmed by speed + dwell
  if (!arrived && dist <= eff_accept) {
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
  if (dist > eff_accept && !arrived) _dwell = 0.f;

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
