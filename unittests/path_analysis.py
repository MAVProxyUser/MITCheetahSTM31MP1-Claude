#!/usr/bin/env python3
"""
Quantitative planned-vs-flown path deviation analysis: corner rounding and
hairpinning, measured from the actual coordinate data (state["planned"] /
state["positions"][i]["trail"], world x=east/y=north metres) rather than
from a screenshot. No browser needed - this is the same reasoning
mission_runner.py's own render_report() already uses for its PNG plot
("a description built by re-deriving the geometry could silently diverge
from what the operator saw"): the numeric arrays ARE what gets drawn, on
the panel's canvas and in the matplotlib plot alike, so analyzing them
directly is strictly more precise than doing image/pixel processing on a
rendering of the same numbers - no edge-detection noise, no scale/DPI
ambiguity, exact metre-level distances instead of pixels.

DEFINITIONS
  closest_approach_m   how close the flown trail ever got to the planned
                       vertex. Small = the dog nearly touched the corner.
                       Large = the corner was cut/rounded well inside the
                       planned corridor - EXPECTED for a filleted corner,
                       only worth flagging if it's larger than the
                       mission's own configured acceptance radius.
  overshoot_m          how far PAST the vertex (measured along the outgoing
                       leg's own direction) the trail traveled before
                       having to turn back toward it - the geometric
                       signature of "hairpinning" / the "elephant foot"
                       pattern this project's own CLAUDE.md documents at
                       length. Zero means no backtracking was ever needed;
                       a real positive number means the follower overshot
                       and had to recover.
"""
import math


def _vertices_from_planned(planned):
    """planned is a closed polygon (app.js/mission_runner.py's own
    convention - the last point implicitly connects back to the first).
    Returns (vertex, incoming_unit, outgoing_unit) for every vertex except
    the first (index 0 is where the dog spawns/starts - not a corner the
    follower has to negotiate, there is no "before" leg for it)."""
    n = len(planned)
    out = []
    for k in range(1, n):
        prev_pt = planned[k - 1]
        cur = planned[k]
        next_pt = planned[(k + 1) % n]
        inc = (cur[0] - prev_pt[0], cur[1] - prev_pt[1])
        out_v = (next_pt[0] - cur[0], next_pt[1] - cur[1])
        inc_len = math.hypot(*inc)
        out_len = math.hypot(*out_v)
        if inc_len < 1e-6 or out_len < 1e-6:
            continue  # degenerate leg (duplicate point) - nothing to analyze
        inc_u = (inc[0] / inc_len, inc[1] / inc_len)
        out_u = (out_v[0] / out_len, out_v[1] / out_len)
        out.append((cur, inc_u, out_u))
    return out


def analyze_corner(vertex, outgoing_unit, trail, search_radius_m=5.0):
    """One vertex's closest-approach and overshoot, from the flown trail."""
    vx, vy = vertex
    ux, uy = outgoing_unit

    best_d = float("inf")
    best_idx = None
    for idx, (px, py) in enumerate(trail):
        d = math.hypot(px - vx, py - vy)
        if d < best_d:
            best_d = d
            best_idx = idx

    if best_idx is None:
        return {"closest_approach_m": None, "overshoot_m": 0.0, "note": "no trail data"}

    # Overshoot: project trail points AFTER the closest approach onto the
    # outgoing leg's own axis (0 = at the vertex, positive = further along
    # toward the next waypoint). A follower that overshoots and corrects
    # shows this projection rise, dip back down (sometimes negative - past
    # the vertex on the WRONG side, doubling back over ground already
    # covered), then rise again toward the next waypoint. The overshoot
    # magnitude is that dip's depth below the running peak, restricted to a
    # local search window so a full lap's worth of later, unrelated motion
    # cannot be mistaken for backtracking at THIS corner.
    peak = -float("inf")
    max_backtrack = 0.0
    for (px, py) in trail[best_idx:]:
        d_from_vertex = math.hypot(px - vx, py - vy)
        if d_from_vertex > search_radius_m and peak > 0:
            break  # left the local neighbourhood after making some progress - done
        proj = (px - vx) * ux + (py - vy) * uy
        if proj > peak:
            peak = proj
        elif peak > 0:
            backtrack = peak - proj
            if backtrack > max_backtrack:
                max_backtrack = backtrack

    return {"closest_approach_m": round(best_d, 3), "overshoot_m": round(max_backtrack, 3)}


def analyze_mission(planned, trail, accept_radius_m=1.0):
    """Full per-corner breakdown for one dog's run. Returns a list of dicts,
    one per corner, plus a summary classification per corner:
      "clean"    - closest approach inside accept_radius, no overshoot
      "rounded"  - closest approach OUTSIDE accept_radius, no overshoot
                   (planner cut the corner wider than configured - only
                   interesting if this is unexpectedly large)
      "hairpin"  - real overshoot detected (> 0.05 m, above trail-noise
                   floor) - the follower had to double back
    """
    if not planned or len(planned) < 2 or not trail or len(trail) < 2:
        return []
    vertices = _vertices_from_planned(planned)
    results = []
    for i, (vtx, inc_u, out_u) in enumerate(vertices):
        r = analyze_corner(vtx, out_u, trail)
        if r.get("closest_approach_m") is None:
            r["classification"] = "no data"
        elif r["overshoot_m"] > 0.05:
            r["classification"] = "hairpin"
        elif r["closest_approach_m"] > accept_radius_m * 1.5:
            r["classification"] = "rounded"
        else:
            r["classification"] = "clean"
        r["corner_index"] = i + 1  # matches waypoint numbering (vertex 0 excluded)
        r["vertex"] = vtx
        results.append(r)
    return results


def format_report(results, dog_label=""):
    if not results:
        return "  (no corners to analyze)"
    lines = []
    for r in results:
        tag = r["classification"].upper()
        if r.get("closest_approach_m") is None:
            lines.append("  corner %d: no data" % r["corner_index"])
            continue
        lines.append(
            "  corner %d: closest=%.2fm overshoot=%.2fm -> %s"
            % (r["corner_index"], r["closest_approach_m"], r["overshoot_m"], tag)
        )
    n_hairpin = sum(1 for r in results if r.get("classification") == "hairpin")
    n_rounded = sum(1 for r in results if r.get("classification") == "rounded")
    if n_hairpin:
        lines.append("  -> %d/%d corners show HAIRPINNING (overshoot+recovery)" % (n_hairpin, len(results)))
    if n_rounded:
        lines.append("  -> %d/%d corners cut wider than 1.5x accept radius" % (n_rounded, len(results)))
    return "\n".join(lines)


if __name__ == "__main__":
    # Tiny self-check: a square course. Vertex 1 = (10,0), outgoing axis
    # +y (toward (10,10)). "Clean" approaches (10,0) and turns straight
    # onto +y. "Hairpin" approaches (10,0), overshoots to y=14 (4m past
    # where it needs to end up relative to the vertex), then corrects back
    # down to y=8 before continuing - exactly the overshoot-and-recover
    # shape this function exists to catch.
    planned = [(0, 0), (10, 0), (10, 10), (0, 10)]
    clean_trail = [(x * 0.5, 0.0) for x in range(21)] + \
                  [(10.0, y * 0.5) for y in range(21)] + \
                  [(10 - x * 0.5, 10.0) for x in range(21)]
    hairpin_trail = [(x * 0.5, 0.0) for x in range(21)] + \
                    [(10.0, y * 1.0) for y in range(15)] + \
                    [(10.0, 14.0 - y * 1.0) for y in range(1, 7)] + \
                    [(10 - x * 0.5, 10.0) for x in range(21)]
    print("=== clean trail ===")
    print(format_report(analyze_mission(planned, clean_trail)))
    print("=== hairpin trail (expect corner 1 to flag ~4m overshoot) ===")
    print(format_report(analyze_mission(planned, hairpin_trail)))
