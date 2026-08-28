#!/usr/bin/env python3
"""
terrain.py - procedural heightmap terrain for the Conductor, in place of a
real DEM ingestion pipeline.

Chuck's terrain/ (fetch_terrain.py, prepare_terrain.py, process_terrain.py)
pulls a real USGS DEM + NAIP orthophoto for one fixed site (Berkeley) - a
georeferenced heightmap for a drone flying over real ground. That doesn't
transfer here: this project has no site to reference, no network fetch
appropriate to run blind, and what the request actually asked for is terrain
VARIETY for testing ("play around with different terrain types"), not
geographic accuracy. So this generates heightmaps procedurally instead -
deterministic (seeded), fast, no external data dependency - and is honest
about being a substitute, not a port, of Chuck's pipeline.

SAFETY: every result in CLAUDE.md's campaign was measured on the flat
ground_plane. "flat" here reproduces that EXACT geometry (same plane, not a
heightmap at all) so nothing already validated is put at risk by this
existing as an option; only choosing a non-flat terrain opts into new,
unvalidated ground.
"""
import hashlib
import os
import struct
import zlib

TERRAIN_TYPES = {
    "flat":    dict(amplitude=0.0,  note="the validated ground_plane - identical geometry"),
    "rolling": dict(amplitude=0.35, note="gentle hills, ~0.35 m peak-to-trough"),
    "rough":   dict(amplitude=0.15, note="short-wavelength bumps, ~0.15 m - tests foot placement"),
    "ramp":    dict(amplitude=0.0,  note="flat ground_plane with a fixed pitch"),
    # ---- SURFACE kinds (2026-08-28, per operator: "organize the respective
    # representative types of terrain, make them selectable for a fleet run").
    # Same flat plane GEOMETRY as "flat" - so 2D waypoints sit on the ground
    # by construction - but different CONTACT PHYSICS and look. A surface's
    # mu is applied to BOTH the ground AND the four foot collisions (proto
    # ships foot mu=0.6), so the contact pair's effective friction equals
    # the surface value under any engine combine rule. mu anchors are
    # TERRAIN.md's cheat-sheet (rubber-on-concrete 0.8-1.0, URDF foot 0.6);
    # soft ground (sand/mud) additionally lowers contact stiffness kp and
    # allows a few mm of min_depth - the only mud/sand lever rigid-body gz
    # has (TERRAIN.md: "the slip IS the mud"). kp/kd/min_depth pass-through
    # on this engine is UNVERIFIED until the first A/B; mu is known-honored.
    "concrete": dict(amplitude=0.0, note="mu 0.90, rigid - the high-traction reference",
                     surface=dict(mu=0.90, kp=1e6, kd=1, min_depth=0.0,
                                   torsional=1.0, color=(0.58, 0.58, 0.58))),
    "asphalt":  dict(amplitude=0.0, note="mu 0.85, rigid",
                     surface=dict(mu=0.85, kp=1e6, kd=1, min_depth=0.0,
                                   torsional=1.0, color=(0.25, 0.25, 0.28))),
    "grass":    dict(amplitude=0.0, note="mu 0.60, slightly compliant",
                     surface=dict(mu=0.60, kp=4e5, kd=20, min_depth=0.001,
                                   torsional=0.8, color=(0.30, 0.52, 0.24))),
    "dirt":     dict(amplitude=0.0, note="mu 0.70, packed",
                     surface=dict(mu=0.70, kp=3e5, kd=30, min_depth=0.001,
                                   torsional=0.8, color=(0.45, 0.36, 0.25))),
    "gravel":   dict(amplitude=0.0, note="mu 0.55, low torsional grip",
                     surface=dict(mu=0.55, kp=2e5, kd=40, min_depth=0.002,
                                   torsional=0.3, color=(0.48, 0.45, 0.42))),
    "sand":     dict(amplitude=0.0, note="mu 0.45, soft, feet auger (low torsional)",
                     surface=dict(mu=0.45, kp=8e4, kd=100, min_depth=0.003,
                                   torsional=0.15, color=(0.76, 0.68, 0.50))),
    "mud":      dict(amplitude=0.0, note="mu 0.35, softest - the slip IS the mud",
                     surface=dict(mu=0.35, kp=4e4, kd=200, min_depth=0.006,
                                   torsional=0.2, color=(0.30, 0.24, 0.18))),
    "rock":     dict(amplitude=0.0, note="mu 0.80, rigid slab (uneven rock = later phase)",
                     surface=dict(mu=0.80, kp=1e6, kd=1, min_depth=0.0,
                                   torsional=0.9, color=(0.44, 0.42, 0.40))),
    "ice":      dict(amplitude=0.0, note="mu 0.15 - deliberately past any gait's envelope; adhesion diagnostic",
                     surface=dict(mu=0.15, kp=1e6, kd=1, min_depth=0.0,
                                   torsional=0.05, color=(0.72, 0.80, 0.88))),
}


def surface_xml(spec):
    """The <surface> block for the GROUND collision of a surface kind."""
    s = spec["surface"]
    torsional = ("<torsional><coefficient>%g</coefficient></torsional>"
                 % s["torsional"])
    return ("<surface>"
            "<contact><ode><kp>%g</kp><kd>%g</kd><min_depth>%g</min_depth>"
            "</ode></contact>"
            "<friction><ode><mu>%g</mu><mu2>%g</mu2></ode>%s</friction>"
            "</surface>"
            % (s["kp"], s["kd"], s["min_depth"], s["mu"], s["mu"], torsional))

SIZE_M = 400.0     # matches the existing ground_plane's <size>
GRID = 129         # 2^7+1 - the conventional heightmap dimension for terrain engines


def _png(path, w, h, gray_rows):
    """Write an 8-bit grayscale PNG with no external dependency (this venv
    has PIL, but a hand-rolled writer keeps terrain.py usable from plain
    python3 too, for anyone regenerating a heightmap outside the Conductor)."""
    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c))
    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)
    raw = bytearray()
    for row in gray_rows:
        raw.append(0)  # filter type 0 per scanline
        raw.extend(row)
    idat = zlib.compress(bytes(raw), 9)
    with open(path, "wb") as f:
        f.write(sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(b"IEND", b""))


def generate(kind, out_path, seed_text="", flatten_radius_m=2.0):
    """Write a GRID x GRID grayscale heightmap PNG for `kind` to `out_path`.
    Returns the amplitude (metres) the heightmap's white level represents,
    for the SDF <size> Z component. Deterministic: same kind+seed_text always
    produces the same terrain, so a test can be repeated exactly.

    `flatten_radius_m` sizes the level disc at the heightmap's centre (world
    origin). Caller must size it to just cover the disc's actual purpose -
    letting every dog spawn level, nothing more - because a mission's own
    footprint typically sits on the same 10-40 m scale a generous default
    would need, so an oversized disc quietly reproduces flat ground under the
    whole course. See fleet_world.apply_terrain, which sizes this from the
    real spawn offsets instead of guessing here."""
    import numpy as np
    spec = TERRAIN_TYPES.get(kind, TERRAIN_TYPES["flat"])
    amp = spec["amplitude"]
    if amp <= 0.0:
        _png(out_path, GRID, GRID, [bytes([128] * GRID) for _ in range(GRID)])
        return max(amp, 0.01)  # SDF heightmap Z size must be > 0

    seed = int(hashlib.sha1((kind + seed_text).encode()).hexdigest()[:8], 16)
    rng = np.random.default_rng(seed)
    x = np.linspace(0, 1, GRID)
    y = np.linspace(0, 1, GRID)
    X, Y = np.meshgrid(x, y)
    z = np.zeros_like(X)

    if kind == "rolling":
        # A handful of random low-frequency sine components - smooth hills,
        # nothing sharper than the robot's own footprint needs to negotiate.
        for _ in range(5):
            fx, fy = rng.uniform(1, 3, 2)
            phase = rng.uniform(0, 6.283)
            amp_k = rng.uniform(0.4, 1.0)
            z += amp_k * np.sin(2 * np.pi * fx * X + phase) * np.cos(2 * np.pi * fy * Y)
        z /= max(1e-6, np.abs(z).max())
    elif kind == "rough":
        # High-frequency components dominate - short bumps, small amplitude,
        # meant to exercise foot placement rather than balance on a slope.
        for _ in range(30):
            fx, fy = rng.uniform(6, 14, 2)
            phase = rng.uniform(0, 6.283)
            amp_k = rng.uniform(0.2, 1.0) / fx
            z += amp_k * np.sin(2 * np.pi * fx * X + phase) * np.sin(2 * np.pi * fy * Y)
        z /= max(1e-6, np.abs(z).max())
    # Flatten a margin at the centre so every dog's spawn point is level - a
    # robot must not spawn already tilted. Sized in real metres by the caller
    # (not a fixed fraction of the grid), because the caller knows how far the
    # farthest spawn point actually sits from world origin.
    cx, cy = GRID // 2, GRID // 2
    px_spacing = SIZE_M / (GRID - 1)
    r = max(1, int(round(flatten_radius_m / px_spacing)))
    yy, xx = np.ogrid[:GRID, :GRID]
    mask = ((xx - cx) ** 2 + (yy - cy) ** 2) <= r * r
    falloff = np.clip(((xx - cx) ** 2 + (yy - cy) ** 2) / (r * r), 0, 1)
    z = np.where(mask, 0.0, z * falloff)

    gray = np.clip((z * 0.5 + 0.5) * 255, 0, 255).astype("uint8")
    _png(out_path, GRID, GRID, [gray[i].tobytes() for i in range(GRID)])
    return amp


def build_heightmap_xml(png_path, amplitude_m, textured=False):
    """The <heightmap> geometry block, sized to match the existing flat
    ground_plane's footprint (SIZE_M x SIZE_M) so nothing about mission
    placement (spawn offsets, lane spacing) has to change to use it.

    `textured=False` (used for the COLLISION copy, which is never rendered)
    omits <texture> - Gazebo then has nothing to shade, so the geometry
    cannot trip a shader-compile failure. `textured=True` (the VISUAL copy)
    reuses the heightmap's own grayscale PNG as a single diffuse layer -
    deliberately the simplest possible material, no network fetch, one
    texture instead of Gazebo's default multi-layer auto-blend. That default
    blend is what was measured to crash gz-sim's Ogre2/Metal renderer the
    moment a camera sensor tried to render the terrain (Fragment Program
    ..._PixelShader_ps failed to compile, inside CameraSensor::Update); a
    single explicit layer uses a far simpler shader permutation."""
    tex = ""
    if textured:
        tex = (
            '<texture><diffuse>file://%s</diffuse>'
            '<normal>file://%s</normal><size>10</size></texture>'
        ) % (png_path, png_path)
    return (
        '<heightmap><uri>file://%s</uri>'
        '<size>%g %g %g</size>'
        '<pos>0 0 0</pos>%s</heightmap>'
    ) % (png_path, SIZE_M, SIZE_M, max(amplitude_m * 2.0, 0.02), tex)


if __name__ == "__main__":
    import sys
    kind = sys.argv[1] if len(sys.argv) > 1 else "rolling"
    out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/terrain_%s.png" % kind
    amp = generate(kind, out)
    print("wrote", out, "amplitude", amp)
