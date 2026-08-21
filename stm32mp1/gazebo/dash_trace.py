#!/usr/bin/env python3
"""Time a 100 m dash from Gazebo ground truth.

  dash_trace.py <max_seconds> [target_m]

Samples pose at 10 Hz (the 2 s pose_trace is far too coarse to time a dash) and
exits the moment the robot either crosses the line or goes down, so a failed run
costs seconds instead of the whole timeout.

Reports two different numbers on purpose:
  t100   elapsed from first motion to the line - INCLUDES the velocity ramp, so
         it is what the robot actually takes to cover the ground;
  v_fly  speed between 20 m and the line - EXCLUDES the ramp, so it is the
         gait's sustained cruise. A gentle ramp helps t100 less than it helps
         survival, and these two numbers keep that visible.
"""
import math
import sys
import time

import gz.transport13 as transport
from gz.msgs10.pose_v_pb2 import Pose_V

MAX_S = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
TARGET = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0

node = transport.Node()
cur = [None]


def cb(msg):
    for p in msg.pose:
        if p.name == 'go1':
            cur[0] = (p.position.x, p.position.y, p.position.z)


node.subscribe(Pose_V, '/world/go1_world/dynamic_pose/info', cb)

t0 = time.time()
origin = None          # pose at first stand
moving_t = None        # wall time the robot first travelled 1 m
mark20 = None          # (t, dist) crossing 20 m
best = 0.0
down_since = None
stood = False
result = dict(reached=0, t100=0.0, v_fly=0.0, maxdist=0.0, fell=0)

while time.time() - t0 < MAX_S:
    time.sleep(0.1)
    c = cur[0]
    if c is None:
        continue
    now = time.time()

    if c[2] > 0.22:
        stood = True
    if origin is None and c[2] > 0.15:
        origin = c

    if origin is None:
        continue

    d = math.hypot(c[0] - origin[0], c[1] - origin[1])
    best = max(best, d)

    if moving_t is None and d >= 1.0:
        moving_t = now
    if mark20 is None and d >= 20.0:
        mark20 = (now, d)

    if d >= TARGET:
        result['reached'] = 1
        result['t100'] = now - moving_t if moving_t else 0.0
        if mark20:
            dt = now - mark20[0]
            result['v_fly'] = (d - mark20[1]) / dt if dt > 0 else 0.0
        break

    # down: sustained low body height once it has actually stood
    if stood and c[2] < 0.15:
        down_since = down_since or now
        if now - down_since > 1.0:
            result['fell'] = 1
            break
    else:
        down_since = None

result['maxdist'] = best
print('RESULT reached={reached} t100={t100:.1f} v_fly={v_fly:.2f} '
      'maxdist={maxdist:.1f} fell={fell}'.format(**result), flush=True)
