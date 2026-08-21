"""Score one gait run from Gazebo ground truth.

Reports what actually matters for "did it walk": how far it got before it fell,
how long it stayed up, and its speed while upright. The previous version scored
a fixed window of along-heading travel and reported ~0 for a run that in fact
covered 1.58 m before tipping - measure the whole run, and say when it died.
"""
import gz.transport13 as t, math, time, sys
from gz.msgs10.pose_v_pb2 import Pose_V

FALL_ROLL = math.radians(35); FALL_Z = 0.15
n=t.Node(); r=[]
def cb(m):
  for p in m.pose:
    if p.name=='go1':
      q=p.orientation
      roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
      pit =math.asin(max(-1,min(1,2*(q.w*q.y-q.z*q.x))))
      r.append((time.time(),p.position.x,p.position.y,p.position.z,roll,pit))
n.subscribe(Pose_V,'/world/go1_world/dynamic_pose/info',cb)
time.sleep(float(sys.argv[1]))
if len(r)<40:
    print(f"{'nodata':>9} {'-':>7} {'-':>7} {'-':>7} {'-':>8}"); raise SystemExit
t0=r[0][0]
# origin = pose once the robot has stood (first sample above 0.18 m)
up=[p for p in r if p[3]>0.18]
if not up:
    print(f"{'nostand':>9} {'-':>7} {'-':>7} {'-':>7} {'-':>8}"); raise SystemExit
ox,oy=up[0][1],up[0][2]; tup=up[0][0]
fall=None
for p in up:
    if abs(p[4])>FALL_ROLL or abs(p[5])>FALL_ROLL or p[3]<FALL_Z:
        fall=p; break
end = fall if fall else r[-1]
dist = math.hypot(end[1]-ox, end[2]-oy)
dur  = max(1e-3, end[0]-tup)
zmax = max(p[3] for p in up)
print(f"{dist:9.2f} {dist/dur:7.2f} {dur:7.1f} {zmax:7.3f} {('FELL' if fall else 'upright'):>8}")
