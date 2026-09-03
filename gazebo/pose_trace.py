import gz.transport13 as t, time, sys, math
from gz.msgs10.pose_v_pb2 import Pose_V
n=t.Node(); cur=[None]
def cb(m):
  for p in m.pose:
    if p.name=='go1': cur[0]=(p.position.x,p.position.y,p.position.z)
n.subscribe(Pose_V,'/world/go1_world/dynamic_pose/info',cb)
t0=time.time(); start=None
while time.time()-t0 < float(sys.argv[1]):
    time.sleep(2.0)
    c=cur[0]
    if c:
        if start is None and c[2]>0.15: start=c
        d=math.hypot(c[0]-start[0],c[1]-start[1]) if start else 0
        print(f"t={time.time()-t0:4.0f}s E={c[0]:+6.2f} N={c[1]:+6.2f} z={c[2]:.3f} dist={d:5.2f}m", flush=True)
