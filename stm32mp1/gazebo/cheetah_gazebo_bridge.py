#!/usr/bin/env python3
"""
cheetah_gazebo_bridge.py - bridge a Gazebo (gz-harmonic) Go1 to the Cheetah
controller (jpos_ctrl_sim), same pattern as the OpenPilot gazebo_bridge.

  Gazebo  ── /imu, /.../joint_state ──►  bridge  ── UDP sensor ──►  controller
          ◄── /.../cmd_force ×12 ──────  (motor PD)  ◄── UDP command ──┘

The bridge runs the Unitree motor law itself: it receives the controller's per-joint
impedance command (q_des, qd_des, kp, kd, tau_ff) and applies
    tau = kp*(q_des - q) + kd*(qd_des - qd) + tau_ff
to each Go1 joint via cmd_force - exactly how the real motor behaves.

Run (needs gz python bindings):
  GZ_SIM_RESOURCE_PATH=$PWD/unitree_ros/robots \
  <python-with-gz> cheetah_gazebo_bridge.py [controller_ip]

Joints, flat Cheetah order (index = leg*3 + joint), leg 0..3 = FR,FL,RR,RL,
joint 0=abad(hip),1=hip(thigh),2=knee(calf).
"""
import os, sys, socket, struct, threading, time

import gz.transport13 as transport
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.model_pb2 import Model
from gz.msgs10.double_pb2 import Double
from gz.msgs10.fluid_pressure_pb2 import FluidPressure
from gz.msgs10.navsat_pb2 import NavSat

WORLD = "go1_world"
MODEL = "go1"
JOINTS = [f"{leg}_{j}_joint" for leg in ("FR", "FL", "RR", "RL")
          for j in ("hip", "thigh", "calf")]           # flat order

# Go1 <-> Cheetah joint convention: q_cheetah = SIGN*q_gz + OFFSET (rad),
# applied symmetrically to q (sensor), qd, the PD error, and tau (command).
#
#  BRIDGE_CONV=identity (default): SIGN=[1..], for the joint-space Stand/Walk
#    controllers, which are written directly in Go1 URDF angles.
#
#  BRIDGE_CONV=mit: SIGN=[1,-1,-1] per leg. The MIT_Controller works in the
#    Cheetah *abstract* leg convention (buildGo1 keeps robotType=MINI_CHEETAH).
#    Matching the abstract forward kinematics in LegController.cpp
#    (p = f(abad,hip,knee), l1=abadLink, l2=hipLink, l3=kneeLink) against the
#    Go1 URDF chain (abad about +x, thigh/calf about +y, same link lengths)
#    gives EXACTLY: abad_abstract = abad_go1, hip_abstract = -hip_go1,
#    knee_abstract = -knee_go1, all offsets 0 (both have q=0 => leg straight
#    down). Without this the MIT stand/MPC drives the Go1 hips/knees the wrong
#    way and it rolls over the instant it leaves the (abad~0) stand.
if os.environ.get("BRIDGE_CONV", "identity").lower() == "mit":
    SIGN   = [1.0, -1.0, -1.0] * 4
    OFFSET = [0.0] * 12
    print("[bridge] joint convention: MIT abstract (hip & knee negated)", flush=True)
else:
    SIGN   = [1.0] * 12
    OFFSET = [0.0] * 12
    print("[bridge] joint convention: identity (Go1 URDF angles)", flush=True)

IMU_TOPIC   = "/imu"
BARO_TOPIC  = "/air_pressure"
GPS_TOPIC   = "/navsat"
JOINT_TOPIC = f"/world/{WORLD}/model/{MODEL}/joint_state"
FORCE_TOPIC = lambda jn: f"/model/{MODEL}/joint/{jn}/cmd_force"

CMD_PORT    = 9100     # bridge receives controller commands here
SENSOR_PORT = 9101     # controller receives sensor packets here
CTRL_IP     = sys.argv[1] if len(sys.argv) > 1 else None   # learned from first cmd if None

SENSOR_MAGIC  = 0x53454E53   # 'SENS'
COMMAND_MAGIC = 0x434D4443   # 'CMDC'
# trailing 3f 4f 3f = sim ground truth pos/quat/vWorld (cheater mode)
SENSOR_FMT  = "<II 3f 3f 4f 12f 12f f f d d f 3f 3f 4f 3f"
COMMAND_FMT = "<II 12f 12f 12f 12f 12f"

# ---- shared state ----
lock = threading.Lock()
imu = {"accel": [0,0,0], "gyro": [0,0,0], "quat": [0,0,0,1]}      # quat x,y,z,w
baro = {"pressure": 101325.0, "alt": 0.0}
gps  = {"lat": 0.0, "lon": 0.0, "alt": 0.0, "vel": [0.0,0.0,0.0]}  # vel NED
qj  = [0.0]*12    # joint pos, Go1 frame
qdj = [0.0]*12
# sim ground truth (cheater mode): world pose + finite-diff world velocity
truth = {"pos": [0.0,0.0,0.0], "quat": [0.0,0.0,0.0,1.0], "vworld": [0.0,0.0,0.0],
         "t": None}
cmd = {"q_des":[0.0]*12, "qd_des":[0.0]*12, "kp":[0.0]*12, "kd":[0.0]*12, "tau_ff":[0.0]*12}
last_cmd_t = [0.0]          # wall time of the most recent controller packet
CMD_TIMEOUT = float(os.environ.get("BRIDGE_CMD_TIMEOUT", "0.25"))   # s
peer_ip = [CTRL_IP]
seq_out = [0]
cmd_rx  = [0]   # commands received (for heartbeat)
last_tau = [0.0]*12
_wd_said = [False]

node = transport.Node()

# Gazebo's IMU reports the sensor orientation as body->world; MIT's estimator
# wants the quaternion whose rotation matrix is world->body (vBody = R*vWorld).
# QUAT_CONJ=1 sends the conjugate [-x,-y,-z,w] to match MIT's convention.
QUAT_CONJ = os.environ.get("QUAT_CONJ", "0") == "1"
if QUAT_CONJ:
    print("[bridge] IMU quaternion: conjugated (body->world => world->body)", flush=True)

def on_imu(msg: IMU):
    with lock:
        imu["accel"] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        imu["gyro"]  = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        imu["quat"] = [-q[0], -q[1], -q[2], q[3]] if QUAT_CONJ else q

def on_baro(msg: FluidPressure):
    with lock:
        baro["pressure"] = msg.pressure
        # standard-atmosphere altitude from pressure
        try:
            baro["alt"] = 44330.0 * (1.0 - (msg.pressure / 101325.0) ** 0.190295)
        except Exception:
            baro["alt"] = 0.0

def on_navsat(msg: NavSat):
    with lock:
        gps["lat"] = msg.latitude_deg
        gps["lon"] = msg.longitude_deg
        gps["alt"] = msg.altitude
        # gz NavSat velocity is ENU-ish (east/north/up); convert to NED
        gps["vel"] = [msg.velocity_north, msg.velocity_east, -msg.velocity_up]

def on_pose(msg):
    """Ground truth from /world/.../dynamic_pose/info (published at physics rate).
    World velocity by finite difference, lightly low-passed (alpha=0.25 @1kHz
    ~ 45Hz bandwidth) so cheater-mode vBody is usable by the WBC."""
    now = time.monotonic()
    for p in msg.pose:
        if p.name != MODEL:
            continue
        with lock:
            tprev, pprev = truth["t"], truth["pos"]
            pos = [p.position.x, p.position.y, p.position.z]
            if tprev is not None:
                dt = now - tprev
                if 1e-4 < dt < 0.1:
                    a = 0.25
                    for i in range(3):
                        v_raw = (pos[i] - pprev[i]) / dt
                        truth["vworld"][i] += a * (v_raw - truth["vworld"][i])
            truth["pos"] = pos
            truth["quat"] = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            truth["t"] = now
        return

def on_joint(msg: Model):
    idx = {jn: i for i, jn in enumerate(JOINTS)}
    with lock:
        for j in msg.joint:
            i = idx.get(j.name)
            if i is None:
                continue
            # gz JointStatePublisher puts pos/vel in axis1
            qj[i]  = j.axis1.position
            qdj[i] = j.axis1.velocity

# per-joint force publishers
force_pub = {jn: node.advertise(FORCE_TOPIC(jn), Double) for jn in JOINTS}

def apply_torque(tau_gz):
    for i, jn in enumerate(JOINTS):
        d = Double(); d.data = float(tau_gz[i])
        force_pub[jn].publish(d)

# ---- UDP ----
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", CMD_PORT))
sock.settimeout(0.1)

def udp_rx():
    while True:
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        if len(data) != struct.calcsize(COMMAND_FMT):
            continue
        vals = struct.unpack(COMMAND_FMT, data)
        if vals[0] != COMMAND_MAGIC:
            continue
        if peer_ip[0] is None:
            peer_ip[0] = addr[0]
            print(f"[bridge] controller at {addr[0]}", flush=True)
        cmd_rx[0] += 1
        last_cmd_t[0] = time.time()
        with lock:
            cmd["q_des"]  = list(vals[2:14])
            cmd["qd_des"] = list(vals[14:26])
            cmd["kp"]     = list(vals[26:38])
            cmd["kd"]     = list(vals[38:50])
            cmd["tau_ff"] = list(vals[50:62])

threading.Thread(target=udp_rx, daemon=True).start()

def send_sensor():
    with lock:
        # Go1 -> Cheetah frame
        qc  = [SIGN[i]*qj[i]  + OFFSET[i] for i in range(12)]
        qdc = [SIGN[i]*qdj[i]             for i in range(12)]
        a, g, quat = imu["accel"], imu["gyro"], imu["quat"]
        ba, bp = baro["alt"], baro["pressure"]
        glat, glon, galt, gvel = gps["lat"], gps["lon"], gps["alt"], gps["vel"]
        tpos, tquat, tvw = list(truth["pos"]), list(truth["quat"]), list(truth["vworld"])
    if peer_ip[0] is None:
        return
    seq_out[0] += 1
    pkt = struct.pack(SENSOR_FMT, SENSOR_MAGIC, seq_out[0],
                      *a, *g, *quat, *qc, *qdc,
                      ba, bp, glat, glon, galt, *gvel,
                      *tpos, *tquat, *tvw)
    try:
        sock.sendto(pkt, (peer_ip[0], SENSOR_PORT))
    except OSError:
        pass   # controller restarting / gone; keep the bridge alive

def control_step():
    # WATCHDOG. If the controller stops talking (crash, kill, unplugged cable)
    # the last command must NOT be replayed for ever: a controller killed
    # mid-swing leaves two diagonal feet commanded into the air and the robot
    # simply tips over, which is exactly how a completed mission ended with the
    # dog on its side. Real hardware is worse - stale torques against a fallen
    # machine cook motors. So: fade the stiffness out and let it settle.
    if last_cmd_t[0] > 0.0:
        age = time.time() - last_cmd_t[0]
        if age > CMD_TIMEOUT:
            with lock:
                fade = max(0.0, 1.0 - (age - CMD_TIMEOUT) / 0.5)   # 0.5 s ramp
                for i in range(12):
                    cmd["kp"][i] *= fade
                    cmd["tau_ff"][i] *= fade
                    cmd["qd_des"][i] = 0.0
                    cmd["kd"][i] = min(cmd["kd"][i], 0.5) if fade > 0 else 0.4
            if not _wd_said[0]:
                _wd_said[0] = True
                print(f"[bridge] WATCHDOG: no controller command for {age:.2f}s "
                      f"- fading stiffness to a safe limp", flush=True)
        elif _wd_said[0]:
            _wd_said[0] = False
            print("[bridge] controller is back", flush=True)
    with lock:
        # PD in Cheetah frame, then convert torque back to Go1 frame
        qc  = [SIGN[i]*qj[i] + OFFSET[i] for i in range(12)]
        qdc = [SIGN[i]*qdj[i]            for i in range(12)]
        tau_gz = [0.0]*12
        for i in range(12):
            tau_c = (cmd["kp"][i]*(cmd["q_des"][i]-qc[i])
                     + cmd["kd"][i]*(cmd["qd_des"][i]-qdc[i])
                     + cmd["tau_ff"][i])
            tau_gz[i] = SIGN[i]*tau_c
        last_tau[:] = tau_gz
    apply_torque(tau_gz)

def main():
    assert node.subscribe(IMU, IMU_TOPIC, on_imu), "IMU subscribe failed"
    assert node.subscribe(Model, JOINT_TOPIC, on_joint), "joint_state subscribe failed"
    node.subscribe(FluidPressure, BARO_TOPIC, on_baro)
    node.subscribe(NavSat, GPS_TOPIC, on_navsat)
    from gz.msgs10.pose_v_pb2 import Pose_V
    node.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info", on_pose)
    print(f"[bridge] subscribed {IMU_TOPIC} {BARO_TOPIC} {GPS_TOPIC} {JOINT_TOPIC}; {len(JOINTS)} force pubs")
    print(f"[bridge] UDP: recv cmd :{CMD_PORT}, send sensors :{SENSOR_PORT} -> {peer_ip[0] or '(learn)'}")
    period = 1.0/500.0    # 500 Hz
    last = time.time()
    hb = time.time()
    stalls = [0, 0.0]    # count >5ms, worst gap (python GIL/callback jitter diagnostics)
    prev = time.time()
    while True:
        control_step()
        send_sensor()
        now = time.time()
        gap = now - prev
        prev = now
        if gap > 0.005:
            stalls[0] += 1
            if gap > stalls[1]: stalls[1] = gap
        if now - hb >= 1.0:
            with lock:
                q0 = round(qj[0], 3); t0 = round(last_tau[0], 2); t2 = round(last_tau[2], 2)
                bp = round(baro["pressure"], 0); ba = round(baro["alt"], 2)
                la = round(gps["lat"], 5); lo = round(gps["lon"], 5)
            print(f"[bridge] cmd_rx={cmd_rx[0]}/s peer={peer_ip[0]} imu_az={round(imu['accel'][2],2)} "
                  f"q_FR_hip={q0} tau=[{t0},{t2}] stalls>{5}ms={stalls[0]}/worst={round(stalls[1]*1000,1)}ms "
                  f"baro={bp}Pa/{ba}m gps=({la},{lo})", flush=True)
            cmd_rx[0] = 0; hb = now; stalls[0] = 0; stalls[1] = 0.0
        last += period
        dt = last - now
        if dt > 0: time.sleep(dt)
        else: last = now

if __name__ == "__main__":
    main()
