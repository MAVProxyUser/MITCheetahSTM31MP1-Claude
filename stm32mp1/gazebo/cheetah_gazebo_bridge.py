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
import os, sys, socket, struct, subprocess, signal, threading, time

import gz.transport13 as transport
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.model_pb2 import Model
from gz.msgs10.double_pb2 import Double
from gz.msgs10.fluid_pressure_pb2 import FluidPressure
from gz.msgs10.navsat_pb2 import NavSat

WORLD = os.environ.get("SIM_WORLD", "go1_world")
# $SIM_MODEL selects WHICH dog this bridge drives when several share one
# physics engine (make_multi_world.py emits go1_0, go1_1, ...). Everything
# scoped by model name - joint_state, cmd_force, the pose filter - follows from
# this automatically; only the raw sensor topics needed namespacing in the SDF.
MODEL = os.environ.get("SIM_MODEL", "go1")
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

# Bare topics when driving the single-dog world; namespaced under the model
# name when several dogs share one engine.
_NS         = "" if MODEL == "go1" else "/" + MODEL
IMU_TOPIC   = _NS + "/imu"
BARO_TOPIC  = _NS + "/air_pressure"
GPS_TOPIC   = _NS + "/navsat"
JOINT_TOPIC = f"/world/{WORLD}/model/{MODEL}/joint_state"
FORCE_TOPIC = lambda jn: f"/model/{MODEL}/joint/{jn}/cmd_force"

# $SIM_INSTANCE shifts the port pair so several dogs can run on one machine:
# 0 -> 9100/9101, 1 -> 9110/9111, 2 -> 9120/9121. Must match the controller's
# own SIM_INSTANCE. Gazebo transport needs GZ_PARTITION set to match as well.
_INST       = int(os.environ.get("SIM_INSTANCE", "0"))
CMD_PORT    = 9100 + 10 * _INST   # bridge receives controller commands here
SENSOR_PORT = 9101 + 10 * _INST   # controller receives sensor packets here
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
# MICRO-STALENESS GUARD for tau_ff, separate from (and much faster than)
# CMD_TIMEOUT above - see its comment in control_step() for the mechanism.
# 8ms matches RobotRunner.cpp's own SIM_STALL_MS default deliberately, not
# by coincidence: that threshold was chosen there specifically because a
# single 4-5ms tick is ordinary macOS scheduler jitter (this machine never
# exceeds ~24% CPU and still produces them) and reacting to one was
# measured to do more harm than the jitter itself. Reusing the same
# number here means this guard only ever engages on the same events the
# C++ side already calls genuinely abnormal, not on routine noise.
TAU_FF_STALE_S = float(os.environ.get("BRIDGE_TAU_FF_STALE_MS", "8.0")) / 1000.0
TAU_FF_RAMP_S  = float(os.environ.get("BRIDGE_TAU_FF_RAMP_MS", "15.0")) / 1000.0
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
            _ready["pose"] = True
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
        _ready["joints"] = True

# per-joint force publishers
force_pub = {jn: node.advertise(FORCE_TOPIC(jn), Double) for jn in JOINTS}

def apply_torque(tau_gz):
    for i, jn in enumerate(JOINTS):
        d = Double(); d.data = float(tau_gz[i])
        force_pub[jn].publish(d)

# ---- UDP ----
def _clear_stale_port(port):
    """A bridge left running from an earlier manual test (no gz sim behind
    it any more) can still hold this exact UDP port - no SO_REUSEADDR is set
    below, on purpose, so a stale occupant is detected rather than silently
    shared. Once it bit a whole speed-ladder sweep: the fresh bridge below
    never got a chance to bind, this port's OLD owner kept answering with
    frozen/stale sensor data, and every run in the sweep looked like an
    identical, reproducible physics failure until the stale pid was found by
    hand. This process starting up is authoritative for this port - anything
    already on it is leftover, never a peer to share with - so find and kill
    it before we bind."""
    try:
        out = subprocess.run(["lsof", "-ti", "udp:%d" % port],
                              capture_output=True, text=True, timeout=5).stdout
        pids = {int(p) for p in out.split()} - {os.getpid()}
    except Exception as e:  # noqa: BLE001 - lsof missing/slow must not block startup
        print("[bridge] stale-port check on %d failed (%s) - continuing" % (port, e), flush=True)
        return
    for pid in pids:
        print("[bridge] port %d already held by stale pid %d - killing it" % (port, pid), flush=True)
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if pids:
        time.sleep(0.3)  # let the kernel release the port before we bind it

_clear_stale_port(CMD_PORT)
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
            if _dump_f:
                _dump_n[0] += 1
                if (_dump_n[0] % 5) == 0:
                    row = [f"{time.time():.3f}"]
                    row += [f"{v:.4f}" for v in qj]
                    row += [f"{v:.3f}" for v in cmd["qd_des"]]
                    row += [f"{v:.4f}" for v in cmd["q_des"]]
                    row += [f"{v:.1f}" for v in cmd["kp"]]
                    row += [f"{v:.2f}" for v in cmd["kd"]]
                    row += [f"{v:.2f}" for v in cmd["tau_ff"]]
                    _dump_f.write(",".join(row) + "\n")

# BRIDGE_DUMP=<path>: record every 5th command packet (100 Hz) with the joint
# state at arrival - q, q_des, kp, kd, tau_ff for all 12 joints. Mac-side and
# free: board-side instrumentation measurably destabilises the controller
# (printf stalls on the FIFO control thread), which made every instrumented
# run lie. The bridge sees the entire command stream anyway.
_dump_path = os.environ.get("BRIDGE_DUMP")
_dump_f = open(_dump_path, "w") if _dump_path else None
_dump_n = [0]
if _dump_f:
    _dump_f.write("t," + ",".join(f"q{i}" for i in range(12)) + ","
                  + ",".join(f"qd_des{i}" for i in range(12)) + ","
                  + ",".join(f"qdes{i}" for i in range(12)) + ","
                  + ",".join(f"kp{i}" for i in range(12)) + ","
                  + ",".join(f"kd{i}" for i in range(12)) + ","
                  + ",".join(f"tff{i}" for i in range(12)) + "\n")

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

_ready = {"joints": False, "pose": False}

def control_step():
    # STARTUP GUARD. Until the first joint_state and pose have arrived, qj is
    # all zeros - running the motor PD against q=0 kicks the robot hard enough
    # to scoot it half a metre and spike its attitude, which latches MIT's
    # orientation E-stop before the run even begins (measured: dog displaced
    # 0.45 m at spawn, "Orientation safety check failed!", ESTOP for ever).
    if not (_ready["joints"] and _ready["pose"]):
        return 1.0
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

    # MICRO-STALENESS GUARD (tau_ff only). Separate mechanism from the
    # crash watchdog above and two orders of magnitude faster - that one
    # exists for "the controller is gone" (250ms+); this exists for "the
    # controller is a few milliseconds late," which RobotRunner.cpp's own
    # host-stall detector documents as the actual cause of a measured 8-9x
    # force impulse: a host scheduling hiccup delays the controller's next
    # tick, this bridge keeps recomputing torque from the SAME stale
    # command every ~2ms in the meantime, and because joint state barely
    # moves over that short a gap the recomputed torque comes out roughly
    # CONSTANT - effectively holding one command 6-9x longer than the
    # controller intended it for.
    #
    # Only tau_ff decays, deliberately. q_des/kp/kd stay untouched: that is
    # CLOSED-LOOP position tracking against LIVE qj/qdj feedback, which
    # self-corrects regardless of how stale its setpoint is and is what
    # actually keeps a leg doing something sane through a brief gap.
    # tau_ff is an OPEN-LOOP feedforward force with no such correction -
    # it is the term that turns "held too long" into an oversized impulse,
    # so it is the only one that needs to give ground. A per-tick local
    # scale, not a write into the shared cmd dict, so the FULL tau_ff is
    # available again the instant a fresh command actually arrives rather
    # than staying decayed from having been scaled down on a prior read.
    tau_ff_scale = 1.0
    if last_cmd_t[0] > 0.0:
        age = time.time() - last_cmd_t[0]
        if age > TAU_FF_STALE_S:
            tau_ff_scale = max(0.0, 1.0 - (age - TAU_FF_STALE_S) / TAU_FF_RAMP_S)

    with lock:
        # PD in Cheetah frame, then convert torque back to Go1 frame
        qc  = [SIGN[i]*qj[i] + OFFSET[i] for i in range(12)]
        qdc = [SIGN[i]*qdj[i]            for i in range(12)]
        tau_gz = [0.0]*12
        for i in range(12):
            tau_c = (cmd["kp"][i]*(cmd["q_des"][i]-qc[i])
                     + cmd["kd"][i]*(cmd["qd_des"][i]-qdc[i])
                     + cmd["tau_ff"][i] * tau_ff_scale)
            tau_gz[i] = SIGN[i]*tau_c
        last_tau[:] = tau_gz
    apply_torque(tau_gz)
    return tau_ff_scale

def main():
    assert node.subscribe(IMU, IMU_TOPIC, on_imu), "IMU subscribe failed"
    assert node.subscribe(Model, JOINT_TOPIC, on_joint), "joint_state subscribe failed"
    node.subscribe(FluidPressure, BARO_TOPIC, on_baro)
    node.subscribe(NavSat, GPS_TOPIC, on_navsat)
    from gz.msgs10.pose_v_pb2 import Pose_V
    node.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info", on_pose)
    print(f"[bridge] model={MODEL} world={WORLD}")
    print(f"[bridge] subscribed {IMU_TOPIC} {BARO_TOPIC} {GPS_TOPIC} {JOINT_TOPIC}; {len(JOINTS)} force pubs")
    print(f"[bridge] UDP: recv cmd :{CMD_PORT}, send sensors :{SENSOR_PORT} -> {peer_ip[0] or '(learn)'}")
    period = 1.0/500.0    # 500 Hz
    last = time.time()
    hb = time.time()
    stalls = [0, 0.0]    # count >5ms, worst gap (python GIL/callback jitter diagnostics)
    tau_ff_guard = [0, 1.0]   # ticks where the micro-staleness guard engaged, worst scale seen
    prev = time.time()
    while True:
        scale = control_step()
        send_sensor()
        now = time.time()
        gap = now - prev
        prev = now
        if gap > 0.005:
            stalls[0] += 1
            if gap > stalls[1]: stalls[1] = gap
        if scale < 1.0:
            tau_ff_guard[0] += 1
            if scale < tau_ff_guard[1]: tau_ff_guard[1] = scale
        if now - hb >= 1.0:
            with lock:
                q0 = round(qj[0], 3); t0 = round(last_tau[0], 2); t2 = round(last_tau[2], 2)
                bp = round(baro["pressure"], 0); ba = round(baro["alt"], 2)
                la = round(gps["lat"], 5); lo = round(gps["lon"], 5)
            print(f"[bridge] cmd_rx={cmd_rx[0]}/s peer={peer_ip[0]} imu_az={round(imu['accel'][2],2)} "
                  f"q_FR_hip={q0} tau=[{t0},{t2}] stalls>{5}ms={stalls[0]}/worst={round(stalls[1]*1000,1)}ms "
                  f"baro={bp}Pa/{ba}m gps=({la},{lo})"
                  + (f" tau_ff_guard={tau_ff_guard[0]}/worst_scale={round(tau_ff_guard[1],2)}"
                     if tau_ff_guard[0] else ""), flush=True)
            cmd_rx[0] = 0; hb = now; stalls[0] = 0; stalls[1] = 0.0
            tau_ff_guard[0] = 0; tau_ff_guard[1] = 1.0
        last += period
        dt = last - now
        if dt > 0: time.sleep(dt)
        else: last = now

if __name__ == "__main__":
    main()
