#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include "TrotController.hpp"

// Go1 geometry (matches buildGo1 / the URDF)
static const float L1 = 0.08f;    // abad link (lateral hip offset)
static const float L2 = 0.213f;   // thigh
static const float L3 = 0.213f;   // calf
// leg 0..3 = FR, FL, RR, RL
static const float SIDE[4] = {-1.f, 1.f, -1.f, 1.f};   // +1 = left
static const float FRONT[4] = {1.f, 1.f, -1.f, -1.f};  // +1 = front
// hip x offset from body centre (buildGo1 bodyLength/2)
static const float HIPX = 0.1881f;
// Which legs swing together. Selected by $TR_GAIT:
//   trot  diagonal pairs  FR+RL / FL+RR   - the all-round gait
//   pace  lateral pairs   FR+RR / FL+RL   - less roll-coupling, more sway
//   bound front/rear      FR+FL / RR+RL   - the high-speed gait; pitches
//   pronk all four together                - hops, mostly a torque test
static int PAIR[4] = {0, 1, 1, 0};
static void selectGait(const char* g) {
  if (!g) return;
  if (!strcmp(g, "pace"))       { PAIR[0]=0; PAIR[1]=1; PAIR[2]=0; PAIR[3]=1; }
  else if (!strcmp(g, "bound")) { PAIR[0]=0; PAIR[1]=0; PAIR[2]=1; PAIR[3]=1; }
  else if (!strcmp(g, "pronk")) { PAIR[0]=0; PAIR[1]=0; PAIR[2]=0; PAIR[3]=0; }
  printf("[trot] gait=%s pairs=%d%d%d%d\n", g, PAIR[0], PAIR[1], PAIR[2], PAIR[3]);
}

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void TrotController::legIK(int leg, float x, float y, float z, float* q) {
  q[0] = clampf((y - SIDE[leg] * L1) / (-z), -0.7f, 0.7f);   // abad (small-angle)
  float r = sqrtf(x * x + z * z);
  r = clampf(r, 0.12f, L2 + L3 - 0.015f);
  float c_knee = (L2 * L2 + L3 * L3 - r * r) / (2.f * L2 * L3);
  q[2] = (float)M_PI - acosf(clampf(c_knee, -1.f, 1.f));     // knee (abstract, >0)
  float c_hip = (L2 * L2 + r * r - L3 * L3) / (2.f * L2 * r);
  q[1] = atan2f(x, -z) - acosf(clampf(c_hip, -1.f, 1.f));    // hip (abstract, <0 at stand)
}

void TrotController::runController() {
  static int iter = 0;
  ++iter;
  const float dt = 0.002f;          // 500 Hz
  float t = iter * dt;

  // ---- knobs (read once) ----
  static float V = -1, T = 0, H = 0, LIFT = 0, KV = 0;
  static float KPR = 0, KDR = 0, KPP = 0, KDP = 0, KPJ = 0, KDJ = 0, YAW = 0, STAND_S = 0;
  static float DUTY = 0, FF = 0, KPC = 0, KDC = 0;
  static int CART = 1;
  if (V < 0) {
    V       = getenv("TR_V")        ? atof(getenv("TR_V"))        : 0.6f;
    T       = getenv("TR_T")        ? atof(getenv("TR_T"))        : 0.40f;
    H       = getenv("TR_H")        ? atof(getenv("TR_H"))        : 0.28f;
    LIFT    = getenv("TR_LIFT")     ? atof(getenv("TR_LIFT"))     : 0.08f;
    KV      = getenv("TR_KV")       ? atof(getenv("TR_KV"))       : 0.08f;
    KPR     = getenv("TR_KP_ROLL")  ? atof(getenv("TR_KP_ROLL"))  : 0.09f;
    KDR     = getenv("TR_KD_ROLL")  ? atof(getenv("TR_KD_ROLL"))  : 0.012f;
    KPP     = getenv("TR_KP_PITCH") ? atof(getenv("TR_KP_PITCH")) : 0.09f;
    KDP     = getenv("TR_KD_PITCH") ? atof(getenv("TR_KD_PITCH")) : 0.012f;
    KPJ     = getenv("TR_KP_J")     ? atof(getenv("TR_KP_J"))     : 120.f;
    // Cartesian (foot-space) impedance. Defaults are the published Go1 values
    // from Bezier-curve + impedance control work (Kxd=1000 I N/m, Bxd=44 I
    // Ns/m). Joint PD at kp=120 Nm/rad is an effective foot stiffness of
    // kp/l^2 ~ 2700 N/m - nearly 3x stiffer than that - which is why this gait
    // bounced (17 cm of body heave) and why stiffening it made things worse.
    KPC     = getenv("TR_KP_C")     ? atof(getenv("TR_KP_C"))     : 1000.f;
    KDC     = getenv("TR_KD_C")     ? atof(getenv("TR_KD_C"))     : 44.f;
    CART    = getenv("TR_JOINTPD") ? 0 : 1;
    FF      = getenv("TR_FF")       ? atof(getenv("TR_FF"))       : 1.0f;
    KDJ     = getenv("TR_KD_J")     ? atof(getenv("TR_KD_J"))     : 3.f;
    YAW     = getenv("TR_YAW")      ? atof(getenv("TR_YAW"))      : 0.f;
    STAND_S = getenv("TR_STAND_S")  ? atof(getenv("TR_STAND_S"))  : 3.f;
    // Stance fraction of the cycle. 0.5 = pure trot (always exactly 2 feet
    // down, and the body is free to roll about the support diagonal the whole
    // time). >0.5 overlaps the pairs, giving a short all-four double-support
    // window each half cycle - far more forgiving through the SITL loop's
    // latency, and what real quadrupeds use at walking speed.
    DUTY    = getenv("TR_DUTY")     ? atof(getenv("TR_DUTY"))     : 0.65f;
    DUTY    = DUTY < 0.5f ? 0.5f : (DUTY > 0.9f ? 0.9f : DUTY);
    selectGait(getenv("TR_GAIT"));
    printf("[trot] v=%.2f T=%.2f H=%.2f lift=%.3f kv=%.3f kpj=%.0f\n",
           V, T, H, LIFT, KV, KPJ);
    fflush(stdout);
  }

  Mat3<float> kp = Mat3<float>::Zero();  kp.diagonal() << KPJ, KPJ, KPJ;
  Mat3<float> kd = Mat3<float>::Zero();  kd.diagonal() << KDJ, KDJ, KDJ;
  _legController->_maxTorque = 35;
  _legController->_legsEnabled = true;

  // ---- drive command (waypoint follower / operator), smoothed ----
  static float v_cmd = 0.f, yaw_cmd = 0.f;
  {
    float v_in  = _driverCommand ? _driverCommand->leftStickAnalog[1]  : 0.f;
    float y_in  = _driverCommand ? _driverCommand->rightStickAnalog[0] : 0.f;
    float v_tgt = (fabsf(v_in) > 1e-4f) ? v_in : V;
    float y_tgt = (fabsf(y_in) > 1e-4f) ? y_in : YAW;
    float a = 0.004f;                        // ~0.5 s smoothing at 500 Hz
    v_cmd   += a * (v_tgt - v_cmd);
    yaw_cmd += a * (y_tgt - yaw_cmd);
  }

  // ---- body state feedback (roll/pitch and their rates) ----
  float roll = 0, pitch = 0, yaw = 0, wx = 0, wy = 0, wz_act = 0, vx_act = 0;
  if (_stateEstimate) {
    roll  = _stateEstimate->rpy[0];
    pitch = _stateEstimate->rpy[1];
    yaw   = _stateEstimate->rpy[2];
    wx    = _stateEstimate->omegaBody[0];
    wy    = _stateEstimate->omegaBody[1];
    wz_act = _stateEstimate->omegaBody[2];
    vx_act = _stateEstimate->vBody[0];
  }

  // ---- boot phase: legs limp, robot lies on its belly ----
  // Real Go1 procedure is lie flat -> power on -> stand -> walk; matching it
  // also makes every SITL run start from the same settled pose.
  const float BOOT_S = 1.0f;
  if (t < BOOT_S) {
    for (int leg = 0; leg < 4; ++leg) {
      _legController->commands[leg].qDes = _legController->datas[leg].q;
      _legController->commands[leg].qdDes = Vec3<float>::Zero();
      _legController->commands[leg].tauFeedForward = Vec3<float>::Zero();
      _legController->commands[leg].kpJoint = Mat3<float>::Zero();
      _legController->commands[leg].kdJoint = Mat3<float>::Identity() * 0.5f;
    }
    return;
  }

  // ---- stand phase: ease from the folded rest pose into the trot stance ----
  if (t < STAND_S) {
    float s = fminf((t - BOOT_S) / (STAND_S - BOOT_S - 0.3f), 1.f);
    for (int leg = 0; leg < 4; ++leg) {
      float qd_[3];
      legIK(leg, 0.f, SIDE[leg] * L1, -(0.10f + (H - 0.10f) * s), qd_);
      _legController->commands[leg].qDes = Vec3<float>(qd_[0], qd_[1], qd_[2]);
      _legController->commands[leg].qdDes = Vec3<float>::Zero();
      _legController->commands[leg].tauFeedForward = Vec3<float>::Zero();
      _legController->commands[leg].kpJoint = kp * s;
      _legController->commands[leg].kdJoint = kd;
      _wasStance[leg] = true;
      _liftoffX[leg] = 0.f;
    }
    return;
  }

  // ---- heading hold ----
  // _yawRef tracks the heading we intend to be on: it advances at the commanded
  // yaw rate, so "go straight" means "hold the heading you had", and a commanded
  // turn moves the reference rather than fighting it. The correction is added to
  // the per-leg differential stride below.
  static bool  yawInit = false;
  static float yawRef = 0.f;
  if (!yawInit) { yawRef = yaw; yawInit = true; }

  // ---- gait clock ----
  float tg = t - STAND_S;
  // Accelerate over a fixed 3 s, not "2 gait cycles": at a 0.26 s cycle that
  // was a 0.5 s ramp to full speed, i.e. a standing start into a sprint.
  float RAMP_S = getenv("TR_RAMP_S") ? atof(getenv("TR_RAMP_S")) : 3.0f;
  float ramp = fminf(tg / RAMP_S, 1.f);
  float v = v_cmd * ramp;
  float wz = yaw_cmd * ramp;
  yawRef += wz * dt;
  float yawErr = yaw - yawRef;
  while (yawErr >  (float)M_PI) yawErr -= 2.f * (float)M_PI;
  while (yawErr < -(float)M_PI) yawErr += 2.f * (float)M_PI;
  // steer command = commanded rate + P/D pull back onto the reference heading
  float KP_YAW = getenv("TR_KP_YAW") ? atof(getenv("TR_KP_YAW")) : 2.0f;
  float KD_YAW = getenv("TR_KD_YAW") ? atof(getenv("TR_KD_YAW")) : 0.20f;
  float wz_eff = wz - KP_YAW * yawErr - KD_YAW * (wz_act - wz);
  wz_eff = clampf(wz_eff, -1.5f, 1.5f);

  float ph = fmodf(tg / T, 1.f);                 // 0..1
  float swingFrac = 1.f - DUTY;                  // fraction of cycle in swing
  float T_st = DUTY * T;                         // stance duration per leg

  // 2 Hz diagnostic: is the leg TRACKING the plan (then any speed shortfall is
  // foot slip) or LAGGING it (then it is gain / torque limited)?
  if (getenv("TR_DBG")) {
    static int dbg = 0;
    if ((++dbg % 250) == 0 && _trkN > 0) {
      printf("[trot] v_cmd=%.2f v_act=%.2f | mean|q-qDes|=%.3f rad | z_err=%.3f roll=%.1f\n",
             v, vx_act, _trkErr / _trkN, 0.f, roll * 57.3f);
      fflush(stdout);
      _trkErr = 0.f; _trkN = 0;
    }
  }

  // How many legs are on the ground right now (for sharing bodyweight out).
  int nStance = 0;
  for (int L = 0; L < 4; ++L) {
    float ss = (PAIR[L] == 0) ? 0.5f : 0.0f;
    if (fmodf(ph - ss + 1.f, 1.f) >= swingFrac) ++nStance;
  }
  if (nStance < 1) nStance = 1;

  for (int leg = 0; leg < 4; ++leg) {
    // pair 1 swings at the start of the cycle, pair 0 half a cycle later
    float swingStart = (PAIR[leg] == 0) ? 0.5f : 0.0f;
    float rel = fmodf(ph - swingStart + 1.f, 1.f);       // 0..1 since own swing start
    bool swinging = (rel < swingFrac);
    float sp = swinging ? (rel / swingFrac)              // 0..1 through swing
                        : ((rel - swingFrac) / DUTY);    // 0..1 through stance

    // Per-leg forward speed including yaw rate (differential drive):
    // a leg on the outside of the turn must travel faster.
    // differential stride: outside-of-turn legs travel further
    // (0.094 m = half track width: bodyWidth/2 + abad link)
    float v_leg = v - wz_eff * SIDE[leg] * 0.094f;

    // Attitude feedback: EXTEND the legs on the side that is dropping.
    //   frame: x forward, y left, z down-negative (foot below hip => z < 0)
    //   +roll  (about +x) lifts the LEFT side and drops the RIGHT
    //   +pitch (about +y) is nose-up, so the REAR drops
    // A dropping corner needs a LONGER leg, i.e. z more negative, i.e. dz < 0.
    //   right leg (SIDE=-1) under +roll  -> dz = +roll*SIDE = -roll < 0  OK
    //   rear leg  (FRONT=-1) under +pitch-> dz = +pitch*FRONT = -pitch < 0  OK
    // Getting this sign backwards turns the stabiliser into positive feedback:
    // the robot then rolls onto the same side within ~2 s, every single run.
    float dz = (KPR * roll + KDR * wx) * SIDE[leg]
             + (KPP * pitch + KDP * wy) * FRONT[leg];
    dz = clampf(dz, -0.05f, 0.05f);

    float x, y, z, q[3];
    y = SIDE[leg] * L1 + wz * FRONT[leg] * HIPX * 0.0f;   // lateral hip offset

    if (!swinging) {
      // STANCE: foot travels backward under the body at the commanded speed,
      // starting at +v*T_st/2 (touchdown) and ending at -v*T_st/2 (liftoff).
      x = v_leg * T_st * (0.5f - sp);
      z = -H + dz;
      if (_wasStance[leg] == false) _wasStance[leg] = true;
      _liftoffX[leg] = x;      // remember where this leg leaves the ground
    } else {
      // SWING. Two things here cost most of the commanded speed if you get
      // them wrong, and both did:
      //
      // (a) TOUCHDOWN VELOCITY MATCHING. In the body frame a stance foot moves
      //     backward at v. A swing profile that arrives with ZERO body-frame
      //     velocity therefore lands moving FORWARD at v relative to the
      //     ground: every step scrubs forward and brakes the robot. That alone
      //     turned a 0.5 m/s command into ~0.1 m/s of travel. So the horizontal
      //     profile is a cubic Hermite whose end slopes are the stance sweep
      //     rate: the foot is already moving backward at v when it lands.
      //
      // (b) VERTICAL LANDING SPEED. LIFT*sin(pi*s) has slope -pi*LIFT at
      //     touchdown - with LIFT=0.08 over a 0.12 s swing that is a 2 m/s
      //     slam, which bounces the body (measured 6-9 cm of hop) and breaks
      //     traction. (1-cos(2*pi*s))/2 has zero vertical velocity at BOTH
      //     ends: the foot is set down, not dropped.
      float T_sw = swingFrac * T;
      float x_td = v_leg * T_st * 0.5f + KV * (vx_act - v);
      x_td = clampf(x_td, -0.30f, 0.30f);
      float x0 = _liftoffX[leg];
      float s = sp;
      float m = -v_leg * T_sw;              // d(x)/d(s) at both ends
      float s2 = s * s, s3 = s2 * s;
      float h00 = 2.f * s3 - 3.f * s2 + 1.f;
      float h10 = s3 - 2.f * s2 + s;
      float h01 = -2.f * s3 + 3.f * s2;
      float h11 = s3 - s2;
      x = h00 * x0 + h10 * m + h01 * x_td + h11 * m;
      z = -H + dz + LIFT * 0.5f * (1.f - cosf(2.f * (float)M_PI * s));
      _wasStance[leg] = false;
    }

    legIK(leg, x, y, z, q);
    // Feed-forward joint velocity. With qdDes pinned at zero the joint D term
    // fights every intentional motion: at kd=3 and a few rad/s of real swing
    // speed that is >10 Nm of pure braking per joint. Differentiating the
    // commanded angles turns the D term back into damping-about-the-plan.
    Vec3<float> qdDes = Vec3<float>::Zero();
    if (_qPrevValid[leg]) {
      for (int j = 0; j < 3; ++j) {
        float dq = (q[j] - _qPrev[leg][j]) / dt;
        qdDes[j] = clampf(dq, -25.f, 25.f);
      }
    }
    for (int j = 0; j < 3; ++j) _qPrev[leg][j] = q[j];
    _qPrevValid[leg] = true;

    // STANCE FORCE FEED-FORWARD. Without this the only thing holding 13 kg up
    // is joint position error: the body sags until kp*err equals bodyweight,
    // and that error (measured ~0.25 rad) eats the whole 35 Nm torque budget,
    // leaving nothing for propulsion - which is why commanded speed above
    // ~1 m/s produced no extra ground speed at all. Handing each stance leg
    // its share of bodyweight as a foot force (LegController maps it through
    // J^T) lets the PD spend its authority on TRACKING instead of holding up
    // the robot. MIT's stack gets this from the MPC's ground reaction forces.
    // Sign: forceFeedForward is the force the FOOT applies to the GROUND, so
    // supporting the body means pushing DOWN, i.e. -z in the leg frame.
    Vec3<float> fff = Vec3<float>::Zero();
    if (!swinging) {
      const float W = 13.1f * 9.81f;
      fff[2] = -FF * W / (float)nStance;
    }
    _legController->commands[leg].forceFeedForward = fff;
    _legController->commands[leg].qDes = Vec3<float>(q[0], q[1], q[2]);
    _legController->commands[leg].qdDes = qdDes;

    if (CART) {
      // Impedance in FOOT space: LegController turns (pDes-p, vDes-v) into a
      // foot force and maps it through J^T, so the leg is a spring-damper to
      // the planned foot point rather than three independent joint servos.
      // Foot velocity comes from differentiating the planned foot position.
      Vec3<float> pDes(x, y, z);
      Vec3<float> vDes = Vec3<float>::Zero();
      if (_pPrevValid[leg]) {
        vDes[0] = clampf((x - _pPrev[leg][0]) / dt, -12.f, 12.f);
        vDes[1] = clampf((y - _pPrev[leg][1]) / dt, -12.f, 12.f);
        vDes[2] = clampf((z - _pPrev[leg][2]) / dt, -12.f, 12.f);
      }
      _pPrev[leg][0] = x; _pPrev[leg][1] = y; _pPrev[leg][2] = z;
      _pPrevValid[leg] = true;

      float ks = swinging ? 0.45f : 1.f;   // softer in swing: touchdown is gentler
      _legController->commands[leg].pDes = pDes;
      _legController->commands[leg].vDes = vDes;
      _legController->commands[leg].kpCartesian = Mat3<float>::Identity() * (KPC * ks);
      _legController->commands[leg].kdCartesian = Mat3<float>::Identity() * KDC;
      // joint PD off - the impedance IS the controller now
      _legController->commands[leg].kpJoint = Mat3<float>::Zero();
      _legController->commands[leg].kdJoint = Mat3<float>::Identity() * 0.2f;
      continue;
    }
    if (getenv("TR_DBG")) {
      _trkErr += fabsf(q[0] - _legController->datas[leg].q[0])
               + fabsf(q[1] - _legController->datas[leg].q[1])
               + fabsf(q[2] - _legController->datas[leg].q[2]);
      _trkN += 3;
    }
    _legController->commands[leg].tauFeedForward = Vec3<float>::Zero();
    // Swing legs run softer so a mistimed touchdown does not spike the body.
    float gs = swinging ? 0.45f : 1.f;
    _legController->commands[leg].kpJoint = kp * gs;
    _legController->commands[leg].kdJoint = kd;
  }
}
