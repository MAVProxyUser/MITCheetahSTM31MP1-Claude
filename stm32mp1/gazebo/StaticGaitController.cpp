#include <cmath>
#include <cstdlib>
#include <cstdio>
#include "StaticGaitController.hpp"
#include "WaypointNav.hpp"
#include "rt/rt_gazebo.h"     // gazebo_get_aux(): GPS, same data the real robot gets over CAN

// Go1 geometry (matches buildGo1 / the URDF)
static const float L1 = 0.08f;    // abad link (lateral hip offset)
static const float L2 = 0.213f;   // thigh
static const float L3 = 0.213f;   // calf
// leg order 0..3 = FR, FL, RR, RL; side sign of the abad y-offset
static const float SIDE[4] = {-1.f, 1.f, -1.f, 1.f};
static const float FRONT[4] = {1.f, 1.f, -1.f, -1.f};
static const float HIPX = 0.1881f;   // hip x offset from body centre

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* Abstract-convention analytic IK, matching the LegController FK:
 * verified numerically: IK(0, side*L1, -0.23) = (0, -1.0, 2.0) whose FK is
 * (0, side*0.08, -0.230). Small-angle in abad (shifts <= 6 cm), exact
 * sagittal 2-link solution. */
void StaticGaitController::legIK(int leg, float x, float y, float z, float* q) {
  q[0] = clampf((y - SIDE[leg] * L1) / (-z), -0.6f, 0.6f);   // abad (small-angle)
  float r = sqrtf(x * x + z * z);
  r = clampf(r, 0.10f, L2 + L3 - 0.01f);
  float c_knee = (L2 * L2 + L3 * L3 - r * r) / (2.f * L2 * L3);
  q[2] = (float)M_PI - acosf(clampf(c_knee, -1.f, 1.f));     // knee (abstract, >0)
  float c_hip = (L2 * L2 + r * r - L3 * L3) / (2.f * L2 * r);
  q[1] = atan2f(x, -z) - acosf(clampf(c_hip, -1.f, 1.f));    // hip (abstract, <0 at stand)
}

void StaticGaitController::runController() {
  static int iter = 0;
  ++iter;
  float t = iter * 0.002f;   // 500 Hz

  // ---- knobs (read once) ----
  static float VX = -1, T = 0, SHIFT = 0, LIFT = 0, H = 0, TURN = 0;
  if (VX < 0) {
    VX    = getenv("SG_VX")    ? atof(getenv("SG_VX"))    : 0.05f;
    T     = getenv("SG_T")     ? atof(getenv("SG_T"))     : 5.0f;
    SHIFT = getenv("SG_SHIFT") ? atof(getenv("SG_SHIFT")) : 0.055f;
    LIFT  = getenv("SG_LIFT")  ? atof(getenv("SG_LIFT"))  : 0.06f;
    H     = getenv("SG_H")     ? atof(getenv("SG_H"))     : 0.23f;
    TURN  = getenv("SG_TURN")  ? atof(getenv("SG_TURN"))  : 0.0f;
    printf("[sgait] vx=%.3f T=%.1f shift=%.3f lift=%.3f h=%.2f turn=%.2f\n",
           VX, T, SHIFT, LIFT, H, TURN);
    fflush(stdout);
  }

  Mat3<float> kp = Mat3<float>::Zero();
  kp.diagonal() << 70.f, 70.f, 70.f;
  Mat3<float> kd = Mat3<float>::Zero();
  kd.diagonal() << 2.f, 2.f, 2.f;
  _legController->_maxTorque = 33;
  _legController->_legsEnabled = true;

  // Fall detection: attitude, not impact (footfalls are legitimate impacts).
  if (_safety.update(_stateEstimate, 0.002f)) {
    SafetyCheck::goLimp(_legController);
    return;
  }

  const float standTime = 4.0f;
  // classic crawl order: RR, FR, RL, FL (legs 2,0,3,1); quarter k swings order[k]
  const int order[4] = {2, 0, 3, 1};
  // quarter index of each leg (inverse of order[])
  const int quarterOf[4] = {1 /*FR*/, 3 /*FL*/, 0 /*RR*/, 2 /*RL*/};

  float x_home = -0.01f;   // feet a hair behind hips (trunk CoM sits 2.2 cm fwd)

  float tg = t - standTime;
  float vscale = (tg > 0) ? fminf(tg / T, 1.f) : 0.f;   // speed ramp, 1st cycle

  // ---- drive command: waypoint mission, operator stick, or the SG_* defaults ----
  // $WP_MISSION=circle:<radius>:<points>  or  outback:<distance>
  // Position comes from GPS (what the real dog will have over CAN), heading
  // from the state estimator.
  static WaypointNav* nav = nullptr;
  static bool navTried = false;
  static float navV = 0.f, navW = 0.f;
  if (!navTried) {
    navTried = true;
    const char* m = getenv("WP_MISSION");
    if (m) {
      nav = new WaypointNav();
      float r = 3.f, d = 5.f; int pts = 8;
      if (sscanf(m, "star:%f:%d", &r, &pts) >= 1)      nav->makeStar(r, pts, VX);
      else if (sscanf(m, "circle:%f:%d", &r, &pts) >= 1) nav->makeCircle(r, pts, VX);
      else if (sscanf(m, "outback:%f", &d) == 1)    nav->makeOutAndBack(d, VX);
      else                                          nav->makeCircle(3.f, 8, VX);
      if (getenv("WP_ACCEPT")) nav->accept_radius = atof(getenv("WP_ACCEPT"));
      if (getenv("WP_LOOP"))   nav->loop = true;
    }
  }

  static float vx_cmd = -1.f, turn_cmd = 0.f;
  if (vx_cmd < 0.f) { vx_cmd = 0.f; }
  {
    float vx_tgt = VX, tr_tgt = TURN;
    if (nav) {
      SimAuxSensors aux;
      gazebo_get_aux(&aux);
      if (!nav->originSet() && aux.gps_lat != 0.0) nav->setOrigin(aux.gps_lat, aux.gps_lon);
      if (nav->originSet()) {
        float N, E;
        nav->toLocal(aux.gps_lat, aux.gps_lon, &N, &E);
        // Estimator yaw is zeroed at start and the dog spawns facing north, so
        // heading-from-north (positive toward EAST) is the NEGATIVE of it:
        // +yaw_est is CCW in ENU, i.e. north -> west.
        float yaw_est = _stateEstimate ? _stateEstimate->rpy[2] : 0.f;
        float bearing = -yaw_est;
        float spd = 0.f;
        if (_stateEstimate) {
          float vb0 = _stateEstimate->vBody[0], vb1 = _stateEstimate->vBody[1];
          spd = sqrtf(vb0 * vb0 + vb1 * vb1);
        }
        float nv = 0.f, nw = 0.f;
        bool running = nav->update(N, E, bearing, spd, 0.002f, &nv, &nw);
        navV = running ? nv : 0.f;
        // Sign, measured not assumed: with the body-axis yaw, SG_TURN=+0.6
        // produces a CLOCKWISE turn (world yaw -217 deg in 15 s). nav's yaw
        // rate is compass-sense (+ = turn toward east = clockwise), so the two
        // now agree and the command passes straight through. NOTE this is the
        // opposite of the old differential-stride turn, which was CCW-positive.
        navW = running ? nw : 0.f;
        static int navlog = 0;
        if ((++navlog % 250) == 0) {
          printf("[nav] wp%d/%d  pos N=%.2f E=%.2f  hdg=%.0f deg  d=%.2f m  v=%.2f w=%.2f\n",
                 nav->activeIndex(), nav->count(), N, E, bearing * 57.2958f,
                 nav->lastDistance(), navV, navW);
          fflush(stdout);
        }
      }
      vx_tgt = navV; tr_tgt = navW;
    } else if (_driverCommand) {
      float vx_in = _driverCommand->leftStickAnalog[1];
      float tr_in = _driverCommand->rightStickAnalog[0];
      if (fabsf(vx_in) > 1e-4f) vx_tgt = vx_in;
      if (fabsf(tr_in) > 1e-4f) tr_tgt = tr_in;
    }
    float a = 0.004f;                         // ~0.5 s time constant at 500 Hz
    vx_cmd   += a * (vx_tgt - vx_cmd);
    turn_cmd += a * (tr_tgt - turn_cmd);
  }
  float stride = vx_cmd * T;
  float TURN_NOW = turn_cmd;
  float ph = (tg > 0) ? fmodf(tg / T, 1.f) : 0.f;       // cycle phase 0..1
  int   seg = (int)(ph * 4.f) & 3;                      // quarter 0..3
  float sp = ph * 4.f - seg;                            // phase in quarter 0..1
  int   swingLeg = order[seg];

  // Lateral CoM management: shift the BODY away from the swing leg, i.e. shift
  // ALL FEET toward the swing side: feet y += SIDE[swingLeg]*SHIFT.
  // Blend to the next quarter's lean over the last 30% of each quarter.
  float leanNow  = SIDE[swingLeg] * SHIFT;
  float leanNext = SIDE[order[(seg + 1) & 3]] * SHIFT;
  float lean = (sp < 0.7f)
                   ? leanNow
                   : leanNow + (leanNext - leanNow) * 0.5f *
                         (1.f - cosf((sp - 0.7f) / 0.3f * (float)M_PI));

  // ---- boot phase: legs limp on the deck (real Go1 power-on procedure) ----
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

  for (int leg = 0; leg < 4; ++leg) {
    float x, y, z, qDes[3];

    if (t < standTime) {
      // ramp into the crouch-stand at home positions
      float s = fminf((t - BOOT_S) / (standTime - BOOT_S - 0.5f), 1.f);
      legIK(leg, x_home, SIDE[leg] * L1, -(0.10f + (H - 0.10f) * s), qDes);
      _legController->commands[leg].qDes = Vec3<float>(qDes[0], qDes[1], qDes[2]);
      _legController->commands[leg].qdDes = Vec3<float>::Zero();
      _legController->commands[leg].tauFeedForward = Vec3<float>::Zero();
      _legController->commands[leg].kpJoint = kp * s;
      _legController->commands[leg].kdJoint = kd;
      continue;
    }

    // Forward stride for this leg.
    float legStride = stride * vscale;

    // YAW. Differential stride length alone (what this used to do) is a very
    // weak steering authority - the dog needed metres of arc to come round onto
    // the next leg, which is what produced the big loops in the ground track.
    // A quadruped turns by ROTATING ITS STANCE FEET ABOUT THE BODY AXIS: for a
    // body yaw rate w, a foot under hip (hx,hy) must travel at -(w x r), i.e.
    // (+w*hy, -w*hx) in the body frame. Over a stance that is a real
    // displacement per step, so the dog pivots instead of arcing.
    float hx = FRONT[leg] * HIPX;
    float hy = SIDE[leg] * L1;
    // Radians of body yaw per gait cycle, CLAMPED to what the legs can reach.
    // The lateral foot displacement this asks for is yawStep*HIPX; the abad
    // joint can only take the foot ~0.10 m sideways before the IK clamps, and
    // an over-large request simply saturates every joint and rotates nothing
    // (a 1.6 rad/s command at a 0.85 s cycle asks for 25 cm and deadlocks).
    float yawStep = TURN_NOW * vscale * T;
    const float YAW_STEP_MAX = 0.45f;
    yawStep = clampf(yawStep, -YAW_STEP_MAX, YAW_STEP_MAX);
    float turnDx = clampf( yawStep * hy, -0.06f, 0.06f);
    float turnDy = clampf(-yawStep * hx, -0.10f, 0.10f);

    y = SIDE[leg] * L1 + lean;

    if (leg == swingLeg && sp >= 0.15f && sp <= 0.85f && vscale > 0.02f) {
      // swing: half-sine lift; foot travels rear -> front. Endpoints match the
      // stance sweep evaluated at the swing window edges (f=0.9125 / 0.0875):
      // +-(0.4125 - 0.825*0.0875) = +-0.3403 stride, so touchdown is seamless.
      float ss = (sp - 0.15f) / 0.7f;
      x = x_home + 0.6806f * (legStride * (ss - 0.5f) + turnDx * (ss - 0.5f));
      y += 0.6806f * turnDy * (ss - 0.5f);
      z = -H + LIFT * sinf((float)M_PI * ss);
    } else {
      // stance: sweep backward at body speed. f = cycle fraction since this
      // leg's mid-swing; foot goes +0.4125*stride (just landed) -> -0.4125.
      float f = fmodf(ph - (quarterOf[leg] + 0.5f) / 4.f + 1.f, 1.f);
      x = x_home + (legStride + turnDx) * (0.4125f - 0.825f * f);
      y += turnDy * (0.4125f - 0.825f * f);
      z = -H;
    }

    legIK(leg, x, y, z, qDes);
    _legController->commands[leg].qDes = Vec3<float>(qDes[0], qDes[1], qDes[2]);
    _legController->commands[leg].qdDes = Vec3<float>::Zero();
    _legController->commands[leg].tauFeedForward = Vec3<float>::Zero();
    _legController->commands[leg].kpJoint = kp;
    _legController->commands[leg].kdJoint = kd;
  }
}
