#include <cmath>
#include <cstdlib>
#include <cstdio>
#include "StaticGaitController.hpp"

// Go1 geometry (matches buildGo1 / the URDF)
static const float L1 = 0.08f;    // abad link (lateral hip offset)
static const float L2 = 0.213f;   // thigh
static const float L3 = 0.213f;   // calf
// leg order 0..3 = FR, FL, RR, RL; side sign of the abad y-offset
static const float SIDE[4] = {-1.f, 1.f, -1.f, 1.f};

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

  const float standTime = 4.0f;
  // classic crawl order: RR, FR, RL, FL (legs 2,0,3,1); quarter k swings order[k]
  const int order[4] = {2, 0, 3, 1};
  // quarter index of each leg (inverse of order[])
  const int quarterOf[4] = {1 /*FR*/, 3 /*FL*/, 0 /*RR*/, 2 /*RL*/};

  float x_home = -0.01f;   // feet a hair behind hips (trunk CoM sits 2.2 cm fwd)

  float tg = t - standTime;
  float vscale = (tg > 0) ? fminf(tg / T, 1.f) : 0.f;   // speed ramp, 1st cycle
  float stride = VX * T;
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

  for (int leg = 0; leg < 4; ++leg) {
    float x, y, z, qDes[3];

    if (t < standTime) {
      // ramp into the crouch-stand at home positions
      float s = fminf(t / (standTime - 1.0f), 1.f);
      legIK(leg, x_home, SIDE[leg] * L1, -(0.10f + (H - 0.10f) * s), qDes);
      _legController->commands[leg].qDes = Vec3<float>(qDes[0], qDes[1], qDes[2]);
      _legController->commands[leg].qdDes = Vec3<float>::Zero();
      _legController->commands[leg].tauFeedForward = Vec3<float>::Zero();
      _legController->commands[leg].kpJoint = kp * s;
      _legController->commands[leg].kdJoint = kd;
      continue;
    }

    // per-leg stride (differential for turning: +TURN shortens right strides)
    float legStride = stride * vscale * (1.f - 0.5f * TURN * SIDE[leg]);
    y = SIDE[leg] * L1 + lean;

    if (leg == swingLeg && sp >= 0.15f && sp <= 0.85f && vscale > 0.02f) {
      // swing: half-sine lift; foot travels rear -> front. Endpoints match the
      // stance sweep evaluated at the swing window edges (f=0.9125 / 0.0875):
      // +-(0.4125 - 0.825*0.0875) = +-0.3403 stride, so touchdown is seamless.
      float ss = (sp - 0.15f) / 0.7f;
      x = x_home + 0.6806f * legStride * (ss - 0.5f);
      z = -H + LIFT * sinf((float)M_PI * ss);
    } else {
      // stance: sweep backward at body speed. f = cycle fraction since this
      // leg's mid-swing; foot goes +0.4125*stride (just landed) -> -0.4125.
      float f = fmodf(ph - (quarterOf[leg] + 0.5f) / 4.f + 1.f, 1.f);
      x = x_home + legStride * (0.4125f - 0.825f * f);
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
