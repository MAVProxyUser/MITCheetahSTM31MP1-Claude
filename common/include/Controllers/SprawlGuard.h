#ifndef CHEETAH_SPRAWL_GUARD_H
#define CHEETAH_SPRAWL_GUARD_H

/*!
 * @file SprawlGuard.h
 * @brief Last-ditch roll arrest: throw the legs out and drop.
 *
 * ---------------------------------------------------------------------------
 * WHY
 * ---------------------------------------------------------------------------
 * `ControlFSM::safetyPreCheck` is a detector with no actuator behind it. At
 * |roll| or |pitch| >= 0.5 rad it sets FSM_OperatingMode::ESTOP and prints
 * "broken: Orientation Safety Ceck FAIL", and that is the end of the run. There
 * is no behaviour between "running normally" and "give up" - and the Go1's abad
 * joints, +/-0.863938 rad of travel at 23.7 Nm, have never been used for
 * anything but small foot-placement corrections. The robot goes over with half
 * a radian of unused lateral authority in each hip.
 *
 * A person shoved sideways does not keep their feet together and hope. They
 * throw a foot out.
 *
 * ---------------------------------------------------------------------------
 * THE SIGNATURE, MEASURED (not guessed)
 * ---------------------------------------------------------------------------
 * 50 Hz attitude traces ($CTRL_ATT_DBG), five atom-course roll-outs against
 * six passing runs of both courses. Peak |roll| reached, excluding the tail
 * after the 0.5 rad trip:
 *
 *     falls   0.362  0.477  0.478  0.482  0.492
 *     passes  0.209  0.212  0.217  0.232  0.234  0.235  0.239  0.246
 *             0.253  0.254  0.255
 *
 * so |roll| = 0.30 separates the populations with 18 % margin over the worst
 * pass, and the warning it buys before the 0.5 rad trip is
 *
 *     160 ms   1060 ms   1400 ms   1580 ms   1700 ms
 *
 * - four of five over a second, worst case 160 ms. An abad joint rated at
 * 30.1 rad/s crosses 0.45 rad in 15 ms, so even the worst case is actionable.
 *
 * ROLL RATE IS NOT USABLE AS THE TRIGGER, which is worth recording because it
 * is the obvious first choice. |roll rate| > 2.2 rad/s fires only 60-200 ms
 * before the trip (too late, and it misses one fall entirely), and healthy
 * hard cornering reaches 3.8 rad/s - so it false-fires on passing runs. A
 * roll + 0.25*rate predictor buys 1.5-2.4 s of warning and fires on EVERY
 * passing run. Angle alone is the clean discriminator here.
 *
 * ---------------------------------------------------------------------------
 * RESULT: IT ARRESTS ROLLS. IT DOES NOT SAVE RUNS. ($CTRL_SPRAWL stays OFF.)
 * ---------------------------------------------------------------------------
 * Atom course at 2.3 m/s, four runs per arm:
 *
 *     control   0/4      roll runs away to 0.93 - 1.56 rad
 *     mode 2    0/4      arrests 3 of 5 rolls, then pitches over (to 1.03 rad)
 *     mode 1    0/4      arrests 1 of 4, and still pitches over
 *
 * The arrest itself is real and is worth keeping on record. Mode 2 caught rolls
 * at +0.405, +0.365 and +0.338 - all of them past the 0.362 floor of the
 * unaided fall population - and each gained under 0.03 rad more before coming
 * back under 0.20 in 0.10-0.68 s. Against a control where roll reaches 1.56
 * rad, the abad joints demonstrably stop a roll-out.
 *
 * It does not follow that the run is saved, and it did not. The lesson worth
 * carrying is that WARNING TIME BEFORE A THRESHOLD IS NOT WARNING TIME BEFORE
 * THE POINT OF NO RETURN. |roll| = 0.30 buys 160-1700 ms before the 0.5 rad
 * trip, which sounded like plenty; but by 0.30 the robot is already committed,
 * and arresting the roll just means it fails via pitch or via the collapse
 * detector instead. Both modes are kept, off by default, because the next
 * attempt at this should start from a signal that fires EARLIER than roll -
 * lateral velocity, or the planned lateral demand - rather than from a better
 * actuator response to the same late trigger.
 *
 * There is also a hint that arming can HURT: at 2.1 m/s, which passes 3/3 and
 * 6/6 unaided, runs with the guard enabled went 2/3, the failure being a run
 * that armed. n is small and the roll genuinely reached 0.393 there, so this is
 * a flag rather than a finding - but it is the reason for the default.
 *
 * ---------------------------------------------------------------------------
 * WHAT IT DOES
 * ---------------------------------------------------------------------------
 * Splays the abad joints outward. From the leg kinematics
 * (LegController.cpp, computeLegJacobianAndPosition),
 *
 *     p_y = (l1+l4)*sideSign*cos(q0) + sin(q0)*(l3*c23 + l2*c2)
 *
 * and with the leg near its stance posture the bracket is about +0.297 m, so
 * `q0 = sideSign * splay` moves each foot OUTWARD - which is why the command
 * below is naturally symmetric in sideSign. Two things follow from one action:
 *
 *   - the foot moves out ~0.13 m per side at 0.45 rad, widening the support
 *     polygon on the side the body is falling toward, which is where the
 *     restoring moment has to come from;
 *   - p_z rises from -0.297 to -0.233, so the body DROPS ~0.064 m, cutting the
 *     moment arm the toppling torque acts through.
 *
 * Extra splay goes to the falling side. In this frame (x forward, y left,
 * z up) positive roll takes +y up and -y down, so positive roll is falling
 * RIGHT, and the right legs are sideSign -1.
 *
 * The hip and knee angles are LATCHED at the moment of arming and held, not
 * tracked. Commanding `qDes = q` every tick is a zero-error PD - it holds
 * nothing.
 */

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>

/*!
 * Forward-speed scale the guard asks for while it is arrested, 1 = untouched.
 * Read by ConvexMPCLocomotion and applied to the velocity COMMAND, alongside
 * the height governor's own derate. Function-local static so both translation
 * units share one object without a definition in a .cpp.
 */
inline std::atomic<float>& sprawlSpeedScaleRef() {
  static std::atomic<float> s{1.f};
  return s;
}

class SprawlGuard {
 public:
  float on_roll   = 0.30f;   //!< rad, arm above this (measured: passes <= 0.255)
  float off_roll  = 0.20f;   //!< rad, disarm below this
  /*!
   * PITCH IS THE OTHER DOOR. `checkSafeOrientation` tests roll OR pitch, and
   * the first version of this guard - which watched roll only - arrested the
   * roll and then let the robot go over its nose at 1.034 rad. Guarding one
   * axis of a two-axis test just changes which one kills you.
   *
   * Threshold from the same traces. Peak |pitch| reached:
   *     atom passes   0.098 0.125 0.127
   *     star passes   0.125 0.137 0.152     (height governor ON)
   *     atom falls    0.351 0.368 0.413 0.450 0.459
   * so 0.30 clears the passing population by 2x and sits under every fall.
   *
   * CAVEAT worth keeping: star runs with the governor OFF reach 0.275-0.419
   * pitch and still pass, so this threshold assumes the height governor is
   * running. With $CTRL_HGOV=0 it will false-fire.
   */
  float on_pitch  = 0.30f;
  float off_pitch = 0.20f;
  float arm_s     = 0.06f;   //!< s the trigger must hold - 3 samples at 50 Hz
  float hold_s    = 0.40f;   //!< s minimum sprawl once armed
  float splay     = 0.45f;   //!< rad, symmetric outward splay (abad, roll axis)
  float splay_dir = 0.30f;   //!< rad, EXTRA on the side being fallen toward
  /*!
   * The sagittal equivalent, on the hip joint. From the leg kinematics
   * `p_x = l3*sin(q1+q2) + l2*sin(q1)`, so a positive hip bias carries the
   * foot FORWARD. Front feet forward and rear feet back widens the fore-aft
   * base exactly as the abad splay widens the lateral one.
   *
   * Sign is measured, not assumed: on_1 pitched to +1.034 rad and the fall log
   * read `pitch=46 deg` with the robot over its nose, so POSITIVE PITCH IS
   * NOSE-DOWN in this frame. A nose-down event therefore wants the front feet
   * thrown forward to catch it - which is what an animal tripping does, and
   * what nothing in this stack has ever done.
   */
  float reach     = 0.25f;   //!< rad, symmetric fore-aft hip splay
  float reach_dir = 0.20f;   //!< rad, EXTRA on the end being fallen toward
  float slew      = 12.0f;   //!< rad/s onto the target; not a step
  float kp        = 60.0f;   //!< joint PD - stiffer than the 20 used by the
  float kd        = 2.0f;    //!< jump states, because this is a catch

  /*!
   * MODE 1 (default) - ABAD ONLY, ADDITIVE. Add a lateral joint-PD bias to the
   * abad channel and leave hip, knee, and the whole Cartesian/feedforward path
   * exactly as the controller produced them. `LegController::updateCommand`
   * sums joint PD on top of J^T * footForce, so this superimposes a splay on a
   * gait that keeps running.
   *
   * MODE 2 - FULL LATCH. Freeze hip and knee at the arming posture and hold
   * everything with stiff joint PD. This was the first version and it is kept
   * only because its failure is instructive: IT ARRESTS THE ROLL AND THEN THE
   * ROBOT PITCHES OVER THE NOSE. Measured on the atom at 2.3 m/s -
   *
   *     run     roll arrested            |pitch| reached
   *     on_1    peak 0.431, back < 0.20   1.034 rad  (59 deg - face plant)
   *     on_2    peak 0.369                0.526
   *     pass_3  peak 0.351                0.530
   *
   * and `[FALL] collapsed: roll=8 deg pitch=46 deg`. Locking four braced legs
   * under a body still carrying 2.3 m/s plants the feet and levers the body
   * over them. The roll guard was fine; dumping the forward momentum into the
   * ground was not. Hence mode 1, which never stops the legs swinging.
   */
  int   mode      = 1;
  float v_cut     = 0.5f;    //!< forward-speed scale while armed (1 = none)

  bool  enabled   = false;   //!< $CTRL_SPRAWL - OFF unless asked for
  bool  debug     = false;

  void configureFromEnv() {
    auto f = [](const char* k, float d) {
      const char* e = getenv(k); return e ? (float)atof(e) : d;
    };
    enabled   = f("CTRL_SPRAWL", 0.f) > 0.5f;
    on_roll   = f("CTRL_SPRAWL_ON",   on_roll);
    off_roll  = f("CTRL_SPRAWL_OFF",  off_roll);
    splay     = f("CTRL_SPRAWL_SPLAY",splay);
    splay_dir = f("CTRL_SPRAWL_DIR",  splay_dir);
    on_pitch  = f("CTRL_SPRAWL_ONP",  on_pitch);
    off_pitch = f("CTRL_SPRAWL_OFFP", off_pitch);
    reach     = f("CTRL_SPRAWL_REACH",reach);
    reach_dir = f("CTRL_SPRAWL_REACHDIR", reach_dir);
    hold_s    = f("CTRL_SPRAWL_HOLD", hold_s);
    slew      = f("CTRL_SPRAWL_SLEW", slew);
    kp        = f("CTRL_SPRAWL_KP",   kp);
    kd        = f("CTRL_SPRAWL_KD",   kd);
    mode      = (int)f("CTRL_SPRAWL_MODE", (float)mode);
    v_cut     = f("CTRL_SPRAWL_VCUT", v_cut);
    debug     = getenv("CTRL_SPRAWL_DBG") != nullptr;
    _configured = true;
  }

  //! @return true while the guard owns the leg commands.
  bool update(float roll, float pitch, float dt) {
    if (!_configured) configureFromEnv();
    if (!enabled) return false;

    const float a = std::fabs(roll), b = std::fabs(pitch);
    // Severity per axis, 0 at the disarm level and 1 at the arm level, so the
    // response is graded rather than all-or-nothing.
    auto sev = [](float x, float lo, float hi) {
      return std::max(0.f, std::min(1.f, (x - lo) / std::max(1e-3f, hi - lo)));
    };
    if (!_armed) {
      _above = (a >= on_roll || b >= on_pitch) ? _above + dt : 0.f;
      if (_above >= arm_s) {
        _armed = true; _held = 0.f; _blend = 0.f; _latched = false;
        _fallSide = (roll > 0.f) ? -1.f : 1.f;      // +roll = falling right
        _noseDown = (pitch > 0.f);                  // +pitch = nose down
        ++_fires;
        if (debug) {
          printf("[SPRAWL] ARM  roll=%+.3f pitch=%+.3f  falling=%s %s\n",
                 roll, pitch, _fallSide < 0 ? "RIGHT" : "LEFT",
                 _noseDown ? "NOSE-DOWN" : "TAIL-DOWN");
          fflush(stdout);
        }
      }
    } else {
      _held += dt;
      _blend = std::min(1.f, _blend + slew * dt);
      // Do not let go early, and do not let go while either axis is still over.
      if (_held >= hold_s && a < off_roll && b < off_pitch) {
        _armed = false; _above = 0.f;
        if (debug) { printf("[SPRAWL] RELEASE roll=%+.3f pitch=%+.3f after %.2f s\n",
                            roll, pitch, _held); fflush(stdout); }
      }
    }
    _sRoll  = sev(a, off_roll,  on_roll);
    _sPitch = sev(b, off_pitch, on_pitch);
    // Ask for less forward speed while arrested. A catch that has to absorb
    // 2.3 m/s of momentum through braced legs is how mode 2 face-planted.
    sprawlSpeedScaleRef().store(_armed ? (1.f - (1.f - v_cut) * _blend) : 1.f);
    return _armed;
  }

  //! Latch the posture the legs were in when the guard took over. Call once
  //! per arming, before the first abadTarget().
  bool needsLatch() const { return _armed && !_latched; }
  void markLatched() { _latched = true; }

  /*! Commanded abad angle for a leg.
   *  @param sideSign  -1 for right legs (0, 2), +1 for left (1, 3)
   *  @param q0_latched the abad angle at the moment of arming */
  float abadTarget(float sideSign, float q0_latched) const {
    const float extra = (sideSign == _fallSide) ? splay_dir : 0.f;
    const float tgt = sideSign * (splay + extra) * _sRoll;
    return q0_latched + (tgt - q0_latched) * _blend;
  }

  /*! Commanded hip angle - the sagittal half of the sprawl.
   *  @param leg        0,1 = front; 2,3 = rear
   *  @param q1_latched the hip angle at the moment of arming */
  float hipTarget(int leg, float q1_latched) const {
    const bool front = (leg < 2);
    // Front feet forward, rear feet back: widen the fore-aft base.
    const float dir = front ? +1.f : -1.f;
    // Extra reach to the end the body is falling toward.
    const float extra = ((front && _noseDown) || (!front && !_noseDown))
                        ? reach_dir : 0.f;
    const float tgt = q1_latched + dir * (reach + extra) * _sPitch;
    return q1_latched + (tgt - q1_latched) * _blend;
  }

  float rollSeverity() const { return _sRoll; }
  float pitchSeverity() const { return _sPitch; }

  int   fires() const { return _fires; }
  bool  armed() const { return _armed; }

 private:
  bool  _configured = false;
  bool  _armed = false, _latched = false;
  float _above = 0.f, _held = 0.f, _blend = 0.f, _fallSide = -1.f;
  float _sRoll = 0.f, _sPitch = 0.f;
  bool  _noseDown = true;
  int   _fires = 0;
};

#endif  // CHEETAH_SPRAWL_GUARD_H
