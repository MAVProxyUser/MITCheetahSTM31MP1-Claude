#ifndef CHEETAH_HEIGHT_GOVERNOR_H
#define CHEETAH_HEIGHT_GOVERNOR_H

/*!
 * @file HeightGovernor.h
 * @brief Reactive stance-height regulation, from cheetah biomechanics.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS EXISTS
 * ---------------------------------------------------------------------------
 * Across every star-course run logged in this port, the one quantity that
 * separates a pass from a fall is the body height the robot actually achieves.
 * Twelve interleaved runs at 2.5 m/s, minimum height reached:
 *
 *     passing runs   0.231 0.234 0.240 0.249 0.251
 *     failing runs   0.179 0.197 0.198 0.199 0.206 0.207   (and one at 0.233)
 *
 * and the COMMANDED vertical force is the same in both (0.85-0.88 x mg). So
 * height is not a symptom of some other fault; it is the integral of the
 * vertical-force deficit, which makes it the earliest thing that is both
 * measurable and actionable.
 *
 * Eleven separate cornering levers (gait switching, banking, corner crouch,
 * angle grading, hairpin pivots, lateral-budget in both directions, extra yaw
 * authority, acceptance radius, lookahead) were measured and every one came
 * back neutral or worse. The logs say why: six of seven failures in that same
 * sweep happened on a STRAIGHT at |yaw rate| <= 0.3 rad/s and 2.24-2.50 m/s,
 * not in a corner at all. Cornering levers were the wrong department.
 *
 * ---------------------------------------------------------------------------
 * WHAT THE BIOLOGY SAYS
 * ---------------------------------------------------------------------------
 * Xiuli Zhang, Chenliang Zhao, Zhongqi Xu, Senwei Huang, "Mechanism analysis of
 * cheetah's high-speed locomotion based on digital reconstruction", Biomimetic
 * Intelligence and Robotics 2 (2022) 100033, doi:10.1016/j.birob.2021.100033.
 * From a frame-by-frame virtual-prototype reconstruction of a galloping
 * cheetah:
 *
 *   - Section 4.3 / Table 6: the animal holds its body at a FIXED FRACTION of
 *     leg length while running - "the height of the forelegs from the ground
 *     is about 0.55 times the average leg length; the height of the hindlegs
 *     from the ground is about 0.57 times". Hind rides slightly higher than
 *     fore: the body is nose-down by ~1.3 deg, not level.
 *
 *   - Table 6 also shows the VIRTUAL LEG length itself swinging over better
 *     than 2x within one cycle (foreleg 0.25-0.6 m, hindleg 0.3-0.7 m). The
 *     stance height is regulated; the leg length is not. So the fraction above
 *     is a controlled variable, not a by-product of the leg geometry.
 *
 *   - Section 4.7: manipulability is LOWEST in stance - "in ST, the leg has the
 *     lowest manipulability, and at this time it can withstand a greater
 *     force" - and highest at the extremes of the swing. The animal arranges
 *     its stance posture for force capacity and its swing posture for agility.
 *
 * Go1 leg length is 0.213 (thigh) + 0.213 (calf) = 0.426 m, so the cheetah's
 * band maps to
 *
 *     0.55 x 0.426 = 0.234 m      0.57 x 0.426 = 0.243 m
 *
 * which lands at the TOP of this robot's measured passing band and clear above
 * every failure. The animal's number and the robot's data agree, from
 * completely independent directions. That is the hypothesis this class tests.
 *
 * ---------------------------------------------------------------------------
 * WHY A GOVERNOR AND NOT JUST A BIGGER _body_height
 * ---------------------------------------------------------------------------
 * Because the reference is not the achieved height. Stock runs a CONSTANT
 * 0.30 m reference and the robot settles wherever the force balance leaves it,
 * cruising at 0.26-0.29 and departing from there when it fails. Raising the
 * constant was already tried (CTRL_BODY_H=0.32) and does nothing predictable,
 * because the plant between reference and achieved height has a large,
 * load-dependent, completely un-regulated droop.
 *
 * Note also that the cheetah ratio does NOT transfer as a setpoint. 0.234 m is
 * where a cheetah would sit; this two-segment Go1 leg cruises at 0.27, which is
 * 0.63 of leg length. What transfers is the PRINCIPLE the paper establishes -
 * that stance height is a regulated variable held constant while the leg length
 * under it varies by more than 2x - plus a defensible absolute floor. Treating
 * 0.234 as a target rather than a floor is exactly the mistake that made the
 * first version of this class harmful; see lever 1.
 *
 * So close the loop, on the right variable and on the right timescale. See
 * WHAT THE FAILURE ACTUALLY LOOKS LIKE, below, for both.
 *
 * ---------------------------------------------------------------------------
 * DIVISION OF LABOUR
 * ---------------------------------------------------------------------------
 * This class is the REACTIVE half - it answers "the body is sinking right now,
 * what do I do about it" from the robot's own state, at controller rate, with
 * no knowledge of the course. The PREDICTIVE half lives in the planner
 * (BodyPathPlanner::plannedHeightBias): the planner knows a corner is coming
 * and can pre-load height margin before the lateral demand arrives, which is
 * what an animal reading the ground ahead does. The planner's contribution
 * arrives here as `bias` and is simply added to the target.
 *
 * Everything is env-tunable and the whole class can be switched off
 * (CTRL_HGOV=0) so it can be A/B'd against stock honestly.
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>

class HeightGovernor {
 public:
  // ---- biological setpoint (Zhang et al. 2022, section 4.3) ----
  float ratio_fore = 0.55f;   //!< foreleg stance height / leg length
  float ratio_hind = 0.57f;   //!< hindleg stance height / leg length
  float leg_len    = 0.426f;  //!< Go1: 0.213 thigh + 0.213 calf

  // ---- WHAT THE FAILURE ACTUALLY LOOKS LIKE ----
  //
  // Logged at 5 Hz through twelve interleaved runs at 2.5 m/s. Body height,
  // last 3 s of each run:
  //
  //   PASS  0.283 0.277 0.271 0.271 0.272 0.286 0.286 0.277 0.272 0.270 0.287 ...
  //   FAIL  0.274 0.268 0.260 0.246 0.228 0.198  <- gone
  //
  // The robot does NOT sag its way into a collapse. It cruises at 0.26-0.29
  // for tens of seconds and then DEPARTS, losing 8 cm in about 0.6 s at up to
  // 0.35 m/s of sink, and the SafetyChecker's orientation test trips at the
  // bottom of it. (The "roll=0 pitch=0" signature in the fall log is recorded
  // AFTER the legs go limp, so it describes the aftermath, not the event.)
  //
  // Two consequences, both of which killed the first working version:
  //
  //   * The trigger height was far too low. 0.55 x leg length = 0.239 m is
  //     BELOW this robot's healthy cruise, so by the time height crosses it
  //     the departure is already unrecoverable. Measured: over six armed runs
  //     the reference trim moved 6 mm and the derate never fired at all. The
  //     A/B that produced this comment was therefore not a test of the idea -
  //     both arms ran the same controller.
  //
  //   * Absolute height is the wrong variable to trigger on. What separates
  //     the populations is DEPARTURE FROM THE ROBOT'S OWN CRUISE, and it shows
  //     up in the sink rate half a second before it shows up in the height.
  //
  // So this governor triggers on a predicted height - h + lead * dh/dt -
  // referred to a cruise height the robot establishes for itself. Self-
  // referencing matters because this port runs 2.0-3.0 m/s across five gaits
  // and each has its own healthy height; a constant would have to be wrong for
  // most of them.
  //
  // On the traces above, with lead = 0.30 s:
  //     failures reach a 0.028 m departure ~0.6 s before the bottom, then 0.050
  //     passes never exceed 0.015
  // which is where the soft/hard thresholds below come from.

  // The body BOBS vertically once per step by design - that is what a gait is.
  // A first cut differentiated the raw height with an 80 ms filter and the bob
  // went straight through it: at 2.5 m/s trotting the vertical oscillation is
  // roughly +/-0.01 m at ~4 Hz, so dh/dt swings +/-0.25 m/s and a 0.3 s lead
  // turned that into a +/-0.075 m phantom departure. Measured: a run that
  // PASSED reported a peak departure of 0.080 m, well past the full-derate
  // threshold. The governor was braking for the gait, not for a collapse.
  //
  // So everything downstream runs off a height low-passed at roughly one gait
  // period. That costs lead time and there is no way around it: the signal
  // simply does not exist below the gait's own period.
  float bob_tc   = 0.25f;     //!< s, ~one gait period. Removes the step bob.
  float lead     = 0.20f;     //!< s. How far ahead to extrapolate the sink.
  float d_tc     = 0.15f;     //!< s, derivative filter on the DE-BOBBED height
  float cruise_up_tc = 0.5f;  //!< s, cruise reference tracks RISES quickly
  float cruise_dn_tc = 6.0f;  //!< s, and FALLS slowly, so a departure cannot
                              //!< drag the reference down with it and hide.

  // ---- lever 1: reference trim ----
  //
  // ONE-SIDED. The first version ran a symmetric setpoint loop and it was
  // measured to be actively harmful: the Go1 cruises ABOVE 0.239, so the loop
  // spent whole runs pushing the reference DOWN against its floor - 2 cm below
  // stock - and only started climbing once the body was at 0.198 and out of
  // time. 1/5 waypoints against stock's 2/5. A governor that defends height
  // has no business crouching the robot.
  float ki       = 3.0f;      //!< 1/s on the departure. Large, because the
                              //!< whole event lasts 0.6 s.
  float relax_tc = 1.5f;      //!< s, decay of accumulated trim back to nominal
                              //!< once clear. Without it the trim ratchets.
  float ref_hi   = 0.360f;    //!< ceiling. Above this the MPC solves for a
                              //!< large upward force and the robot launches -
                              //!< measured at entry, see the ramp in
                              //!< _SetupCommand.
  float slew     = 0.15f;     //!< m/s. Reference may not step.

  // ---- lever 2: speed derate ----
  // Give up forward speed to buy vertical force. Armed by the same predicted
  // departure as lever 1 rather than by lever 1 saturating: whether raising the
  // reference can lift the body is exactly what is unknown, so gating the lever
  // that certainly reduces demand on the success of the one that might was
  // backwards.
  // Thresholds re-derived on the de-bobbed signal (see bob_tc). Replaying the
  // 5 Hz traces through it:
  //     failures  0.008 -> 0.018 -> 0.032 -> 0.055 over the last 0.8 s
  //     passes    never above 0.009
  float d_soft   = 0.015f;    //!< m of predicted departure before derating
  float d_hard   = 0.040f;    //!< m of predicted departure = full derate
  float derate   = 0.35f;     //!< max fraction of commanded speed surrendered
  float derate_tc = 0.12f;    //!< s. Also inside the event's timescale.

  //! Absolute backstop, independent of the cruise tracker: 0.55-0.57 x leg
  //! length is where the cheetah runs (0.234-0.243 m here). Below this the
  //! robot is outside anything biology or our own passing runs support, so
  //! derate regardless of what the cruise tracker believes.
  float abs_soft = 0.010f;    //!< m below floorHeight() before derating
  float abs_hard = 0.045f;    //!< m below floorHeight() = full derate

  // ---- fore/hind differential (section 4.3: hind rides higher) ----
  //! Nose-down pitch that puts the hindquarters ratio_hind and the shoulders
  //! ratio_fore above the ground, over the Go1's 0.3762 m hip separation.
  //! Off by default: it is a 1.3 deg effect and deserves its own A/B, not a
  //! free ride on the height loop's result.
  float pitch_k  = 0.0f;      //!< 1 = full biological differential

  //! m, situational offset from the planner - added to the absolute floor.
  float bias = 0.f;

  bool  enabled = true;
  int   dbg_every = 0;        //!< debug print decimation in ticks, 0 = off

  void configureFromEnv() {
    auto f = [](const char* k, float d) {
      const char* e = getenv(k); return e ? (float)atof(e) : d;
    };
    enabled   = f("CTRL_HGOV", 1.f) > 0.5f;
    ratio_fore= f("CTRL_HGOV_RFORE", ratio_fore);
    ratio_hind= f("CTRL_HGOV_RHIND", ratio_hind);
    leg_len   = f("CTRL_HGOV_LEG",   leg_len);
    lead      = f("CTRL_HGOV_LEAD",  lead);
    ki        = f("CTRL_HGOV_KI",    ki);
    relax_tc  = f("CTRL_HGOV_RELAX", relax_tc);
    ref_hi    = f("CTRL_HGOV_HI",    ref_hi);
    slew      = f("CTRL_HGOV_SLEW",  slew);
    d_soft    = f("CTRL_HGOV_SOFT",  d_soft);
    d_hard    = f("CTRL_HGOV_HARD",  d_hard);
    derate    = f("CTRL_HGOV_DERATE",derate);
    pitch_k   = f("CTRL_HGOV_PITCH", pitch_k);
    // Instrumentation is independent of the levers, so an A/B logs the height
    // stream from BOTH arms. Value is the tick decimation.
    { const char* e = getenv("CTRL_HGOV_DBG");
      dbg_every = e ? atoi(e) : 0;
      if (e && dbg_every <= 0) dbg_every = 100; }
    _configured = true;
  }

  //! Absolute height backstop: 0.55-0.57 x leg length, where the cheetah runs.
  float floorHeight() const {
    return 0.5f * (ratio_fore + ratio_hind) * leg_len + bias;
  }

  //! The height the robot has been holding for itself lately, m.
  float cruiseHeight() const { return _cruise; }

  //! Predicted height `lead` seconds from now, m. This is the variable
  //! everything triggers on.
  float predictedHeight() const { return _hpred; }

  /*!
   * @param h_meas   measured body height above the stance-foot plane, m.
   *                 This is the state estimator's position[2], which the
   *                 linear KF builds from stance-leg kinematics - NOT sim
   *                 ground truth. It is a real on-robot quantity.
   * @param nominal  the reference the controller would have used, m.
   * @param dt       controller timestep, s.
   * @param settled  false while the entry height ramp is still running; the
   *                 loop stays disarmed and tracks `nominal` so it hands back
   *                 exactly stock behaviour until locomotion is established.
   */
  void update(float h_meas, float nominal, float dt, bool settled) {
    if (!_configured) configureFromEnv();

    // OBSERVE UNCONDITIONALLY. The departure estimate is instrumentation as
    // much as control, and an A/B is worthless if only one arm reports the
    // quantity under test.
    const bool valid = h_meas > 0.05f;
    if (valid) {
      if (!(_cruise > 0.05f)) { _cruise = h_meas; _hf = h_meas; _hlast = h_meas; }
      _hf += (h_meas - _hf) * std::min(1.f, dt / bob_tc);   // de-bob first
      const float raw_d = (_hf - _hlast) / std::max(1e-4f, dt);
      _hlast = _hf;
      _dhdt += (raw_d - _dhdt) * std::min(1.f, dt / d_tc);
      _hpred = _hf + lead * _dhdt;
      // Asymmetric: follow rises, resist falls.
      const float tc = (_hf > _cruise) ? cruise_up_tc : cruise_dn_tc;
      _cruise += (_hf - _cruise) * std::min(1.f, dt / tc);
      _depart  = _cruise - _hpred;                 // >0 = predicted below cruise
      _absSag  = floorHeight() - _hpred;           // >0 = predicted below floor
    }

    const bool armed = enabled && settled && valid;
    if (!armed) {
      _ref = nominal;
      // Bleed any derate off rather than dropping it, so disarming mid-run
      // (gait change, FSM transition) cannot step the velocity command.
      _scale += (1.f - _scale) * std::min(1.f, dt / derate_tc);
    } else {
      if (!(_ref > 0.05f)) _ref = nominal;   // first armed tick

      // Severity: worst of "falling away from my own cruise" and "below what
      // any healthy run or any cheetah sits at".
      auto ramp = [](float x, float lo, float hi) {
        if (x <= lo) return 0.f;
        return std::min(1.f, (x - lo) / std::max(1e-3f, hi - lo));
      };
      const float sev = std::max(ramp(_depart, d_soft,   d_hard),
                                 ramp(_absSag, abs_soft, abs_hard));

      // ---- lever 1: trim the reference UP only ----
      if (_depart > 0.f) {
        _ref += std::min(ki * _depart * dt, slew * dt);
      } else {
        _ref += (nominal - _ref) * std::min(1.f, dt / relax_tc);
      }
      _ref = std::max(nominal, std::min(ref_hi, _ref));

      // ---- lever 2: derate speed ----
      const float want = 1.f - derate * sev;
      _scale += (want - _scale) * std::min(1.f, dt / derate_tc);
      if (sev > 0.f) _fired = true;
    }

    if (dbg_every > 0 && ++_dbg % dbg_every == 0) {
      printf("[HGOV] h=%.3f hf=%.3f cru=%.3f dh=%+.2f pred=%.3f dep=%+.3f "
             "ref=%.3f scale=%.2f %s\n",
             h_meas, _hf, _cruise, _dhdt, _hpred, _depart, _ref, _scale,
             armed ? "armed" : "-");
      fflush(stdout);
    }
  }

  //! Trimmed body-height reference for the MPC, m.
  float reference() const { return _ref; }

  //! Multiply the forward velocity COMMAND by this (apply before the command
  //! filter, so the derate arrives as a ramp and not as a step).
  float speedScale() const { return _scale; }

  //! True once either lever has acted at all this run - so a sweep can tell
  //! "the idea did not help" apart from "the idea never ran".
  bool fired() const { return _fired; }

  //! Nose-down pitch reference, rad. Positive = nose down in the body frame
  //! used here (rotation about +y, x forward / y left / z up).
  float pitchBias() const {
    if (pitch_k <= 1e-6f) return 0.f;
    const float dz = (ratio_hind - ratio_fore) * leg_len;   // hind above fore
    return pitch_k * std::atan2(dz, kHipSeparation);
  }

  //! Predicted metres below the robot's own cruise height. Positive means it
  //! is on its way down. Exposed so the harness can log it.
  double departure() const { return _depart; }

 private:
  static constexpr float kHipSeparation = 0.3762f;   //!< Go1 fore-aft hip span
  bool  _configured = false;
  float _ref    = 0.f;
  float _cruise = 0.f;
  float _hf     = 0.f;
  float _hlast  = 0.f;
  float _dhdt   = 0.f;
  float _hpred  = 0.f;
  float _depart = 0.f;
  float _absSag = 0.f;
  float _scale  = 1.f;
  bool  _fired  = false;
  int   _dbg    = 0;
};

#endif  // CHEETAH_HEIGHT_GOVERNOR_H
