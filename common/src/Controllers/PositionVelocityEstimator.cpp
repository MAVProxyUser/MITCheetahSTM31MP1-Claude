/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include <cmath>
#include <cstdlib>

#include "Controllers/PositionVelocityEstimator.h"
#include "../../../stm32mp1/gazebo/ShmTrace.h"   // [KFHEALTH] diagnostic only

/*!
 * Initialize the state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup() {
  T dt = this->_stateEstimatorData.parameters->controller_dt;
  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(18, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(27, 17) = T(1);
  _C(26, 14) = T(1);
  _C(25, 11) = T(1);
  _C(24, 8) = T(1);
  _P.setIdentity();
  _P = T(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();
  _R0.setIdentity();
}

template <typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator() {}

/*!
 * Run state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::run() {
  // DT-AWARE INTEGRATION. _A/_B/_Q0 were built ONCE in setup() against the
  // fixed nominal controller_dt (2ms) and reused every tick regardless of
  // how long the tick actually took - so a stalled host tick (measured
  // 13-18ms against a 2ms target, see RobotRunner's own host-stall
  // detector) was integrated by this filter as if only 2ms had passed,
  // silently under-integrating position/velocity relative to how far the
  // robot actually moved and understating how much the covariance should
  // have grown. Recomputed here, per tick, from the ACTUAL measured dt
  // RobotRunner feeds in via StateEstimatorContainer::setActualDt() -
  // cheap (a handful of Identity-block scalar multiplies, not a matrix
  // inversion) and touches only the dt-dependent blocks, never _xhat/_P
  // themselves, so a normal on-time tick (dt == nominal) is bit-for-bit
  // the same filter as before this existed.
  //
  // Clamped rather than trusted outright: a genuinely enormous gap (sim
  // paused, a multi-second host freeze, or simply no caller ever having
  // wired setActualDt() up at all - see actualDt's null fallback) would
  // blow the linearization apart if integrated literally, which is a
  // worse failure than mildly under-integrating one tick. Capping at 20x
  // nominal (~40ms at the stock 2ms) means even a serious stall degrades
  // gracefully instead of the filter's estimate diverging outright.
  {
    T dt = this->_stateEstimatorData.actualDt
               ? *this->_stateEstimatorData.actualDt
               : this->_stateEstimatorData.parameters->controller_dt;
    const T dt_nominal = this->_stateEstimatorData.parameters->controller_dt;
    if (!std::isfinite(dt) || dt <= T(0)) dt = dt_nominal;
    dt = std::min(dt, dt_nominal * T(20));
    _dtUsed = dt;
    _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
    _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
    _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
    _Q0.block(3, 3, 3, 3) =
        (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
    _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();
  }

  T process_noise_pimu =
      this->_stateEstimatorData.parameters->imu_process_noise_position;
  T process_noise_vimu =
      this->_stateEstimatorData.parameters->imu_process_noise_velocity;
  T process_noise_pfoot =
      this->_stateEstimatorData.parameters->foot_process_noise_position;
  T sensor_noise_pimu_rel_foot =
      this->_stateEstimatorData.parameters->foot_sensor_noise_position;
  T sensor_noise_vimu_rel_foot =
      this->_stateEstimatorData.parameters->foot_sensor_noise_velocity;
  T sensor_noise_zfoot =
      this->_stateEstimatorData.parameters->foot_height_sensor_noise;

  Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();
  R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) =
      _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<T> g(0, 0, T(-9.81));
  Mat3<T> Rbod = this->_stateEstimatorData.result->rBody.transpose();
  // in old code, Rbod * se_acc + g
  Vec3<T> a = this->_stateEstimatorData.result->aWorld + g; 
  // std::cout << "A WORLD\n" << a << "\n";
  Vec4<T> pzs = Vec4<T>::Zero();
  Vec4<T> trusts = Vec4<T>::Zero();
  Vec3<T> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;
    Quadruped<T>& quadruped =
        *(this->_stateEstimatorData.legControllerData->quadruped);
    Vec3<T> ph = quadruped.getHipLocation(i);  // hip positions relative to CoM
    // hw_i->leg_controller->leg_datas[i].p; 
    Vec3<T> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;
    // hw_i->leg_controller->leg_datas[i].v;
    Vec3<T> dp_rel = this->_stateEstimatorData.legControllerData[i].v;  
    Vec3<T> p_f = Rbod * p_rel;
    Vec3<T> dp_f =
        Rbod *
        (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + i;

    T trust = T(1);
    T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1));
    //T trust_window = T(0.25);
    T trust_window = T(0.2);

    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (T(1) - trust_window)) {
      trust = (T(1) - phase) / trust_window;
    }
    //T high_suspect_number(1000);
    T high_suspect_number(100);

    // FORCE-VALIDITY GATE ($SIM_FORCE_GATE=1), opt-in. Per the IMM-KF paper
    // (Menner & Berntorp, "Simultaneous State Estimation and Contact
    // Detection for Legged Robots", arXiv:2404.03444) - their Eq. 11 computes
    // a "hypothetical" foot force from joint torque via the Jacobian
    // (f = J^-T tau) and uses its physical validity (positive normal force,
    // inside the friction cone) to bias which contact mode is believed,
    // INSTEAD of trusting a fixed schedule. This port's sim applies commanded
    // torque directly (no actuator dynamics), so LegController's tauEstimate
    // is a faithful proxy for applied torque, not just a command - the same
    // ingredient the paper needs, already computed for every leg.
    //
    // This does NOT replace the existing phase-based trust ramp the way the
    // earlier $SIM_CONTACT_DETECT attempt did (that one overwrote the smooth
    // ramp with a two-level signal and regressed walking2 from 21m to 5.6m -
    // see CLAUDE.md). It only additionally derates trust when the SCHEDULE
    // says "confidently mid-stance" but the commanded force is NOT physically
    // consistent with real load-bearing contact (non-positive normal force,
    // or grossly outside a friction cone).
    //
    // DEBOUNCED, not single-tick - per Bledt/Wensing/Ingersoll/Kim ("Contact
    // Model Fusion for Event-Based Locomotion", ICRA 2018), even a REAL
    // momentum-based disturbance observer measuring actual applied torque is
    // "a large amount of noise from the force estimate" during high-dynamic
    // transients (their Fig 3: 4-8 N RMS error during swing on real hardware),
    // and their own event-based FSM adds an explicit delay specifically "to
    // prevent fleeting contact from catastrophically affecting the robot's
    // gait" from a single bad reading. First cut of this gate reacted to one
    // instantaneous reading and immediately cut trust 80% - tested against
    // galloping's dash, it fell during gait ENGAGEMENT (before nav even took
    // the stick), while the identical run with the gate off did not. One
    // sample each is not proof, but the timing matches exactly the failure
    // mode this paper's own data predicts, so the gate is fixed to require
    // the invalidity to PERSIST for $SIM_FORCE_GATE_DEBOUNCE_MS (default 30 ms,
    // roughly this port's own control-loop equivalent of the paper's 4-5ms
    // hardware detection delay, scaled up for this being commanded torque at
    // the port's own state-estimator rate rather than a filtered observer)
    // before touching trust at all, rather than a single-tick threshold.
    {
      static const bool forceGate =
          getenv("SIM_FORCE_GATE") && atoi(getenv("SIM_FORCE_GATE")) != 0;
      static const T debounceS =
          T((getenv("SIM_FORCE_GATE_DEBOUNCE_MS")
                 ? atof(getenv("SIM_FORCE_GATE_DEBOUNCE_MS"))
                 : 30.0) /
            1000.0);
      static T invalidFor[4] = {0, 0, 0, 0};
      if (forceGate && trust > T(0.5)) {
        const Mat3<T>& J = this->_stateEstimatorData.legControllerData[i].J;
        const Vec3<T>& tau =
            this->_stateEstimatorData.legControllerData[i].tauEstimate;
        // f (body frame) solves J^T f = tau, the inverse of the assembly
        // LegController::updateCommand uses to turn a commanded foot force
        // into joint torque (legTorque += J^T * footForce).
        Vec3<T> f_body = J.transpose().fullPivHouseholderQr().solve(tau);
        Vec3<T> f_world = Rbod * f_body;
        T fz = f_world(2);
        T fxy = std::sqrt(f_world(0) * f_world(0) + f_world(1) * f_world(1));
        const T fzMin = T(2.0);   // N - well below any real stance load
        const T muGate = T(2.0); // matches this port's own sim foot friction
        bool physicallyValid = (fz > fzMin) && (fxy < muGate * fz);
        if (physicallyValid) {
          invalidFor[i] = T(0);
        } else {
          invalidFor[i] += this->_dtUsed;
          if (invalidFor[i] > debounceS) {
            trust *= T(0.2);
          }
        }
      } else {
        invalidFor[i] = T(0);
      }
    }

    // printf("Trust %d: %.3f\n", i, trust);
    Q.block(qindex, qindex, 3, 3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

    trusts(i) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  // [LEGVEL] ($SIM_LEGVEL_DBG=1) - the discriminating diagnostic for the
  // galloping ~10% velocity/position scale under-read (see CLAUDE.md).
  // Two hypotheses produce the same fused symptom and need different fixes:
  //   H-measurement: the raw leg-odometry velocity (-dp_f) is itself ~10%
  //     low during confident stance (foot slip under galloping's high
  //     per-foot forces, or kinematics sampled while the scheduled-stance
  //     foot is not truly loaded) -> fix belongs at the sensor/fusion level
  //     (e.g. GPS velocity aiding, built for exactly this).
  //   H-blending: the measurement is faithful but the trust-ramp blend
  //     drags the fused state toward its own prior at galloping's fast
  //     stance edges -> fix belongs in the trust/ramp shape.
  // Logs, at 10 Hz, each leg's trust and measured world-x velocity (-dp_f)
  // beside the fused vx - lined up against $SIM_ESTERR's ground truth at
  // the same timestamps, one of the two hypotheses dies.
  {
    static const bool legdbg =
        getenv("SIM_LEGVEL_DBG") && atoi(getenv("SIM_LEGVEL_DBG")) != 0;
    if (legdbg) {
      static int nlv = 0;
      static double lvElapsed = 0.0;
      lvElapsed += _dtUsed;
      if ((nlv++ % 50) == 0) {
        // reconstruct each leg's -dp_f x-term from _vs is not possible after
        // the blend, so recompute cheaply from the stored measurement vector:
        // _vs holds the BLENDED value; log it per leg plus trust, and the
        // fused prior v0 - the blend equation lets the raw measurement be
        // recovered offline: meas = (vs - (1-trust)*v0) / trust  (trust>0).
        shmtrace::logf(lvElapsed,
               "[LEGVEL] vx_fused_prior=%.3f "
               "L0 t=%.2f vs=%.3f  L1 t=%.2f vs=%.3f  "
               "L2 t=%.2f vs=%.3f  L3 t=%.2f vs=%.3f",
               (double)v0[0],
               (double)trusts(0), (double)_vs[0],
               (double)trusts(1), (double)_vs[3],
               (double)trusts(2), (double)_vs[6],
               (double)trusts(3), (double)_vs[9]);
      }
    }
  }

  Eigen::Matrix<T, 28, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<T, 18, 18> At = _A.transpose();
  Eigen::Matrix<T, 18, 18> Pm = _A * _P * At + Q;
  Eigen::Matrix<T, 18, 28> Ct = _C.transpose();
  Eigen::Matrix<T, 28, 1> yModel = _C * _xhat;
  Eigen::Matrix<T, 28, 1> ey = y - yModel;
  Eigen::Matrix<T, 28, 28> S = _C * Pm * Ct + R;

  // todo compute LU only once
  Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<T, 18, 18> Pt = _P.transpose();
  _P = (_P + Pt) / T(2);

  // DIAGNOSTIC ($SIM_KF_HEALTH=1) for the "estimator diverges after ~35-90s
  // regardless of gait" investigation (see CLAUDE.md). This filter uses the
  // simple (non-Joseph-form) covariance update in single precision (T=float)
  // for tens of thousands of ticks per run - a well-known setup for _P to
  // lose positive-semi-definiteness to accumulated round-off over a long
  // enough run, which would produce exactly the observed signature (position
  // stalling while velocity readout becomes internally inconsistent/noisy).
  // A valid covariance matrix can never have a negative diagonal entry -
  // this is an objective, no-judgment-call test, not a heuristic.
  //
  // EXTENDED to also print the VELOCITY block (indices 3-5), never checked
  // before now. The position block (0-1) is KNOWN to collapse toward zero -
  // that is MIT's own deliberate /=10 suppression a few dozen lines down,
  // not a bug. Velocity has no such suppression, so if IT ALSO collapses
  // toward zero over a long enough run, that is a genuine, unintended
  // covariance-starvation bug: once P's velocity block is tiny, the Kalman
  // gain on every subsequent leg-odometry velocity measurement
  // (K = P/(P+R), computed fresh every tick from the CURRENT _xhat/_P, not
  // cached) is also tiny, so the filter stops listening to new evidence and
  // free-runs on whatever velocity belief it already had - consistent with
  // this file's own ESTERR finding that the estimate stays positive for the
  // whole run while ground truth shows real velocity going negative: an
  // over-confident filter has no mechanism to notice it is wrong. approxGain
  // is the STEADY-STATE single-axis gain a fresh scalar KF would have at
  // this P and the CURRENT per-tick effective R (post trust-derating) - not
  // the literal 18x28 gain actually used, but enough to see whether it is
  // heading toward zero.
  {
    static const bool kfHealth = getenv("SIM_KF_HEALTH") && atoi(getenv("SIM_KF_HEALTH")) != 0;
    if (kfHealth) {
      static double kfElapsed = 0.0;
      kfElapsed += _dtUsed;
      T minDiag = _P(0, 0);
      int minIdx = 0;
      for (int d = 1; d < 18; ++d) {
        if (_P(d, d) < minDiag) { minDiag = _P(d, d); minIdx = d; }
      }
      const T pvx = _P(3, 3), pvy = _P(4, 4), pvz = _P(5, 5);
      // Representative effective velocity measurement noise for a TRUSTED
      // (trust=1) foot: R.block(rindex2,...) with the (1-trust)*100 term
      // zeroed out, i.e. just sensor_noise_vimu_rel_foot itself.
      const T r_trusted =
          this->_stateEstimatorData.parameters->foot_sensor_noise_velocity;
      const T approxGainVx = pvx / (pvx + r_trusted);
      static int nkf = 0;
      const bool bad = minDiag < T(0);
      if (bad || (nkf++ % 500) == 0) {
        shmtrace::logf(kfElapsed,
               "[KFHEALTH] minDiag=%.6e at idx=%d trace=%.4f p00=%.6f p11=%.6f "
               "pvx=%.6f pvy=%.6f pvz=%.6f approxGainVx=%.6f p00_p11=%.4e%s",
               (double)minDiag, minIdx, (double)_P.trace(),
               (double)_P(0, 0), (double)_P(1, 1),
               (double)pvx, (double)pvy, (double)pvz, (double)approxGainVx,
               (double)(_P(0, 0) * _P(1, 1) - _P(0, 1) * _P(1, 0)),
               bad ? " *** NEGATIVE DIAGONAL - COVARIANCE IS INVALID ***" : "");
      }
    }
  }

  // VELOCITY COVARIANCE FLOOR ($SIM_KF_VFLOOR=<value>, units (m/s)^2), opt-in.
  //
  // $SIM_KF_HEALTH above showed WHY the estimate stays positive for an
  // entire long straight-line run while ESTERR ground truth shows real
  // velocity going negative and never gets corrected: P's velocity block
  // (indices 3-5) collapses from its fresh-start ~0.02-0.2 to ~0.0007-0.002
  // within roughly ONE SECOND of a run starting (measured directly, not
  // inferred), and stays there - not a gradual 35-90s numerical drift as
  // first guessed. This is the filter's own genuine algebraic steady state,
  // not corruption: up to 4 legs supply near-simultaneous, low-noise
  // (foot_sensor_noise_velocity=0.1) velocity pseudo-measurements at 500 Hz
  // against a small per-tick process noise (imu_process_noise_velocity=0.02,
  // Q_vel/tick ~= dt*9.8/20*0.02 ~= 2e-5), and the scalar-KF steady-state
  // relation P^2 ~= Q*(P+R) predicts P_ss ~= sqrt(Q*R_eff) ~= 0.001 for this
  // Q/R pair - matching the measured collapse almost exactly. So this is
  // MIT's own stock tuning doing exactly what it is mathematically supposed
  // to do; the mismatch is that a P this small was only ever exercised on
  // MIT's short real-hardware tests, not on a run long enough (tens of
  // seconds to minutes) for a small UNMODELED bias (foot slip, an imperfect
  // swing/stance transition, whatever) to compound past what a near-zero
  // Kalman gain can ever claw back. Once collapsed, EVERY subsequent
  // measurement's gain is tiny (measured approxGainVx falling from ~0.20 to
  // ~0.007-0.017) and the filter has no built-in mechanism to ever revisit
  // its own belief - this is classic Kalman-filter overconfidence, the same
  // failure class the IMM-KF paper (Menner & Berntorp, arXiv:2404.03444) and
  // the event-based re-triggering in Bledt/Wensing/Ingersoll/Kim (ICRA 2018)
  // both exist to avoid, just via a full multiple-model/event architecture
  // rather than this filter's single fixed model.
  //
  // The standard, minimal fix for this specific failure mode is covariance
  // inflation: floor the velocity diagonal after every update so the filter
  // always retains SOME minimum ongoing authority to correct toward new leg-
  // odometry evidence, no matter how long it has already been running - at
  // the cost of a slightly noisier tick-to-tick estimate. Opt-in and off by
  // default (0 = MIT's stock behaviour, bit-for-bit unchanged) specifically
  // so this can be A/B tested against the exact confirmed failure case
  // (trotRunning's dash, real ESTERR ground truth) before ever being
  // considered for promotion - see CLAUDE.md for the measured result.
  {
    static const T vFloor =
        getenv("SIM_KF_VFLOOR") ? (T)atof(getenv("SIM_KF_VFLOOR")) : T(0);
    if (vFloor > T(0)) {
      for (int ax = 3; ax < 6; ++ax) {
        if (_P(ax, ax) < vFloor) _P(ax, ax) = vFloor;
      }
    }
  }

  // MIT SUPPRESSES THE x,y POSITION COVARIANCE EVERY TICK.
  //
  // This zeroes the cross-covariance between horizontal position and the rest
  // of the state and then divides the x,y block by 10, so _P(0,0) and _P(1,1)
  // are driven toward zero no matter what the filter actually knows. The
  // consequence is that the estimator is PERMANENTLY CONFIDENT about the one
  // quantity it never observes: absolute position is not measurable from leg
  // odometry (which is relative) plus IMU, so the true error dead-reckons
  // upward while the reported covariance stays small.
  //
  // Measured here: 4.50 m of position error after 83 m of walking (5.4%), with
  // _P small throughout - so an absolute-position Kalman update computes
  // K = P/(P+R) ~ 0 and GPS gets no authority at all.
  //
  // $SIM_KF_UNCAP=1 skips the suppression so the covariance can grow to reflect
  // genuine unobservability, which is what makes GPS aiding work as a real
  // Kalman update rather than a bolted-on complementary filter. Default keeps
  // MIT's behaviour so nothing silently changes underneath existing results.
  {
    static const bool uncap = getenv("SIM_KF_UNCAP") &&
                              atoi(getenv("SIM_KF_UNCAP")) != 0;
    if (!uncap) {
      if (_P.block(0, 0, 2, 2).determinant() > T(0.000001)) {
        _P.block(0, 2, 2, 16).setZero();
        _P.block(2, 0, 16, 2).setZero();
        _P.block(0, 0, 2, 2) /= T(10);
      }
    }
  }

  // ---- ABSOLUTE POSITION AIDING (baro / GPS) -------------------------------
  // Sequential Kalman update, applied after the leg-odometry update above. Done
  // sequentially rather than by widening MIT's fixed-size 18x28 C/R matrices:
  // it is mathematically the same thing for independent measurements, and it
  // leaves the stock filter untouched when no aiding is present.
  //
  // WHY: the update above weights each foot by a contact `trust` that goes to
  // zero in swing, so an all-swing window leaves p and v with NO measurement -
  // pure accelerometer double-integration, covariance diverging, estimate going
  // non-finite (see the NaN guard in RobotRunner). Leg odometry is relative and
  // vanishes exactly when a flight gait needs it; baro and GPS are absolute and
  // do not care about contact.
  //
  //   z = p_abs,  H = [I3 0 0],  K = P H^T (H P H^T + R)^-1
  {
    auto* aid = this->_stateEstimatorData.absAiding;
    if (getenv("SIM_AID_DBG")) {
      static int once = 0;
      if ((once++ % 2000) == 0) {
        printf("[AID-PTR] estimator sees absAiding=%p haveXY=%d\n",
               (void*)aid, aid ? (int)aid->haveXY : -1);
        fflush(stdout);
      }
    }
    if (aid && (aid->haveXY || aid->haveZ)) {
      // Per-axis: only correct the axes we actually have a measurement for.
      for (int ax = 0; ax < 3; ++ax) {
        const bool have = (ax < 2) ? aid->haveXY : aid->haveZ;
        if (!have) continue;
        const T sig = aid->sigma[ax] > T(1e-4) ? aid->sigma[ax] : T(1e-4);
        const T R_ax = sig * sig;

        const T innov = aid->position[ax] - _xhat[ax];
        if (!std::isfinite(innov)) continue;

        // With $SIM_KF_UNCAP=1 the covariance is allowed to grow honestly, so
        // a proper Kalman update has real gain and is the correct estimator.
        // Without it, MIT's suppression makes K ~ 0 and only the time-constant
        // form below does anything.
        static const bool kfForm = getenv("SIM_KF_UNCAP") &&
                                   atoi(getenv("SIM_KF_UNCAP")) != 0;
        if (kfForm) {
          const T S_ax = _P(ax, ax) + R_ax;
          if (S_ax > T(1e-12) && std::isfinite(S_ax)) {
            Eigen::Matrix<T, 18, 1> K = _P.col(ax) / S_ax;
            const T inn = aid->position[ax] - _xhat[ax];
            if (std::isfinite(inn)) {
              _xhat += K * inn;
              _P -= K * _P.row(ax);
              if (getenv("SIM_AID_DBG")) {
                static int d2 = 0;
                if (ax == 1 && (d2++ % 500) == 0) {
                  printf("[AID-KF] axis=%d est=%.2f meas=%.2f innov=%.2f "
                         "P=%.6f K=%.6f\n", ax, (double)_xhat[ax],
                         (double)aid->position[ax], (double)inn,
                         (double)_P(ax, ax), (double)K[ax]);
                  fflush(stdout);
                }
              }
            }
          }
          continue;
        }

        // A textbook Kalman update here is INERT, and the reason is worth
        // recording: MIT caps the x,y position covariance every tick -
        //     if (_P.block(0,0,2,2).determinant() > 1e-6) _P.block(0,0,2,2) /= 10;
        // - so the filter is permanently overconfident about the one quantity
        // it never actually observes. Absolute position is NOT observable from
        // leg odometry (which is relative) plus IMU: the estimate dead-reckons
        // and its true error grows without bound while _P stays small. With
        // _P(ax,ax) tiny, K = P/(P+R) ~ 0 and GPS gets no authority at all.
        // Measured: 4.50 m of drift over 83 m (5.4%), IDENTICAL with the
        // Kalman-form aiding switched on.
        //
        // So drive the correction from a time constant instead of from a
        // covariance that has been suppressed. tau is how long it takes to wash
        // out an absolute error; it must be long compared with the gait period
        // so per-step odometry still dominates the short term.
        static const T tau = getenv("SIM_AID_TAU") ? (T)atof(getenv("SIM_AID_TAU"))
                                                   : T(2.0);
        // was a hardcoded T(0.002) (the nominal dt) regardless of how long
        // this tick actually took - now the same measured/clamped dt the
        // predict step above uses, so a stalled tick washes out an
        // absolute error proportionally more, not by the nominal amount.
        const T k = _dtUsed / (tau > T(1e-3) ? tau : T(1e-3));   // dt / tau
        _xhat[ax] += k * innov;
        if (getenv("SIM_AID_DBG")) {
          static int dbg = 0;
          if (ax == 1 && (dbg++ % 500) == 0) {
            printf("[AID] axis=%d est=%.2f meas=%.2f innov=%.2f k=%.5f\n",
                   ax, (double)_xhat[ax], (double)aid->position[ax],
                   (double)innov, (double)k);
            fflush(stdout);
          }
        }
        // Let the velocity state feel a fraction of it too, so a persistent
        // offset is corrected rather than fought every tick.
        _xhat[3 + ax] += T(0.1) * k * innov;
      }
      Eigen::Matrix<T, 18, 18> Pa = _P.transpose();
      _P = (_P + Pa) / T(2);
    }
  }

  // ---- GPS VELOCITY AIDING ---------------------------------------------
  // A textbook sequential Kalman update, same structure as the position
  // aiding above, but deliberately NOT the same code path - see the
  // derivation in the header comment on AbsolutePositionAiding::haveVel for
  // why this is expected to behave differently, not just "more of the same
  // fix". The short version: MIT's covariance-suppression hack
  // (`_P.block(0,0,2,2) /= 10`, a few dozen lines up) targets indices 0-1
  // (x,y POSITION) exclusively - the velocity block (indices 3-5) is never
  // touched by it, so its covariance evolves normally and a real Kalman
  // gain here has genuine authority, unlike position aiding's K ~ 0 problem.
  //
  //   z = v_abs (from GPS Doppler),  H = [0 I3 0 ...],  K = P H^T (H P H^T + R)^-1
  //
  // This is the exactness/structural claim worth stating plainly per the
  // control-math-verification discipline: this update does not correct
  // POSITION at all (H has zero columns there), so it cannot reproduce the
  // "GPS position aiding destabilizes locomotion" failure mode already
  // measured and documented - a genuinely different mechanism, not an
  // unverified hope that it is different.
  {
    auto* aid = this->_stateEstimatorData.absAiding;
    if (aid && aid->haveVel) {
      for (int ax = 0; ax < 3; ++ax) {
        const T sig = aid->velSigma[ax] > T(1e-4) ? aid->velSigma[ax] : T(1e-4);
        const T R_ax = sig * sig;
        const int si = 3 + ax;  // velocity state index in _xhat/_P
        const T innov = aid->velocity[ax] - _xhat[si];
        if (!std::isfinite(innov)) continue;
        const T S_ax = _P(si, si) + R_ax;
        if (!(S_ax > T(1e-12)) || !std::isfinite(S_ax)) continue;
        Eigen::Matrix<T, 18, 1> K = _P.col(si) / S_ax;
        _xhat += K * innov;
        _P -= K * _P.row(si);
        if (getenv("SIM_AID_DBG")) {
          static int dbg = 0;
          if (ax == 0 && (dbg++ % 500) == 0) {
            printf("[VELAID] axis=%d est=%.3f meas=%.3f innov=%.3f P=%.6f K=%.4f\n",
                   ax, (double)_xhat[si], (double)aid->velocity[ax],
                   (double)innov, (double)_P(si, si), (double)K[si]);
            fflush(stdout);
          }
        }
      }
      Eigen::Matrix<T, 18, 18> Pv = _P.transpose();
      _P = (_P + Pv) / T(2);
    }
  }

  this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
  this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
  this->_stateEstimatorData.result->vBody =
      this->_stateEstimatorData.result->rBody *
      this->_stateEstimatorData.result->vWorld;
}

template class LinearKFPositionVelocityEstimator<float>;
template class LinearKFPositionVelocityEstimator<double>;


/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterPositionVelocityEstimator<T>::run() {
  this->_stateEstimatorData.result->position = this->_stateEstimatorData.cheaterState->position.template cast<T>();
  this->_stateEstimatorData.result->vWorld =
      this->_stateEstimatorData.result->rBody.transpose().template cast<T>() * this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
  this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
}

template class CheaterPositionVelocityEstimator<float>;
template class CheaterPositionVelocityEstimator<double>;
