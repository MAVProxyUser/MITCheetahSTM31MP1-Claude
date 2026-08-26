/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include <cmath>

#include "Controllers/PositionVelocityEstimator.h"

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
