/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */

#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "ControlParameters/RobotParameters.h"
#include "Controllers/LegController.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/VisualizationData.h"
#include "state_estimator_lcmt.hpp"

/*!
 * Result of state estimation
 */
template <typename T>
struct StateEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<T> contactEstimate;
  Vec3<T> position;
  Vec3<T> vBody;
  Quat<T> orientation;
  Vec3<T> omegaBody;
  RotMat<T> rBody;
  Vec3<T> rpy;

  Vec3<T> omegaWorld;
  Vec3<T> vWorld;
  Vec3<T> aBody, aWorld;

  void setLcm(state_estimator_lcmt& lcm_data) {
    for(int i = 0; i < 3; i++) {
      lcm_data.p[i] = position[i];
      lcm_data.vWorld[i] = vWorld[i];
      lcm_data.vBody[i] = vBody[i];
      lcm_data.rpy[i] = rpy[i];
      lcm_data.omegaBody[i] = omegaBody[i];
      lcm_data.omegaWorld[i] = omegaWorld[i];
    }

    for(int i = 0; i < 4; i++) {
      lcm_data.quat[i] = orientation[i];
    }
  }
};

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */
/*!
 * ABSOLUTE POSITION AIDING (baro / GPS).
 *
 * MIT's LinearKF fuses IMU with LEG ODOMETRY, and leg odometry is a RELATIVE
 * source that is weighted per foot by a contact `trust` which goes to zero in
 * swing. During an all-swing window there is therefore NO position measurement
 * at all: p and v become pure double-integration of the accelerometer, the
 * covariance grows without bound, and the estimate goes non-finite (this port
 * carries a NaN guard in RobotRunner for exactly that). It is worst for the
 * gaits with real flight phases - precisely the ones that need it most.
 *
 * Baro and GPS are ABSOLUTE and do not care about contact, so they bound the
 * drift the leg odometry cannot. On the real Go1 both arrive over CAN; in the
 * Gazebo SITL they arrive over UDP via gazebo_get_aux(). Same data either way.
 *
 * Resolution honesty: baro is ~0.08 m 1-sigma against gait-scale height changes
 * of 2-5 cm, so this does NOT track the bounce. It BOUNDS the divergence, which
 * is the failure being fixed.
 */
template <typename T>
struct AbsolutePositionAiding {
  bool  haveXY = false;          //!< GPS fix projected to the local tangent plane
  bool  haveZ  = false;          //!< barometric altitude
  Vec3<T> position = Vec3<T>::Zero();   //!< world frame, metres (x=E, y=N, z=up)
  Vec3<T> sigma = Vec3<T>::Ones();      //!< 1-sigma measurement noise per axis
  // GPS VELOCITY (Doppler-derived, from the same NavSat fix as position above -
  // gps_vel[3] has been carried in the sim_sensor_packet and read by
  // gazebo_get_aux() since this port's earliest UDP bridge work, but nothing
  // ever consumed it - "not consumed by Cheetah yet" in rt_gazebo.h). This is
  // a DIFFERENT correction from position aiding above, not a redundant one:
  // position aiding was found to be harmful to locomotion (see CLAUDE.md,
  // "position drift does not destabilize walking; CORRECTING it does") because
  // absolute position is never used by the controller's own tracking cost, so
  // stepping it only confuses the MPC. VELOCITY is different - it feeds the
  // Raibert foothold formula and the MPC's own velocity-tracking cost directly,
  // and it is NOT covariance-suppressed the way position is (MIT's
  // `_P.block(0,0,2,2) /= 10` hack only ever touches the position indices).
  // So a real Kalman correction here has genuine authority, and correcting a
  // WRONG velocity belief is fixing the actual bad feedback, not fighting a
  // harmless one.
  bool  haveVel = false;
  Vec3<T> velocity = Vec3<T>::Zero();   //!< world frame, m/s (x=E, y=N, z=up)
  Vec3<T> velSigma = Vec3<T>::Ones();   //!< 1-sigma measurement noise per axis
};

template <typename T>
struct StateEstimatorData {
  StateEstimate<T>* result;  // where to write the output to
  VectorNavData* vectorNavData;
  CheaterState<double>* cheaterState;
  LegControllerData<T>* legControllerData;
  Vec4<T>* contactPhase;
  RobotControlParameters* parameters;
  AbsolutePositionAiding<T>* absAiding = nullptr;  //!< optional; null = stock MIT
  // ACTUAL measured wall-clock dt for the tick about to run, in seconds -
  // NOT parameters->controller_dt, which is the fixed nominal config value
  // (2ms) every estimator was integrating against regardless of how long a
  // tick actually took. RobotRunner measures the real gap between ticks
  // (the same clock the host-stall detector already uses) and calls
  // StateEstimatorContainer::setActualDt() with it once per tick, BEFORE
  // running the estimators - a pointer, like contactPhase, so every
  // estimator's own (copied-by-value) StateEstimatorData sees updates
  // without needing setData() called again. Null is a valid, deliberate
  // fallback (an estimator that ignores timing, or a caller that never
  // wires this up) - LinearKFPositionVelocityEstimator::run() falls back
  // to the nominal controller_dt when it is.
  T* actualDt = nullptr;
};

/*!
 * All Estimators should inherit from this class
 */
template <typename T>
class GenericEstimator {
 public:
  virtual void run() = 0;
  virtual void setup() = 0;

  void setData(StateEstimatorData<T> data) { _stateEstimatorData = data; }

  virtual ~GenericEstimator() = default;
  StateEstimatorData<T> _stateEstimatorData;
};

/*!
 * Main State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 */
template <typename T>
class StateEstimatorContainer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Construct a new state estimator container
   */
  StateEstimatorContainer(CheaterState<double>* cheaterState,
                          VectorNavData* vectorNavData,
                          LegControllerData<T>* legControllerData,
                          StateEstimate<T>* stateEstimate,
                          RobotControlParameters* parameters) {
    _data.cheaterState = cheaterState;
    _data.vectorNavData = vectorNavData;
    _data.legControllerData = legControllerData;
    _data.result = stateEstimate;
    _phase = Vec4<T>::Zero();
    _data.contactPhase = &_phase;
    _data.parameters = parameters;
    // Sane startup default before the first real measurement lands -
    // matches the nominal config value exactly, so a caller that never
    // calls setActualDt() at all (nothing in the estimator hierarchy
    // requires it) behaves identically to before this existed.
    _actualDt = parameters ? (T)parameters->controller_dt : (T)0.002;
    _data.actualDt = &_actualDt;
  }

  /*!
   * Feed the ACTUAL measured wall-clock duration of the tick about to run,
   * in seconds - see StateEstimatorData::actualDt's comment for why this
   * exists (a stalled host tick is otherwise integrated by every estimator
   * as if only the nominal 2ms had passed). Call once per tick, before
   * run(). A pointer under the hood, so every estimator already added
   * sees the update with no need to re-call setData().
   */
  void setActualDt(T dt) { _actualDt = dt; }

  /*!
   * Run all estimators
   */
  void run(CheetahVisualization* visualization = nullptr) {
    for (auto estimator : _estimators) {
      estimator->run();
    }
    if (visualization) {
      visualization->quat = _data.result->orientation.template cast<float>();
      visualization->p = _data.result->position.template cast<float>();
      // todo contact!
    }
  }

  /*!
   * Get the result
   */
  const StateEstimate<T>& getResult() { return *_data.result; }
  StateEstimate<T> * getResultHandle() { return _data.result; }

  /*!
   * Set the contact phase
   */
  //! Attach absolute position aiding (baro / GPS). Null leaves the filter
  //! exactly as MIT ships it. See AbsolutePositionAiding for why it exists.
  void setAbsoluteAiding(AbsolutePositionAiding<T>* aiding) {
    _data.absAiding = aiding;
    for (auto est : _estimators) est->setData(_data);
  }

  void setContactPhase(Vec4<T>& phase) { 
    *_data.contactPhase = phase; 
  }

  /*!
   * Add an estimator of the given type
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator() {
    auto* estimator = new EstimatorToAdd();
    estimator->setData(_data);
    estimator->setup();
    _estimators.push_back(estimator);
  }

  /*!
   * Remove all estimators of a given type
   * @tparam EstimatorToRemove
   */
  template <typename EstimatorToRemove>
  void removeEstimator() {
    int nRemoved = 0;
    _estimators.erase(
        std::remove_if(_estimators.begin(), _estimators.end(),
                       [&nRemoved](GenericEstimator<T>* e) {
                         if (dynamic_cast<EstimatorToRemove*>(e)) {
                           delete e;
                           nRemoved++;
                           return true;
                         } else {
                           return false;
                         }
                       }),
        _estimators.end());
  }

  /*!
   * Remove all estimators
   */
  void removeAllEstimators() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
    _estimators.clear();
  }

  ~StateEstimatorContainer() {
    for (auto estimator : _estimators) {
      delete estimator;
    }
  }

 private:
  StateEstimatorData<T> _data;
  std::vector<GenericEstimator<T>*> _estimators;
  Vec4<T> _phase;
  T _actualDt;
};

#endif  // PROJECT_STATEESTIMATOR_H
