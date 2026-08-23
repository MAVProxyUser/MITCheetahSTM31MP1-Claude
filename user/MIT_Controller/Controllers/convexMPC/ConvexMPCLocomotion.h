#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <FSM_States/ControlFSMData.h>
#include <SparseCMPC/SparseCMPC.h>
#include "cppTypes.h"
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "Gait.h"
#include <Controllers/HeightGovernor.h>

#include <cstdio>

using Eigen::Array4f;
using Eigen::Array4i;


template<typename T>
struct CMPC_Result {
  LegControllerCommand<T> commands[4];
  Vec4<T> contactPhase;
};

struct CMPC_Jump {
  static constexpr int START_SEG = 6;
  static constexpr int END_SEG = 0;
  static constexpr int END_COUNT = 2;
  bool jump_pending = false;
  bool jump_in_progress = false;
  bool pressed = false;
  int seen_end_count = 0;
  int last_seg_seen = 0;
  int jump_wait_counter = 0;

  void debug(int seg) {
    (void)seg;
    //printf("[%d] pending %d running %d\n", seg, jump_pending, jump_in_progress);
  }

  void trigger_pressed(int seg, bool trigger) {
    (void)seg;
    if(!pressed && trigger) {
      if(!jump_pending && !jump_in_progress) {
        jump_pending = true;
        //printf("jump pending @ %d\n", seg);
      }
    }
    pressed = trigger;
  }

  bool should_jump(int seg) {
    debug(seg);

    if(jump_pending && seg == START_SEG) {
      jump_pending = false;
      jump_in_progress = true;
      //printf("jump begin @ %d\n", seg);
      seen_end_count = 0;
      last_seg_seen = seg;
      return true;
    }

    if(jump_in_progress) {
      if(seg == END_SEG && seg != last_seg_seen) {
        seen_end_count++;
        if(seen_end_count == END_COUNT) {
          seen_end_count = 0;
          jump_in_progress = false;
          //printf("jump end @ %d\n", seg);
          last_seg_seen = seg;
          return false;
        }
      }
      last_seg_seen = seg;
      return true;
    }

    last_seg_seen = seg;
    return false;
  }
};


class ConvexMPCLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters);
  void initialize();
  ~ConvexMPCLocomotion();

  template<typename T>
  void run(ControlFSMData<T>& data);
  bool currently_jumping = false;

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

private:
  void _SetupCommand(ControlFSMData<float> & data);

  float _yaw_turn_rate;
  float _yaw_des;
  float _yaw_rate_ff = 0.f;   //!< proportional heading feedback into the yaw-rate channel

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  // High speed running
  //float _body_height = 0.34;
  float _body_height = 0.29;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  void recompute_timing(int iterations_per_mpc);
  //! True while the command has been ~zero long enough that a dynamic gait
  //! should not be running (ported from Unitree's Legged_sport; MIT has no
  //! equivalent). See the .cpp - the table-patching alternative was measured
  //! and does not work.
  bool zeroVelHold();

  //! Reactive stance-height regulation - see HeightGovernor.h. The planner
  //! feeds it a situational bias through setHeightBias() (free function,
  //! same hook pattern as setEdamp()/setStandUpHeight()).
  HeightGovernor _hgov;
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode);
  void solveDenseMPC(int *mpcTable, ControlFSMData<float> &data);
  void solveSparseMPC(int *mpcTable, ControlFSMData<float> &data);
  void initSparseMPC();
  // ---- GAIT/SPEED PARAMETER SCHEDULER ----------------------------------
  // The best gait segment and swing clearance are NOT constants: they depend on
  // the gait AND on how fast it is being asked to go. Rather than make a human
  // set an environment variable per run (and get it wrong when the robot changes
  // speed mid-mission), the parameters are looked up every tick and applied at
  // points where they cannot cause a discontinuity.
  struct SchedParams { int segMs; float swingH; float vMax; };
  static SchedParams scheduleFor(int gaitNumber, float speedCmd);
  void applySchedule(int gaitNumber, float speedCmd, Gait* activeGait);
  float _swingHLatched[4] = {0.11f, 0.11f, 0.11f, 0.11f};  // per-leg, set at swing start
  int   _segMsCurrent = 0;        // what is actually in force
  int   _segMsPending = 0;        // what the schedule wants, applied at a cycle boundary

  int iterationsBetweenMPC;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait trotting, bounding, pronking, jumping, galloping, standing, trotRunning, walking, walking2, pacing;
  MixedFrequncyGait random, random2;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  // entry height ramp: avoid stepping _body_height at LOCOMOTION entry
  float _entry_height = 0.f;
  float _height_blend = 0.f;
  float _height_ramp_s = 1.0f;

  // ---- asynchronous MPC solve ----------------------------------------------
  // The dense convex-MPC solve measures 60-105 ms on this Cortex-A7 against a
  // 2 ms control period, and it lands on the first tick of LOCOMOTION. Solved
  // inline it stalls the 500 Hz loop for ~30-50 periods; because MIT runs
  // stance at Kp_stance = 0 (all support is MPC/WBC force), the robot simply
  // free-falls through the stall and rolls out. MIT's own hardware runs the MPC
  // asynchronously - roughly 30-40 Hz - while leg control stays at 500 Hz, so
  // this restores their architecture rather than changing their maths: the
  // worker below runs solveDenseMPC's body verbatim on a snapshot, and the
  // control loop keeps applying the most recent solution.
  struct MpcSnapshot {
    float p[3], v[3], q[4], w[3], r[12], yaw, alpha;
    float traj[12 * 36];
    int   table[4 * 36];
    int   horizon;
    float dtMPC;
    Mat3<float> rBody;
    int64_t t_ms;      // steady-clock ms when the snapshot was taken
  };
  MpcSnapshot        _mpcIn;
  // Async solution: WORLD-frame ground-reaction trajectory for the first three
  // MPC segments, plus when its snapshot was taken. MIT's solver already
  // computes forces for every step of the horizon; the stock code only ever
  // read step 0. With a solve that takes 1-2 gait segments, applying step-0
  // forces "now" means applying them one or two segments late - worst exactly
  // at contact switches, which is where the cheater-mode runs fell. The
  // control loop instead indexes this trajectory by ELAPSED TIME since the
  // snapshot and rotates into the body frame with the CURRENT attitude, so
  // solve latency is compensated with data the solver already produced.
  Vec3<float>        _frTraj[3][4];
  int64_t            _snapMs = 0;
  int64_t            _locoEntryMs = 0;   // when this LOCOMOTION episode began
  // pipelined-MPC state: the solution currently being APPLIED (promoted at
  // segment boundaries), and the prefetched next-segment contact table.
  Vec3<float>        _fApplied[4];
  bool               _fAppliedValid = false;
  int                _mpcTableNext[4 * 36] = {0};
  float              _snapDtMPC = 0.045f;
  static int64_t nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
  }
  std::thread        _mpcThread;
  std::mutex         _mpcMtx;
  std::condition_variable _mpcCv;
  bool               _mpcRequest = false;
  bool               _mpcBusy = false;
  std::atomic<bool>  _mpcQuit{false};
  std::atomic<bool>  _mpcHaveSolution{false};
  bool               _mpcAsync = true;
  void _mpcWorker();
  void _runSolve(const MpcSnapshot& in, Vec3<float> frTraj[3][4]);
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  CMPC_Result<float> result;
  float trajAll[12*36];

  MIT_UserParameters* _parameters = nullptr;
  CMPC_Jump jump_state;

  vectorAligned<Vec12<double>> _sparseTrajectory;

  SparseCMPC _sparseCMPC;

};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
