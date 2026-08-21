#include <cstdlib>
#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <pthread.h>
#include <sched.h>
#include <cmath>
#include "ConvexMPCLocomotion.h"

// Per-foot force cap handed to the convex MPC. Mini-cheetah's 120 N suits a
// 9 kg / 88 N robot; the Go1 is 13.1 kg / 128 N and its knee (35.55 Nm over a
// ~0.10-0.15 m stance moment arm) is good for 240-355 N at the foot. At 120 N
// the solver cannot plan the 2-3x bodyweight peaks bounding and galloping need.
#ifdef USE_GO1_MODEL
// MIT uses 120 N for the 9 kg / 88 N mini-cheetah - 1.36x bodyweight per foot.
// Holding that ratio for the 13.1 kg / 128 N Go1 gives 175 N, which is well
// inside what its knee can actually deliver (35.55 Nm over a 0.10-0.15 m stance
// moment arm is 240-355 N at the foot). 250 N was tried and made the stand sit
// lower, so this keeps MIT's tuning rather than inventing a new one.
#define MPC_F_MAX 175
#else
#define MPC_F_MAX 120
#endif
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"

#include "Gait.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH


////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
  iterationsBetweenMPC(_iterations_between_mpc),
  // Horizon is env-tunable ($SIM_MPC_HORIZON) because dense-QP solve cost grows
  // steeply with it, and this board is 30x slower than the x86 UP board MIT
  // shipped on: the 10-step horizon measures 62-68 ms per solve on the A7
  // against a 2 ms control period.
  horizonLength(getenv("SIM_MPC_HORIZON") ? atoi(getenv("SIM_MPC_HORIZON")) : 10),
  dt(_dt),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
  //bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  jumping(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  //trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
  trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, MPC_F_MAX);
  //setup_problem(dtMPC, horizonLength, 0.4, 650); // DH

  // Start the MPC worker (see the header for why the solve cannot run inline on
  // this board). SIM_MPC_ASYNC=0 restores the stock inline behaviour for A/B.
  _mpcAsync = !(getenv("SIM_MPC_ASYNC") && atoi(getenv("SIM_MPC_ASYNC")) == 0);
  if (_mpcAsync) {
    _mpcThread = std::thread(&ConvexMPCLocomotion::_mpcWorker, this);
    printf("[Convex MPC] solve runs ASYNC on a worker thread\n");
  } else {
    printf("[Convex MPC] solve runs INLINE (stock)\n");
  }

  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  initSparseMPC();

   pBody_des.setZero();
   vBody_des.setZero();
   aBody_des.setZero();
}

ConvexMPCLocomotion::~ConvexMPCLocomotion() {
  _mpcQuit.store(true);
  { std::lock_guard<std::mutex> lk(_mpcMtx); _mpcRequest = true; }
  _mpcCv.notify_all();
  if (_mpcThread.joinable()) _mpcThread.join();
}

void ConvexMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
  _height_blend = 0.f;      // restart the entry height ramp (see _SetupCommand)
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
  if(data._quadruped->_robotType == RobotType::MINI_CHEETAH){
#ifdef USE_GO1_MODEL
    _body_height = 0.30;   // Go1 nominal stance (legs 0.426 m vs MC's 0.404)
#else
    _body_height = 0.29;
#endif
#ifdef USE_GO1_MODEL
    // Go1 SITL: allow a crouched gait ($SIM_BODY_H, m) - lower CoM = much
    // larger roll/pitch margins through the UDP loop's extra latency.
    {
      static float h_env = -1.f;
      if (h_env < 0.f) {
        const char* e = getenv("SIM_BODY_H");
        h_env = e ? atof(e) : 0.f;
      }
      if (h_env > 0.05f) _body_height = h_env;
    }
#endif
  }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
    _body_height = 0.45;
  }else{
    assert(false);
  }

  // ---- ENTRY HEIGHT RAMP ----
  // BALANCE_STAND settles the Go1 around 0.21-0.26 m, but locomotion's nominal
  // _body_height is 0.30. Handing the MPC that 9 cm step on the first tick of
  // LOCOMOTION makes it solve for a large upward force and LAUNCH the robot:
  // measured z 0.211 -> 0.342 in ~0.2 s at 0.32 m/s vertical, after which it
  // comes down badly and rolls out (wB peaked at -3.2 rad/s, roll -49 deg,
  // then the FSM bailed to RECOVERY_STAND for the rest of the run). MIT never
  // sees this because a mini-cheetah's balance stand already sits at its
  // locomotion height.
  // So start from the height the robot is ACTUALLY at and ramp to nominal.
  {
    float z_now = data._stateEstimator->getResult().position[2];
    if (_height_blend <= 0.f && z_now > 0.05f) _entry_height = z_now;
    if (_height_blend < 1.f) {
      _height_blend += dt / _height_ramp_s;
      if (_height_blend > 1.f) _height_blend = 1.f;
      _body_height = _entry_height + (_body_height - _entry_height) * _height_blend;
    }
  }

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);
  if(data.controlParameters->use_rc){
    const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;
    data.userParameters->cmpc_gait = rc_cmd->variable[0];
    _yaw_turn_rate = -rc_cmd->omega_des[2];
    x_vel_cmd = rc_cmd->v_des[0];
    y_vel_cmd = rc_cmd->v_des[1] * 0.5;
    _body_height += rc_cmd->height_variation * 0.08;
  }else{
    _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
    x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
    y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];
  }
  _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter;
  _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;

  // STM32MP1 SITL: prove the velocity command actually reaches the MPC.
  if (getenv("STM32MP1_EST_DBG")) {
    static int _vdbg = 0; ++_vdbg;
    if ((_vdbg % 25) == 0) {
      printf("[MPC] gait=%d pad=%.3f stick=%.3f xcmd=%.3f -> xdes=%.3f  yawrate=%.3f  bodyH=%.3f\n",
             gaitNumber,
             data._desiredStateCommand->gamepadCommand
               ? data._desiredStateCommand->gamepadCommand->leftStickAnalog[1] : -9.f,
             data._desiredStateCommand->leftAnalogStick[1],
             x_vel_cmd, _x_vel_des, _yaw_turn_rate, _body_height);
      fflush(stdout);
    }
  }

  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des = 0.;

}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data) {
  bool omniMode = false;

  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;
  // 20+ are this port's additions (walking / walking2 / galloping); they must
  // bypass MIT's omni rewrite, which would otherwise turn 20/21/22 into
  // 10/11/12 and then into 0/1/2.
  if(gaitNumber >= 20) {
    // keep as-is
  } else if(gaitNumber >= 10) {
    gaitNumber -= 10;
    omniMode = true;
  }

  auto& seResult = data._stateEstimator->getResult();

  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.21;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &pronking;
  else if(gaitNumber == 3)
    gait = &random;
  else if(gaitNumber == 4)
    gait = &standing;
  else if(gaitNumber == 5)
    gait = &trotRunning;
  else if(gaitNumber == 6)
    gait = &random2;
  else if(gaitNumber == 7)
    gait = &random2;
  else if(gaitNumber == 8)
    gait = &pacing;
  // MIT defines `walking` and `walking2` but the stock selector stops at 8, so
  // neither is reachable and anything else silently falls through to `trotting`.
  // Both matter here:
  // NUMBERING: MIT reserves >=10 for "same gait, omniMode" - run() does
  //   if(gaitNumber >= 10) { gaitNumber -= 10; omniMode = true; }
  // so 10/11/12 are rewritten to 0/1/2 and silently select trotting/bounding/
  // pronking. The new gaits therefore live at 20+, which survives that
  // subtraction as 10/11/12 only if omni is wanted - so they are matched
  // BEFORE the subtraction instead (see run()).
  //   20 = walking  - 4-beat, offsets (0,3,5,8), one foot down at a time
  //   21 = walking2 - diagonal pairs like a trot but 7/10 duty, i.e. a 40%
  //                   DOUBLE-SUPPORT overlap. A 50%-duty trot is on exactly two
  //                   feet at every instant and free to roll about that
  //                   diagonal the whole time; the overlap is what removes that
  //                   window, and it is the single thing that made this port's
  //                   own hand-rolled trot stable.
  else if(gaitNumber == 20)
    gait = &walking;
  else if(gaitNumber == 21)
    gait = &walking2;
  //   22 = galloping - offsets (0,2,7,9), the only gait here with a real
  //        flight phase, and the one the published 2.5-4.7 m/s figures rely on.
  else if(gaitNumber == 22)
    gait = &galloping;
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);


  jumping.setIterations(27/2, iterationCounter);

  //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
  // check jump trigger
  jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
      data._desiredStateCommand->trigger_pressed);


  // bool too_high = seResult.position[2] > 0.29;
  // check jump action
  if(jump_state.should_jump(jumping.getCurrentGaitPhase())) {
    gait = &jumping;
    recompute_timing(27/2);
    _body_height = _body_height_jumping;
    currently_jumping = true;

  } else {
    recompute_timing(default_iterations_between_mpc);
    currently_jumping = false;
  }

  if(_body_height < 0.02) {
#ifdef USE_GO1_MODEL
    _body_height = 0.30;   // Go1 nominal stance (legs 0.426 m vs MC's 0.404)
#else
    _body_height = 0.29;
#endif
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = 
    omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");

  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
  }

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {

#ifdef USE_GO1_MODEL
      footSwingTrajectories[i].setHeight(0.055);
#else
      footSwingTrajectories[i].setHeight(0.05);
#endif
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    // Make the FIRST control tick land on an MPC solve boundary.
    // iterationCounter++ runs before updateMPCIfNeeded, so with a fresh
    // counter the first solve would otherwise wait iterationsBetweenMPC
    // ticks (~26 ms) - during which f_ff/Fr_des are ZERO and the stance
    // legs have only the soft joint PD: the body free-falls at gait entry,
    // then the catch-up force spike tumbles the robot.
    iterationCounter = iterationsBetweenMPC - 1;
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  //float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {
    //footSwingTrajectories[i].setHeight(.05);
#ifdef USE_GO1_MODEL
    footSwingTrajectories[i].setHeight(.07);   // Go1 leg is 5% longer; give the
                                               // swing real clearance - a foot
                                               // that clips is a trip, and this
                                               // gait fails in roll.
#else
    footSwingTrajectories[i].setHeight(.06);
#endif
#ifdef USE_GO1_MODEL
    // Place swing feet at the robot's own lateral hip offset (abad link). The
    // stock .065 is mini-cheetah's abad link; Go1's is .08, so .065 narrows the
    // stance 1.5 cm every step and erodes roll stability.
    Vec3<float> offset(0, side_sign[i] * data._quadruped->_abadLinkLength, 0);
#else
    Vec3<float> offset(0, side_sign[i] * .065, 0);
#endif

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected = 
      coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;


    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
          + des_vel * swingTimeRemaining[i]);

    //+ seResult.vWorld * swingTimeRemaining[i];

    //float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);

    // NOTE the missing `* dtMPC`. Upstream reads
    //     vWorld[1] * .5 * stance_time * dtMPC
    // while the x term one line up is
    //     vWorld[0] * (.5 + bonus) * stance_time
    // - no dtMPC. Both are the same capture-point quantity (v * T_stance / 2),
    // so the extra factor makes LATERAL foot placement ~22x too weak at
    // dtMPC = 0.045 s. Sideways velocity then goes essentially uncorrected:
    // measured vB_y climbing 0.013 -> 0.466 m/s over ~1 s at gait entry while
    // roll ran away to -0.75 rad and the robot fell over every single time.
    // A mini-cheetah tolerates it; the Go1 does not.
    float pfy_rel = seResult.vWorld[1] * .5 * stance_time +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    Pf[2] = -0.003;
    //Pf[2] = 0.0;
    footSwingTrajectories[i].setFinalPosition(Pf);

  }

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
     0, 700, 0,
     0, 0, 150;
  Kp_stance = 0*Kp;


  Kd << 7, 0, 0,
     0, 7, 0,
     0, 0, 7;
  Kd_stance = Kd;
  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, data, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0,0,0,0);

#ifdef DRAW_DEBUG_PATH
  auto* trajectoryDebug = data.visualizationData->addPath();
  if(trajectoryDebug) {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for(int i = 0; i < 10; i++) {
      trajectoryDebug->position[i][0] = trajAll[12*i + 3];
      trajectoryDebug->position[i][1] = trajAll[12*i + 4];
      trajectoryDebug->position[i][2] = trajAll[12*i + 5];
      auto* ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

#ifdef DRAW_DEBUG_SWINGS
      auto* debugPath = data.visualizationData->addPath();
      if(debugPath) {
        debugPath->num_points = 100;
        debugPath->color = {0.2,1,0.2,0.5};
        float step = (1.f - swingState) / 100.f;
        for(int i = 0; i < 100; i++) {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
          debugPath->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }
      auto* finalSphere = data.visualizationData->addSphere();
      if(finalSphere) {
        finalSphere->position = footSwingTrajectories[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      auto* actualSphere = data.visualizationData->addSphere();
      auto* goalSphere = data.visualizationData->addSphere();
      goalSphere->position = footSwingTrajectories[foot].getPosition();
      actualSphere->position = pFoot[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);


      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      
      if(!data.userParameters->use_wbc){
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

#ifdef DRAW_DEBUG_SWINGS
      auto* actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pFoot[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }else{ // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0.*Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.; 
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update


}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<double>& data) {
  (void)data;
  printf("call to old CMPC with double!\n");

}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode) {
  //iterationsBetweenMPC = 30;
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};


    //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

    if(current_gait == 4)
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)_body_height/*fsm->main_control_settings.p_des[2]*/,
        0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    }

    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = {(float)rpy_comp[0],  // 0
        (float)rpy_comp[1],    // 1
        _yaw_des,    // 2
        //yawStart,    // 2
        xStart,                                   // 3
        yStart,                                   // 4
        (float)_body_height,      // 5
        0,                                        // 6
        0,                                        // 7
        _yaw_turn_rate,  // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        0};                                       // 11

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    Timer solveTimer;

    if(_parameters->cmpc_use_sparse > 0.5) {
      solveSparseMPC(mpcTable, data);
    } else {
      solveDenseMPC(mpcTable, data);
    }
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  // NOTE: setup_problem() is NOT called here any more. It calls
  // resize_qp_mats(), which setZero()s S, fmat, qH, qg and friends - and with
  // the solve now on a worker thread, doing that from the control thread wipes
  // the matrices the worker is mid-way through building. That is exactly what
  // happened: the worker's problem came out with |S| = 0 and |fmat| = 0, i.e.
  // no state cost and no friction constraints, so the QP reduced to
  // min alpha*||u||^2 and both solvers correctly returned ZERO force.
  // The worker calls setup_problem itself, on its own thread, before each solve.
  if (!_mpcAsync) setup_problem(dtMPC,horizonLength,0.4,MPC_F_MAX);
  update_x_drag(x_comp_integral);
  if(vxy[0] > 0.3 || vxy[0] < -0.3) {
    //x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
      _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  //t1.stopPrint("Setup MPC");

  // ---- hand the solve to the worker, keep using the latest solution --------
  // Everything above (weights, alpha, x_drag, solver settings) is unchanged
  // MIT; only WHERE the solve runs changes. setup_problem/update_problem_data/
  // get_solution touch solver globals and are not re-entrant, so ONLY the
  // worker ever calls them.
  if (!_mpcAsync) {                    // stock behaviour: solve right here
    MpcSnapshot in;
    for (int i = 0; i < 3; ++i) { in.p[i]=p[i]; in.v[i]=v[i]; in.w[i]=w[i]; }
    for (int i = 0; i < 4; ++i) in.q[i]=q[i];
    for (int i = 0; i < 12; ++i) in.r[i]=r[i];
    in.yaw=yaw; in.alpha=alpha; in.horizon=horizonLength; in.dtMPC=dtMPC;
    in.rBody=seResult.rBody;
    for (int i = 0; i < 12*horizonLength; ++i) in.traj[i]=trajAll[i];
    for (int i = 0; i < 4*horizonLength; ++i) in.table[i]=mpcTable[i];
    _runSolve(in, f_ff, Fr_des);
    return;
  }
  {
    std::unique_lock<std::mutex> lk(_mpcMtx);
    if (!_mpcBusy) {
      for (int i = 0; i < 3; ++i) { _mpcIn.p[i]=p[i]; _mpcIn.v[i]=v[i]; _mpcIn.w[i]=w[i]; }
      for (int i = 0; i < 4; ++i) _mpcIn.q[i]=q[i];
      for (int i = 0; i < 12; ++i) _mpcIn.r[i]=r[i];
      _mpcIn.yaw = yaw;
      _mpcIn.alpha = alpha;
      _mpcIn.horizon = horizonLength;
      _mpcIn.dtMPC = dtMPC;
      _mpcIn.rBody = seResult.rBody;
      int ntraj = 12 * horizonLength, ntab = 4 * horizonLength;
      if (ntraj > (int)(sizeof(_mpcIn.traj)/sizeof(float))) ntraj = sizeof(_mpcIn.traj)/sizeof(float);
      if (ntab  > (int)(sizeof(_mpcIn.table)/sizeof(int)))  ntab  = sizeof(_mpcIn.table)/sizeof(int);
      for (int i = 0; i < ntraj; ++i) _mpcIn.traj[i] = trajAll[i];
      for (int i = 0; i < ntab;  ++i) _mpcIn.table[i] = mpcTable[i];
      _mpcRequest = true;
      _mpcBusy = true;
      _mpcCv.notify_one();
    }
    // BOOTSTRAP. The first solve takes 73-240 ms on this board, and until it
    // lands f_ff is zero - with MIT's Kp_stance = 0 that is zero support, so
    // the robot collapses before its first MPC solution ever arrives and every
    // later solve then sees a robot already on the floor (which is why the
    // solutions came back all-zero). Hold static equilibrium until then:
    // bodyweight shared over the feet the gait currently calls stance. This is
    // the same quantity BALANCE_STAND is already holding the robot up with, so
    // it is continuous across the transition.
    if (!_mpcHaveSolution.load()) {
      int nStance = 0;
      for (int leg = 0; leg < 4; ++leg) if (mpcTable[leg]) nStance++;
      if (nStance < 1) nStance = 1;
      const float W = 13.1f * 9.81f;
      for (int leg = 0; leg < 4; ++leg) {
        Vec3<float> fw(0.f, 0.f, mpcTable[leg] ? (W / (float)nStance) : 0.f);
        f_ff[leg]   = -seResult.rBody * fw;
        Fr_des[leg] = fw;
      }
    }
    // publish whatever the worker has finished most recently
    if (_mpcHaveSolution.load()) {
      for (int leg = 0; leg < 4; ++leg) {
        f_ff[leg]   = _f_ff_async[leg];
        Fr_des[leg] = _Fr_des_async[leg];
      }
    }
  }
}

//! The original MIT solve, run on the worker thread against a snapshot.
void ConvexMPCLocomotion::_runSolve(const MpcSnapshot& in,
                                    Vec3<float>* f_ff_out, Vec3<float>* fr_out) {
  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};
  setup_problem(in.dtMPC, in.horizon, 0.4, MPC_F_MAX);
  update_x_drag(x_comp_integral);
  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
      _parameters->jcqp_sigma, _parameters->jcqp_alpha,
      _parameters->jcqp_terminate, _parameters->use_jcqp);
  if (getenv("STM32MP1_MPC_IN")) {
    static int _ic = 0;
    if ((++_ic % 10) == 1) {
      printf("[MPCIN] p=%.2f %.2f %.2f  v=%.2f %.2f %.2f  yaw=%.2f h=%d\n",
             in.p[0], in.p[1], in.p[2], in.v[0], in.v[1], in.v[2], in.yaw, in.horizon);
      printf("[MPCIN] table[0..7]=%d%d%d%d %d%d%d%d  traj[0..5]=%.2f %.2f %.2f %.2f %.2f %.2f\n",
             in.table[0],in.table[1],in.table[2],in.table[3],
             in.table[4],in.table[5],in.table[6],in.table[7],
             in.traj[0],in.traj[1],in.traj[2],in.traj[3],in.traj[4],in.traj[5]);
      printf("[MPCIN] r(foot rel CoM) x=%.2f %.2f %.2f %.2f  z=%.2f %.2f %.2f %.2f\n",
             in.r[0],in.r[1],in.r[2],in.r[3], in.r[8],in.r[9],in.r[10],in.r[11]);
      fflush(stdout);
    }
  }
  update_problem_data_floats((float*)in.p, (float*)in.v, (float*)in.q, (float*)in.w,
                             (float*)in.r, in.yaw, Q, (float*)in.traj,
                             in.alpha, (int*)in.table);
  for (int leg = 0; leg < 4; ++leg) {
    Vec3<float> f;
    for (int axis = 0; axis < 3; ++axis) f[axis] = get_solution(leg*3 + axis);
    f_ff_out[leg] = -in.rBody * f;
    fr_out[leg]   = f;
  }
}

void ConvexMPCLocomotion::_mpcWorker() {
  // The worker MUST NOT compete with the 500 Hz control loop. It inherits the
  // creator's SCHED_FIFO priority (49 here), and with the control loop and the
  // motor task also at FIFO 49 on a DUAL-core A7, an equal-priority 60 ms solve
  // preempts the very thing it was supposed to protect - moving the solve off
  // the control thread bought nothing at all until this was fixed.
  // Drop to SCHED_OTHER so the RT threads always win, and pin to the second
  // core so the solve does not evict the control loop's cache either.
  {
    // Priority: BELOW the 500 Hz control loop and motor task (both FIFO 49) so
    // it can never preempt them, but still real-time so it is not starved to
    // death by them - as SCHED_OTHER was: one solve took 359 ms instead of 60,
    // and the run completed a single MPC solve in 28 s.
    int prio = getenv("SIM_MPC_PRIO") ? atoi(getenv("SIM_MPC_PRIO")) : 20;
    struct sched_param sp; sp.sched_priority = prio;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      sp.sched_priority = 0;
      pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
      printf("[Convex MPC] worker: SCHED_OTHER (FIFO %d refused)\n", prio);
    } else {
      printf("[Convex MPC] worker: SCHED_FIFO %d\n", prio);
    }
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(1, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    fflush(stdout);
  }
  Vec3<float> f_local[4], fr_local[4];
  while (!_mpcQuit.load()) {
    MpcSnapshot in;
    {
      std::unique_lock<std::mutex> lk(_mpcMtx);
      _mpcCv.wait(lk, [&]{ return _mpcRequest || _mpcQuit.load(); });
      if (_mpcQuit.load()) return;
      in = _mpcIn;
      _mpcRequest = false;
    }
    Timer _tsolve;
    _runSolve(in, f_local, fr_local);
    // NaN GUARD. An ill-conditioned or diverging QP can return non-finite
    // forces; those propagate through J^T into the joint torques and Gazebo
    // rejects them ("Invalid joint force value [nan]"). On real hardware a NaN
    // torque command is undefined behaviour, so never publish one - drop the
    // solution and keep the previous (or the bootstrap) instead.
    bool finite = true;
    for (int leg = 0; leg < 4 && finite; ++leg)
      for (int a = 0; a < 3; ++a)
        if (!std::isfinite(f_local[leg][a]) || !std::isfinite(fr_local[leg][a])) finite = false;
    if (!finite) {
      static int _nanc = 0;
      if ((++_nanc % 20) == 1)
        printf("[MPCW] REJECTED non-finite solution (%d so far)\n", _nanc), fflush(stdout);
      continue;
    }
    if (getenv("STM32MP1_EST_DBG")) {
      static int _sc = 0;
      if ((++_sc % 10) == 1)
        printf("[MPCW] solve #%d took %.1f ms, fz=[%.0f %.0f %.0f %.0f]\n", _sc, _tsolve.getMs(),
               f_local[0][2], f_local[1][2], f_local[2][2], f_local[3][2]), fflush(stdout);
    }
    {
      std::lock_guard<std::mutex> lk(_mpcMtx);
      for (int leg = 0; leg < 4; ++leg) {
        _f_ff_async[leg]   = f_local[leg];
        _Fr_des_async[leg] = fr_local[leg];
      }
      _mpcBusy = false;
    }
    _mpcHaveSolution.store(true);
  }
}

void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data) {
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for(int i = 0; i < horizonLength; i++) {
    contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
  }

  for(int i = 0; i < horizonLength; i++) {
    for(u32 j = 0; j < 12; j++) {
      _sparseTrajectory[i][j] = trajAll[i*12 + j];
    }
  }

  Vec12<float> feet;
  for(u32 foot = 0; foot < 4; foot++) {
    for(u32 axis = 0; axis < 3; axis++) {
      feet[foot*3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for(u32 foot = 0; foot < 4; foot++) {
    Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
    //printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC() {
  Mat3<double> baseInertia;
#ifdef USE_GO1_MODEL
  // Match RobotState (the dense-MPC path): Go1 is 13.1 kg, not the 9 kg
  // mini-cheetah. Unused while cmpc_use_sparse is 0, but a 30% mass error
  // sitting in the alternate solver is a trap for whoever enables it.
  baseInertia << 0.102, 0, 0,
              0, 0.379, 0,
              0, 0, 0.352;
  double mass = 13.1;
#else
  baseInertia << 0.07, 0, 0,
              0, 0.26, 0,
              0, 0, 0.242;
  double mass = 9;
#endif
  double maxForce = 120;

  std::vector<double> dtTraj;
  for(int i = 0; i < horizonLength; i++) {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(0.4);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}

