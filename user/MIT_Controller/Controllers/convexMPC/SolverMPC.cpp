#include "SolverMPC.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
//#include <unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>
#include <stdio.h>
#include <sys/time.h>
#include <Utilities/Timer.h>
#include <JCQP/QpProblem.h>
#include "../../../../stm32mp1/gazebo/ShmTrace.h"   // per-tick/text SHM tracing - see that file's own header

// Precision for the JCQP solve. Float on the A7 (no double-precision SIMD);
// SIM_MPC_DOUBLE restores MIT's double at build time if ever needed.
#ifdef STM32MP1_MPC_DOUBLE
typedef double JCQP_T;
#else
typedef float JCQP_T;
#endif

//#define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10
//big enough to act like infinity, small enough to avoid numerical weirdness.

RobotState rs;
using std::cout;
using std::endl;
using Eigen::Dynamic;

//qpOASES::real_t a;

Matrix<fpt,Dynamic,13> A_qp;
Matrix<fpt,Dynamic,Dynamic> B_qp;
Matrix<fpt,13,12> Bdt;
Matrix<fpt,13,13> Adt;
Matrix<fpt,25,25> ABc,expmm;
Matrix<fpt,Dynamic,Dynamic> S;
Matrix<fpt,Dynamic,1> X_d;
Matrix<fpt,Dynamic,1> U_b;
Matrix<fpt,Dynamic,Dynamic> fmat;

Matrix<fpt,Dynamic,Dynamic> qH;
Matrix<fpt,Dynamic,1> qg;

Matrix<fpt,Dynamic,Dynamic> eye_12h;

qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* ub_qpoases;
qpOASES::real_t* q_soln;

qpOASES::real_t* H_red;
qpOASES::real_t* g_red;
qpOASES::real_t* A_red;
qpOASES::real_t* lb_red;
qpOASES::real_t* ub_red;
qpOASES::real_t* q_red;
u8 real_allocated = 0;


char var_elim[2000];
char con_elim[2000];

mfp* get_q_soln()
{
  return q_soln;
}

s8 near_zero(fpt a)
{
  return (a < 0.01 && a > -.01) ;
}

s8 near_one(fpt a)
{
  return near_zero(a-1);
}
void matrix_to_real(qpOASES::real_t* dst, Matrix<fpt,Dynamic,Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for(s16 r = 0; r < rows; r++)
  {
    for(s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r,c);
      a++;
    }
  }
}


void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt dt,s16 horizon)
{
  ABc.setZero();
  ABc.block(0,0,13,13) = Ac;
  ABc.block(0,13,13,12) = Bc;
  ABc = dt*ABc;
  expmm = ABc.exp();
  Adt = expmm.block(0,0,13,13);
  Bdt = expmm.block(0,13,13,12);
#ifdef K_PRINT_EVERYTHING
  cout<<"Adt: \n"<<Adt<<"\nBdt:\n"<<Bdt<<endl;
#endif
  if(horizon > 19) {
    throw std::runtime_error("horizon is too long!");
  }

  Matrix<fpt,13,13> powerMats[20];
  powerMats[0].setIdentity();
  for(int i = 1; i < horizon+1; i++) {
    powerMats[i] = Adt * powerMats[i-1];
  }

  for(s16 r = 0; r < horizon; r++)
  {
    A_qp.block(13*r,0,13,13) = powerMats[r+1];//Adt.pow(r+1);
    for(s16 c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        s16 a_num = r-c;
        B_qp.block(13*r,12*c,13,12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
      }
    }
  }

#ifdef K_PRINT_EVERYTHING
  cout<<"AQP:\n"<<A_qp<<"\nBQP:\n"<<B_qp<<endl;
#endif
}

void resize_qp_mats(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon*horizon;

  A_qp.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon*1;

  B_qp.resize(13*horizon, 12*horizon);
  mcount += 13*h2*12;

  S.resize(13*horizon, 13*horizon);
  mcount += 13*13*h2;

  X_d.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  U_b.resize(20*horizon, Eigen::NoChange);
  mcount += 20*horizon;

  fmat.resize(20*horizon, 12*horizon);
  mcount += 20*12*h2;

  qH.resize(12*horizon, 12*horizon);
  mcount += 12*12*h2;

  qg.resize(12*horizon, Eigen::NoChange);
  mcount += 12*horizon;

  eye_12h.resize(12*horizon, 12*horizon);
  mcount += 12*12*horizon;

  //printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  fmat.setZero();
  qH.setZero();
  eye_12h.setIdentity();

  //TODO: use realloc instead of free/malloc on size changes

  if(real_allocated)
  {

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_qpoases = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  lb_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  ub_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  q_soln = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;

  H_red = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_red = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_red = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  lb_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  ub_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  q_red = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  real_allocated = 1;

  //printf("malloc'd %d floating point numbers.\n",mcount);



#ifdef K_DEBUG
  shmtrace::logf(0.0, "RESIZED MATRICES FOR HORIZON: %d",horizon);
#endif
}

inline Matrix<fpt,3,3> cross_mat(Matrix<fpt,3,3> I_inv, Matrix<fpt,3,1> r)
{
  Matrix<fpt,3,3> cm;
  cm << 0.f, -r(2), r(1),
    r(2), 0.f, -r(0),
    -r(1), r(0), 0.f;
  return I_inv * cm;
}
//continuous time state space matrices.
void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,4> r_feet, Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B, float x_drag)
{
  A.setZero();
  A(3,9) = 1.f;
  A(11,9) = x_drag;
  A(4,10) = 1.f;
  A(5,11) = 1.f;

  A(11,12) = 1.f;
  A.block(0,6,3,3) = R_yaw.transpose();

  B.setZero();
  Matrix<fpt,3,3> I_inv = I_world.inverse();

  for(s16 b = 0; b < 4; b++)
  {
    B.block(6,b*3,3,3) = cross_mat(I_inv,r_feet.col(b));
    B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
  }
}


void quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy)
{
  //from my MATLAB implementation

  //edge case!
  fpt as = t_min(-2.*(q.x()*q.z()-q.w()*q.y()),.99999);
  rpy(0) = atan2(2.f*(q.x()*q.y()+q.w()*q.z()),sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f*(q.y()*q.z()+q.w()*q.x()),sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));

}
void print_problem_setup(problem_setup* setup)
{
  shmtrace::logf(0.0, "DT: %.3f Mu: %.3f F_Max: %.3f Horizon: %d",
                 setup->dt, setup->mu, setup->f_max, setup->horizon);
}

void print_update_data(update_data_t* update, s16 horizon)
{
  print_named_array("p",update->p,1,3);
  print_named_array("v",update->v,1,3);
  print_named_array("q",update->q,1,4);
  print_named_array("w",update->r,3,4);
  pnv("Yaw",update->yaw);
  print_named_array("weights",update->weights,1,12);
  print_named_array("trajectory",update->traj,horizon,12);
  pnv("Alpha",update->alpha);
  print_named_array("gait",update->gait,horizon,4);
}


Matrix<fpt,13,1> x_0;
Matrix<fpt,3,3> I_world;
Matrix<fpt,13,13> A_ct;
Matrix<fpt,13,12> B_ct_r;


void solve_mpc(update_data_t* update, problem_setup* setup)
{
  rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);
#ifdef K_PRINT_EVERYTHING

  shmtrace::logf(0.0, "----------------- PROBLEM DATA  -----------------");
    print_problem_setup(setup);

    shmtrace::logf(0.0, "-----------------    ROBOT DATA   -----------------");
    rs.print();
    print_update_data(update,setup->horizon);
#endif

  //roll pitch yaw
  Matrix<fpt,3,1> rpy;
  quat_to_rpy(rs.q,rpy);

  //initial state (13 state representation)
  x_0 << rpy(2), rpy(1), rpy(0), rs.p , rs.w, rs.v, -9.8f;
  I_world = rs.R_yaw * rs.I_body * rs.R_yaw.transpose(); //original
  //I_world = rs.R_yaw.transpose() * rs.I_body * rs.R_yaw;
  //cout<<rs.R_yaw<<endl;
  ct_ss_mats(I_world,rs.m,rs.r_feet,rs.R_yaw,A_ct,B_ct_r, update->x_drag);


#ifdef K_PRINT_EVERYTHING
  cout<<"Initial state: \n"<<x_0<<endl;
    cout<<"World Inertia: \n"<<I_world<<endl;
    cout<<"A CT: \n"<<A_ct<<endl;
    cout<<"B CT (simplified): \n"<<B_ct_r<<endl;
#endif
  //QP matrices
  c2qp(A_ct,B_ct_r,setup->dt,setup->horizon);

  //weights
  Matrix<fpt,13,1> full_weight;
  for(u8 i = 0; i < 12; i++)
    full_weight(i) = update->weights[i];
  full_weight(12) = 0.f;
  S.diagonal() = full_weight.replicate(setup->horizon,1);

  // FLIGHT-PHASE COST GATING.
  // MIT replicates one weight vector across every horizon step, and the
  // trajectory hands every step the SAME z reference (_body_height) with vz = 0.
  // For a gait with a real flight phase that is a contradiction with the gait's
  // own contact schedule: on an airborne step the robot has NO contacts, so the
  // friction cone forces every foot force to zero and the height error is
  // physically uncontrollable - yet z carries Q[5] = 50, the LARGEST weight in
  // MIT's vector. The optimiser cannot reduce that cost during flight, so it
  // distorts the forces on the surrounding CONTACT steps trying to, which is a
  // good description of what pronking and galloping do here (they fail the
  // FLIGHT-COST GATE: REMOVED 2026-08-29 (OPEN-13), measured HARMFUL.
  // The idea was not to put cost on a state you have no authority over -
  // zero the z/vz weights on all-swing horizon steps. Sound reasoning,
  // wrong conclusion: the cost is what makes the optimiser command force
  // at the CONTACT steps, so dropping it on the majority of the horizon
  // (6 of 10 for pronking) removed most of the objective and solved force
  // collapsed from 39-42 N per foot to 6.1. It shipped default-ON once by
  // mistake and was reverted; it then sat as a dead opt-in flag for
  // months. Deleted rather than left as a trap for the next reader.

  //trajectory
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 12; j++)
      X_d(13*i+j,0) = update->traj[12*i+j];
  }
  //cout<<"XD:\n"<<X_d<<endl;



  //note - I'm not doing the shifting here.
  s16 k = 0;
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 4; j++)
    {
      U_b(5*k + 0) = BIG_NUMBER;
      U_b(5*k + 1) = BIG_NUMBER;
      U_b(5*k + 2) = BIG_NUMBER;
      U_b(5*k + 3) = BIG_NUMBER;
      U_b(5*k + 4) = update->gait[i*4 + j] * setup->f_max;
      k++;
    }
  }



  fpt mu = 1.f/setup->mu;
  Matrix<fpt,5,3> f_block;

  f_block <<  mu, 0,  1.f,
    -mu, 0,  1.f,
    0,  mu, 1.f,
    0, -mu, 1.f,
    0,   0, 1.f;

  for(s16 i = 0; i < setup->horizon*4; i++)
  {
    fmat.block(i*5,i*3,5,3) = f_block;
  }




  if(getenv("STM32MP1_MPC_MAT")) {
    static int _mc = 0;
    if((++_mc % 10) == 1) {
      shmtrace::logf(0.0, "[MPCMAT] m=%.2f I=(%.4f %.4f %.4f) |A_qp|=%.3g |B_qp|=%.3g |X_d|=%.3g |U_b|=%.3g",
             rs.m, rs.I_body(0,0), rs.I_body(1,1), rs.I_body(2,2),
             A_qp.norm(), B_qp.norm(), X_d.norm(), U_b.norm());
    }
  }
  // S is DIAGONAL - it is built as `S.diagonal() = full_weight.replicate(...)`
  // and nothing else ever writes to it - but it is declared as a dense
  // 13h x 13h matrix, so `B_qp.transpose()*S*B_qp` was a full dense triple
  // product: at horizon 10 that is (120x130)(130x130)(130x120), about 3.9M
  // multiply-adds, and it is the dominant cost of the whole solve on this
  // board. Telling Eigen it is diagonal turns the first product into a row
  // scaling (15.6k ops), roughly halving the work, and is numerically
  // identical.
  const auto Sd = S.diagonal().asDiagonal();
  qH = 2*(B_qp.transpose()*Sd*B_qp + update->alpha*eye_12h);
  qg = 2*B_qp.transpose()*Sd*(A_qp*x_0 - X_d);
  if(getenv("STM32MP1_MPC_MAT")) {
    static int _mc2 = 0;
    if((++_mc2 % 10) == 1) {
      shmtrace::logf(0.0, "[MPCCOST] |S|=%.3g |qH|=%.3g |qg|=%.3g |fmat|=%.3g alpha=%.2g",
             S.norm(), qH.norm(), qg.norm(), fmat.norm(), (double)update->alpha);
    }
  }

  // SINGLE PRECISION. Everything MIT builds here is already `fpt` = float, and
  // this was the one place it got widened: QpProblem<double> plus .cast<double>()
  // on every matrix. A Cortex-A7's NEON is 4-wide FLOAT and has no
  // double-precision SIMD at all, so the solve was running scalar VFP on data
  // that started out single precision anyway. jcqp_f keeps it float end to end.
  if(update->use_jcqp == 1) {
    // ---- CONTACT-ONLY REDUCTION ----
    // This ports MIT's own qpOASES-path variable elimination to the JCQP path,
    // which never had it: swing-foot forces are pinned to exactly zero by
    // their friction-cone rows (U_b = gait*f_max = 0 forces fz = 0, and the
    // four mu-rows then pin fx = fy = 0), yet the stock JCQP path still
    // carried all 12h of them as decision variables - half of them known-zero
    // in a trot. Setting x_elim = 0 and dropping those columns/rows is EXACT
    // (P_red = P(keep,keep), q_red = q(keep): the eliminated block contributes
    // nothing at x_elim = 0), and the KKT system the solver factorises shrinks
    // from (12h + 20h) to about half that in a trot - factorisation cost is
    // cubic in that size.
    static s16 vi[12*36];                       // kept variable indices
    static s16 ci[20*36];                       // kept constraint rows
    s32 nv = 0, nc = 0, nA = 0;
    for(s16 i = 0; i < setup->horizon; i++) {
      for(s16 j = 0; j < 4; j++) {
        s32 fs = i*4 + j;
        if(update->gait[fs]) {
          for(s32 a = 0; a < 3; a++) vi[nv++] = 3*fs + a;
          for(s32 a = 0; a < 5; a++) ci[nc++] = 5*fs + a;
          nA++;
        }
      }
    }
    for(s16 i = 0; i < 12*setup->horizon; i++) q_soln[i] = 0.0;

    if(nA > 0) {
      // ---- PERSISTENT SOLVER + WARM START ----
      // The stock path constructed a fresh QpProblem every solve and cold
      // started ADMM from zero. Consecutive MPC problems differ only by one
      // gait segment and a slightly moved state, so the previous solution is
      // an excellent initial iterate. Only the worker thread ever runs this,
      // so a static is safe; on a contact-pattern change the dimensions
      // differ and we rebuild + cold start.
      static QpProblem<JCQP_T>* jc = nullptr;
      static s32 jc_nv = -1, jc_nc = -1;
      bool fresh = (!jc || jc_nv != nv || jc_nc != nc);
      if(fresh) {
        delete jc;
        jc = new QpProblem<JCQP_T>(nv, nc);
        jc_nv = nv; jc_nc = nc;
      }
      for(s32 rI = 0; rI < nv; rI++) {
        jc->q[rI] = qg(vi[rI]);
        for(s32 cI = 0; cI < nv; cI++) jc->P(rI,cI) = qH(vi[rI], vi[cI]);
      }
      for(s32 rI = 0; rI < nc; rI++) {
        jc->u[rI] = U_b(ci[rI]);
        jc->l[rI] = 0.;
        for(s32 cI = 0; cI < nv; cI++) jc->A(rI,cI) = fmat(ci[rI], vi[cI]);
      }
      jc->settings.sigma = update->sigma;
      jc->settings.alpha = update->solver_alpha;
      jc->settings.terminate = update->terminate;
      jc->settings.rho = update->rho;
      jc->settings.maxIterations = update->max_iterations;
      // Warm starting is only valid when the previous solution means the same
      // thing: the gait table SHIFTS one segment per solve, so slot k of the
      // reduced vector changes identity (a different foot-step) between
      // consecutive problems. ADMM started from an identity-mismatched iterate
      // and truncated at 60 iterations returns a half-converged wrong answer -
      // measured as weak, lopsided forces and a launch. So warm start ONLY
      // when the contact table is bit-identical to the previous solve
      // ($CTRL_MPC_WARM=0 disables even that).
      static u8 prev_gait[4*36];
      static s32 prev_ng = -1;
      s32 ng = 4*setup->horizon;
      bool same_table = (prev_ng == ng);
      if(same_table)
        for(s32 g = 0; g < ng; g++) if(prev_gait[g] != update->gait[g]) { same_table = false; break; }
      for(s32 g = 0; g < ng; g++) prev_gait[g] = update->gait[g];
      prev_ng = ng;
      static const bool warm_ok = !(getenv("CTRL_MPC_WARM") && atoi(getenv("CTRL_MPC_WARM")) == 0);
      if(!fresh && same_table && warm_ok) jc->hotStart();
      jc->runFromDense(update->max_iterations, true, false);
      for(s32 rI = 0; rI < nv; rI++) q_soln[vi[rI]] = jc->getSolution()[rI];
    }
  } else {



    matrix_to_real(H_qpoases,qH,setup->horizon*12, setup->horizon*12);
    matrix_to_real(g_qpoases,qg,setup->horizon*12, 1);
    matrix_to_real(A_qpoases,fmat,setup->horizon*20, setup->horizon*12);
    matrix_to_real(ub_qpoases,U_b,setup->horizon*20, 1);

    for(s16 i = 0; i < 20*setup->horizon; i++)
      lb_qpoases[i] = 0.0f;

    s16 num_constraints = 20*setup->horizon;
    s16 num_variables = 12*setup->horizon;


    // Working-set recalculation budget. qpOASES is explicitly designed to be
    // truncated - it returns its best iterate - and this solve is COLD STARTED
    // every MPC update (a fresh QProblem, no hotstart), which on this Cortex-A7
    // costs 56-85 ms for the 12*horizon variable problem against a 2 ms control
    // period. MIT's x86 UP board did the same solve in 1-2 ms and never noticed.
    // $CTRL_MPC_NWSR trades optimality for latency.
    static const int _nwsr_env = getenv("CTRL_MPC_NWSR") ? atoi(getenv("CTRL_MPC_NWSR")) : 100;
    qpOASES::int_t nWSR = _nwsr_env;


    int new_vars = num_variables;
    int new_cons = num_constraints;

    for(int i =0; i < num_constraints; i++)
      con_elim[i] = 0;

    for(int i = 0; i < num_variables; i++)
      var_elim[i] = 0;


    for(int i = 0; i < num_constraints; i++)
    {
      if(! (near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue;
      double* c_row = &A_qpoases[i*num_variables];
      for(int j = 0; j < num_variables; j++)
      {
        if(near_one(c_row[j]))
        {
          new_vars -= 3;
          new_cons -= 5;
          int cs = (j*5)/3 -3;
          var_elim[j-2] = 1;
          var_elim[j-1] = 1;
          var_elim[j  ] = 1;
          con_elim[cs] = 1;
          con_elim[cs+1] = 1;
          con_elim[cs+2] = 1;
          con_elim[cs+3] = 1;
          con_elim[cs+4] = 1;
        }
      }
    }
    //if(new_vars != num_variables)
    if(1==1)
    {
      int var_ind[new_vars];
      int con_ind[new_cons];
      int vc = 0;
      for(int i = 0; i < num_variables; i++)
      {
        if(!var_elim[i])
        {
          if(!(vc<new_vars))
          {
            shmtrace::logf(0.0, "BAD ERROR 1");
          }
          var_ind[vc] = i;
          vc++;
        }
      }
      vc = 0;
      for(int i = 0; i < num_constraints; i++)
      {
        if(!con_elim[i])
        {
          if(!(vc<new_cons))
          {
            shmtrace::logf(0.0, "BAD ERROR 1");
          }
          con_ind[vc] = i;
          vc++;
        }
      }
      for(int i = 0; i < new_vars; i++)
      {
        int olda = var_ind[i];
        g_red[i] = g_qpoases[olda];
        for(int j = 0; j < new_vars; j++)
        {
          int oldb = var_ind[j];
          H_red[i*new_vars + j] = H_qpoases[olda*num_variables + oldb];
        }
      }

      for (int con = 0; con < new_cons; con++)
      {
        for(int st = 0; st < new_vars; st++)
        {
          float cval = A_qpoases[(num_variables*con_ind[con]) + var_ind[st] ];
          A_red[con*new_vars + st] = cval;
        }
      }
      for(int i = 0; i < new_cons; i++)
      {
        int old = con_ind[i];
        ub_red[i] = ub_qpoases[old];
        lb_red[i] = lb_qpoases[old];
      }

      if(update->use_jcqp == 0) {
        Timer solve_timer;
        qpOASES::QProblem problem_red (new_vars, new_cons);
        qpOASES::Options op;
        op.setToMPC();
        op.printLevel = qpOASES::PL_NONE;
        problem_red.setOptions(op);
        //int_t nWSR = 50000;


        int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
        (void)rval;
        int rval2 = problem_red.getPrimalSolution(q_red);
        if(rval2 != qpOASES::SUCCESSFUL_RETURN)
          shmtrace::logf(0.0, "failed to solve!");

        // printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);


        vc = 0;
        for(int i = 0; i < num_variables; i++)
        {
          if(var_elim[i])
          {
            q_soln[i] = 0.0f;
          }
          else
          {
            q_soln[i] = q_red[vc];
            vc++;
          }
        }
      } else { // use jcqp == 2
        QpProblem<double> reducedProblem(new_vars, new_cons);

        reducedProblem.A = DenseMatrix<double>(new_cons, new_vars);
        int i = 0;
        for(int r = 0; r < new_cons; r++) {
          for(int c = 0; c < new_vars; c++) {
            reducedProblem.A(r,c) = A_red[i++];
          }
        }

        reducedProblem.P = DenseMatrix<double>(new_vars, new_vars);
        i = 0;
        for(int r = 0; r < new_vars; r++) {
          for(int c = 0; c < new_vars; c++) {
            reducedProblem.P(r,c) = H_red[i++];
          }
        }

        reducedProblem.q = Vector<double>(new_vars);
        for(int r = 0; r < new_vars; r++) {
          reducedProblem.q[r] = g_red[r];
        }

        reducedProblem.u = Vector<double>(new_cons);
        for(int r = 0; r < new_cons; r++) {
          reducedProblem.u[r] = ub_red[r];
        }

        reducedProblem.l = Vector<double>(new_cons);
        for(int r = 0; r < new_cons; r++) {
          reducedProblem.l[r] = lb_red[r];
        }

//        jcqp.A = fmat.cast<double>();
//        jcqp.P = qH.cast<double>();
//        jcqp.q = qg.cast<double>();
//        jcqp.u = U_b.cast<double>();
//        for(s16 i = 0; i < 20*setup->horizon; i++)
//          jcqp.l[i] = 0.;

        reducedProblem.settings.sigma = update->sigma;
        reducedProblem.settings.alpha = update->solver_alpha;
        reducedProblem.settings.terminate = update->terminate;
        reducedProblem.settings.rho = update->rho;
        reducedProblem.settings.maxIterations = update->max_iterations;
        reducedProblem.runFromDense(update->max_iterations, true, false);

        vc = 0;
        for(int kk = 0; kk < num_variables; kk++)
        {
          if(var_elim[kk])
          {
            q_soln[kk] = 0.0f;
          }
          else
          {
            q_soln[kk] = reducedProblem.getSolution()[vc];
            vc++;
          }
        }
      }

    }
  }




  // (JCQP solution is scattered into q_soln inside its branch above;
  // eliminated swing-foot entries stay exactly zero.)

  if(getenv("STM32MP1_MPC_MAT")) {
    static int _sc2 = 0;
    if((++_sc2 % 10) == 1) {
      double nrm = 0; for(int i=0;i<12*setup->horizon;i++) nrm += q_soln[i]*q_soln[i];
      shmtrace::logf(0.0, "[MPCSOL] |q_soln|=%.4g  first12=%.1f %.1f %.1f  %.1f %.1f %.1f  %.1f %.1f %.1f  %.1f %.1f %.1f",
             sqrt(nrm), q_soln[0],q_soln[1],q_soln[2], q_soln[3],q_soln[4],q_soln[5],
             q_soln[6],q_soln[7],q_soln[8], q_soln[9],q_soln[10],q_soln[11]);
    }
  }



#ifdef K_PRINT_EVERYTHING
  //cout<<"fmat:\n"<<fmat<<endl;
#endif



}
