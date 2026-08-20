#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

#include "common_types.h"
class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w;
        Matrix<fpt,3,4> r_feet;
        Matrix<fpt,3,3> R;
        Matrix<fpt,3,3> R_yaw;
        Matrix<fpt,3,3> I_body;
        Quaternionf q;
        fpt yaw;
#ifdef USE_GO1_MODEL
        // Unitree Go1: total robot mass (gazebo go1.urdf sums to 13.1 kg).
        // The stock 9 kg (mini cheetah) under-supports the Go1 by ~30%,
        // dropping the body at gait start and tumbling the robot.
        fpt m = 13.1f;
#else
        fpt m = 9;
#endif
        //fpt m = 50.236; //DH
    //private:
};
#endif
