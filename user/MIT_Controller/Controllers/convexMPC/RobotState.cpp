#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = yaw_;

    //for(u8 i = 0; i < 12; i++)
    //    this->r_feet(i) = r[i];
    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 4; c++)
            this->r_feet(rs,c) = r_[rs*4 + c];

    R = this->q.toRotationMatrix();
    fpt yc = cos(yaw_);
    fpt ys = sin(yaw_);

    R_yaw <<  yc,  -ys,   0,
             ys,  yc,   0,
               0,   0,   1;

    Matrix<fpt,3,1> Id;
#ifdef USE_GO1_MODEL
    // Go1 whole-body inertia about the CoM, recovered from Unitree's own
    // Legged_sport binary (immediates in RobotState::set, 0xfb9f0). These
    // REPLACE a guess: this port had scaled mini-cheetah's (.07,.26,.242) by the
    // 13.1/9 mass ratio to get (0.102,0.379,0.352), which under-estimates roll
    // by 25% and yaw by 24%. An MPC that thinks the body spins up more easily
    // than it does under-commands corrective moments on precisely the two axes
    // this port's gaits fail on - roll collapse and heading drift.
    Id << 0.13662f, 0.425578f, 0.460359f;
#else
    Id << .07f, 0.26f, 0.242f;
#endif
    //Id << 0.3f, 2.1f, 2.1f; // DH
    I_body.diagonal() = Id;

    //TODO: Consider normalizing quaternion??
}

void RobotState::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_yaw<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl;
}



