//
// Created by shivesh on 7/12/18.
//

#include "xdwa_local_planner/trajectory.h"

namespace xdwa_local_planner{
    Trajectory::Trajectory() {

    }

    Trajectory::~Trajectory() {

    }

    std::ostream & operator << (std::ostream& out, Trajectory traj){
        for(int i = 0; i < traj.num_points_; ++i){
            out<<"Step: "<<i<<std::endl;
            out<<"Pose X: "<<traj.x_[i]<<" Pose Y: "<<traj.y_[i]<<" Pose Theta: "<<traj.theta_[i]<<std::endl;
            out<<"Vel X: "<<traj.vel_x_[i]<<" Vel Y: "<<traj.vel_y_[i]<<" Vel Theta: "<<traj.vel_theta_[i]<<std::endl;
        }
        out<<std::endl;
        return out;
    }
}