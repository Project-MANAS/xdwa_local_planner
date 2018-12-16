//
// Created by shivesh on 7/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H
#define XDWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H

#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

#include "xdwa_local_planner/velocity_sample.h"
#include "xdwa_local_planner/trajectory.h"
#include "xdwa_local_planner/trajectory_scorer.h"

namespace xdwa_local_planner{
    class TrajectoryGenerator{
    public:
        TrajectoryGenerator();
        ~TrajectoryGenerator();
        void generateSamples();
        bool generateTrajectory(std::shared_ptr<VelocitySample> vs, double pose_x, double pose_y, double pose_theta,
                                double vel_x, double vel_y, double vel_theta, double sim_time, int num_steps, std::shared_ptr<Trajectory> traj);


        std::vector<std::shared_ptr<VelocitySample>> vsamples_;
        bool samples_generated_;
    private:
        void computeNewPose(double &x, double &y, double &theta, double vel_x, double vel_y, double vel_theta, double dt);

        int num_samples_x_, num_samples_y_, num_samples_theta_;

        double sim_period_;

        double min_vel_x_, min_vel_y_, min_vel_theta_;
        double max_vel_x_, max_vel_y_, max_vel_theta_;
        double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
        double deacc_lim_x_, deacc_lim_y_, deacc_lim_theta_;
        double min_trans_vel_, min_rot_vel_;
        double max_trans_vel_, max_rot_vel_;
    };
}

#endif //XDWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H
