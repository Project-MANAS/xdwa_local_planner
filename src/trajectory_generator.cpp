//
// Created by shivesh on 7/12/18.
//

#include "xdwa_local_planner/trajectory_generator.h"

namespace xdwa_local_planner{
    TrajectoryGenerator::TrajectoryGenerator() {
        min_vel_x_ = 0;
        max_vel_x_ = 5;
        min_vel_y_ = 0;
        max_vel_y_ = 0;
        min_vel_theta_ = 0;
        max_vel_theta_ = 0;
        num_samples_x_ = 100;
        num_samples_y_ = 100;
        num_samples_theta_ = 100;
        acc_lim_x_ = 1;
        acc_lim_y_ = 0;
        acc_lim_theta_ = 1;
        deacc_lim_x_ = -1;
        deacc_lim_y_ = 0;
        deacc_lim_theta_ = -1;
        min_trans_vel_ = 0;
        max_trans_vel_ = 10;
        min_rot_vel_ = 0;
        max_rot_vel_ = 5;
        sim_period_ = 0.05;
        samples_generated_ = false;
    }

    TrajectoryGenerator::~TrajectoryGenerator() {

    }

    void TrajectoryGenerator::generateSamples() {
        double step_size_x = std::max(1e-30, (max_vel_x_ - min_vel_x_) / (std::max(1, (num_samples_x_ - 1))));
        double step_size_y = std::max(1e-30, (max_vel_y_ - min_vel_y_) / (std::max(1, (num_samples_y_ - 1))));
        double step_size_theta_ = std::max(1e-30, (max_vel_theta_ - min_vel_theta_) / (std::max(1, (num_samples_theta_ - 1))));

        for(double i = min_vel_x_; i <= max_vel_x_; i += step_size_x){
            for(double j = min_vel_y_; j <= max_vel_y_; j += step_size_y){
                for(double k = min_vel_theta_; k <= max_vel_theta_; k += step_size_theta_){
                    double vmag = hypot(i, j);
                    if(vmag < min_trans_vel_|| vmag > max_trans_vel_ || fabs(k) < min_rot_vel_ || fabs(k) > max_rot_vel_)
                        continue;
                    double minx = i - acc_lim_x_ * sim_period_;
                    double maxx = i - deacc_lim_x_ * sim_period_;
                    double miny = j - acc_lim_y_ * sim_period_;
                    double maxy = j - deacc_lim_y_ * sim_period_;
                    double mintheta = k - acc_lim_theta_ * sim_period_;
                    double maxtheta = k - deacc_lim_theta_ * sim_period_;
                    std::shared_ptr<VelocitySample> vs(std::make_shared<VelocitySample>(i, j, k, minx, miny, mintheta, maxx, maxy, maxtheta));
                    vsamples_.push_back(vs);
                }
            }
        }
//        Trajectory tj;
//        tj.num_points_ = 0;
//        std::cout<<**(--vsamples_.end());
//        generateTrajectory(*--vsamples_.end(), 20, 1, 1, 4.95, 0, 0, 5, 5, &tj);
//        std::cout<<tj;

        samples_generated_ = true;
    }

    bool TrajectoryGenerator::generateTrajectory(std::shared_ptr<VelocitySample> vs, double pose_x, double pose_y,
            double pose_theta, double vel_x, double vel_y, double vel_theta, double sim_time, int num_steps, std::shared_ptr<Trajectory> traj) {
        if(vs->vminsample_x_ > vel_x || vs->vmaxsample_x_ < vel_x)
            return false;
        if(vs->vminsample_y_ > vel_y || vs->vmaxsample_y_ < vel_y)
            return false;
        if(vs->vminsample_theta_ > vel_theta || vs->vmaxsample_theta_ < vel_theta)
            return false;
        double dt = sim_time / num_steps;

        for(int i = 0; i < num_steps; ++i){
            computeNewPose(pose_x, pose_y, pose_theta, vs->vsample_x_, vs->vsample_y_, vs->vsample_theta_, dt);
            traj->x_.push_back(pose_x);
            traj->y_.push_back(pose_y);
            traj->theta_.push_back(pose_theta);
            traj->vel_x_.push_back(vs->vsample_x_);
            traj->vel_y_.push_back(vs->vsample_y_);
            traj->vel_theta_.push_back(vs->vsample_theta_);
            ++traj->num_points_;
        }
        return true;
    }

    void TrajectoryGenerator::computeNewPose(double &x, double &y, double &theta, double vel_x, double vel_y,
                                             double vel_theta, double dt) {
        x = x + ((vel_x * cos(theta)) - (vel_y * sin(theta)) * dt);
        y = y + ((vel_x * sin(theta)) + (vel_y * cos(theta)) * dt);
        theta = theta + vel_theta * dt;
    }
}

int main(){
    xdwa_local_planner::TrajectoryGenerator tg;
    tg.generateSamples();
}