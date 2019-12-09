//
// Created by shivesh on 7/12/18.
//

#include "xdwa_local_planner/trajectory_generator.h"

namespace xdwa_local_planner {
TrajectoryGenerator::TrajectoryGenerator() {
  min_vel_x_ = -1;
  max_vel_x_ = 1;
  min_vel_y_ = 0;
  max_vel_y_ = 0;
  min_vel_theta_ = -0.57;
  max_vel_theta_ = 0.57;
  num_samples_x_ = 25;
  num_samples_y_ = 0;
  num_samples_theta_ = 25;
  acc_lim_x_ = 1;
  acc_lim_y_ = 0;
  acc_lim_theta_ = 0.1;
  deacc_lim_x_ = -0.5;
  deacc_lim_y_ = 0;
  deacc_lim_theta_ = -1;
  min_trans_vel_ = 0;
  max_trans_vel_ = 1;
  min_rot_vel_ = 0;
  max_rot_vel_ = 1.57;
  sim_period_ = 1;
}

TrajectoryGenerator::~TrajectoryGenerator() {}

void TrajectoryGenerator::generateSamples(double vel_x, double vel_y, double vel_theta) {
  vsamples_.clear();
  double max_vel_x = std::min(max_vel_x_, vel_x + acc_lim_x_ * sim_period_);
  double max_vel_y = std::min(max_vel_y_, vel_y + acc_lim_y_ * sim_period_);
  double max_vel_theta = std::min(max_vel_theta_, vel_theta + acc_lim_theta_ * sim_period_);

  double min_vel_x = std::min(std::max(min_vel_x_, vel_x + deacc_lim_x_ * sim_period_), max_vel_x);
  double min_vel_y = std::min(std::max(min_vel_y_, vel_y + deacc_lim_y_ * sim_period_), max_vel_y);
  double min_vel_theta = std::min(std::max(min_vel_theta_, vel_theta + deacc_lim_theta_ * sim_period_), max_vel_theta);

  double step_size_x = (max_vel_x - min_vel_x) / (std::max(1, (num_samples_x_ - 1)));
  double step_size_y = (max_vel_y - min_vel_y) / (std::max(1, (num_samples_y_ - 1)));
  double step_size_theta = (max_vel_theta - min_vel_theta) / (std::max(1, (num_samples_theta_ - 1)));

  step_size_x = step_size_x == 0 ? 1 : step_size_x;
  step_size_y = step_size_y == 0 ? 1 : step_size_y;
  step_size_theta = step_size_theta == 0 ? 1 : step_size_theta;

  for (double i = min_vel_x; i <= max_vel_x; i += step_size_x) {
    for (double j = min_vel_y; j <= max_vel_y; j += step_size_y) {
      for (double k = min_vel_theta; k <= max_vel_theta; k += step_size_theta) {
        double vmag = hypot(i, j);
//        if (vmag < min_trans_vel_ || vmag > max_trans_vel_ || fabs(k) < min_rot_vel_ || fabs(k) > max_rot_vel_)
//          continue;
        std::shared_ptr<VelocitySample> vs(std::make_shared<VelocitySample>(i, j, k));
        vsamples_.push_back(vs);
      }
    }
  }
}

bool TrajectoryGenerator::generateTrajectory(std::shared_ptr<VelocitySample> vs,
                                             double pose_x,
                                             double pose_y,
                                             double pose_theta,
                                             double sim_time,
                                             int num_steps,
                                             std::shared_ptr<Trajectory> traj) {

  double dt = sim_time / num_steps;
  traj->vel_x_.push_back(vs->vsample_x_);
  traj->vel_y_.push_back(vs->vsample_y_);
  traj->vel_theta_.push_back(vs->vsample_theta_);
  for (int i = 0; i < num_steps; ++i) {
    computeNewPose(pose_x, pose_y, pose_theta, vs->vsample_x_, vs->vsample_y_, vs->vsample_theta_, dt);
    traj->x_.push_back(pose_x);
    traj->y_.push_back(pose_y);
    traj->theta_.push_back(pose_theta);
    ++traj->num_points_;
  }
  return true;
}

void TrajectoryGenerator::computeNewPose(double &x, double &y, double &theta, double vel_x, double vel_y,
                                         double vel_theta, double dt) {
  x = x + ((vel_x * cos(theta)) - (vel_y * sin(theta))) * dt;
  y = y + ((vel_x * sin(theta)) + (vel_y * cos(theta))) * dt;
  theta = theta + vel_theta * dt;
}
}