//
// Created by shivesh on 7/12/18.
//

#include "xdwa_local_planner/trajectory.h"

namespace xdwa_local_planner {
Trajectory::Trajectory() {

}

Trajectory::Trajectory(std::shared_ptr<xdwa_local_planner::Trajectory> trajectory) {
  this->x_ = trajectory->x_;
  this->y_ = trajectory->y_;
  this->theta_ = trajectory->theta_;
  this->vel_x_ = trajectory->vel_x_;
  this->vel_y_ = trajectory->vel_y_;
  this->vel_theta_ = trajectory->vel_theta_;
  this->cost_ = trajectory->cost_;
  this->num_points_ = trajectory->num_points_;
  this->num_points_scored_ = trajectory->num_points_scored_;
}

Trajectory::~Trajectory() {

}

std::ostream &operator<<(std::ostream &out, Trajectory traj) {
  for (int i = 0; i < traj.num_points_; ++i) {
    out << "Step: " << i << std::endl;
    out << "Pose X: " << traj.x_[i] << " Pose Y: " << traj.y_[i] << " Pose Theta: " << traj.theta_[i] << std::endl;
    out << "Vel X: " << traj.vel_x_[i] << " Vel Y: " << traj.vel_y_[i] << " Vel Theta: " << traj.vel_theta_[i]
        << std::endl;
  }
  out << std::endl;
  return out;
}
}