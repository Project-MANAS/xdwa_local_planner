//
// Created by shivesh on 2/7/19.
//

#include "xdwa_local_planner/goal_dist_score_function.h"

namespace xdwa_local_planner {
GoalDistScoreFunction::GoalDistScoreFunction() :
    transform_tolerance_(1.0) {}

GoalDistScoreFunction::~GoalDistScoreFunction() {}

void GoalDistScoreFunction::initialize(rclcpp::Node::SharedPtr node,
                                       std::shared_ptr<tf2_ros::Buffer> buffer,
                                       geometry_msgs::msg::PoseStamped::SharedPtr goal,
                                       geometry_msgs::msg::PoseStamped::SharedPtr pose,
                                       std::string costmap_topic,
                                       std::vector<std::vector<double>> footprint) {
  scale_ = 1;
  node_ = node;
  buffer_ = buffer;
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  goal_ = goal;
  pose_ = pose;
}

double GoalDistScoreFunction::scoreTrajectory(std::shared_ptr<xdwa_local_planner::Trajectory> tj) {
  double cost = 0;
  for (int i = tj->num_points_scored_ + 1; i <= tj->num_points_; ++i) {
    cost += getDist(tj->x_[i], tj->y_[i]);
  }
  return cost;
}

double GoalDistScoreFunction::getDist(double x, double y) {
  return hypot(goal_->pose.position.x - x, goal_->pose.position.y - y);
}
}

PLUGINLIB_EXPORT_CLASS(xdwa_local_planner::GoalDistScoreFunction, xdwa_local_planner::TrajectoryScoreFunction)
