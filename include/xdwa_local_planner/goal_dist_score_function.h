//
// Created by shivesh on 2/7/19.
//

#ifndef XDWA_LOCAL_PLANNER_GOAL_DIST_SCORE_FUNCTION_H
#define XDWA_LOCAL_PLANNER_GOAL_DIST_SCORE_FUNCTION_H

#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "xdwa_local_planner/xdwa_local_planner.h"
#include "xdwa_local_planner/trajectory_score_function.h"

namespace xdwa_local_planner {
class GoalDistScoreFunction : public TrajectoryScoreFunction {
 public:
  GoalDistScoreFunction();
  ~GoalDistScoreFunction();

  void initialize(rclcpp::Node::SharedPtr node,
                  std::shared_ptr<tf2_ros::Buffer> buffer,
                  geometry_msgs::msg::PoseStamped::SharedPtr goal,
                  geometry_msgs::msg::PoseStamped::SharedPtr pose,
                  std::string costmap_topic,
                  std::vector<std::array<double, 2>> footprint) override;

  double scoreTrajectory(std::shared_ptr<Trajectory> tj) override;

 private:
  double getDist(double x, double y);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  double transform_tolerance_;
};
}

#endif //XDWA_LOCAL_PLANNER_GOAL_DIST_SCORE_FUNCTION_H
