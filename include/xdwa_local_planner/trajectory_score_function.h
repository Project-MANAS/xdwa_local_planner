//
// Created by shivesh on 10/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_TRAJECTORY_SCORE_FUNCTION_H
#define XDWA_LOCAL_PLANNER_TRAJECTORY_SCORE_FUNCTION_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "xdwa_local_planner/trajectory.h"

#include "pluginlib/class_list_macros.hpp"

namespace xdwa_local_planner {
class TrajectoryScoreFunction {
 public:
  virtual void initialize(rclcpp::Node::SharedPtr node,
                          std::shared_ptr<tf2_ros::Buffer> buffer,
                          geometry_msgs::msg::PoseStamped::SharedPtr goal,
                          geometry_msgs::msg::PoseStamped::SharedPtr pose,
                          std::string costmap_topic,
                          std::vector<std::array<double, 2>> footprint) = 0;

  virtual double scoreTrajectory(std::shared_ptr<Trajectory> tj) = 0;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_, pose_;

  double scale_;
};
}

#endif //XDWA_LOCAL_PLANNER_TRAJECTORY_SCORE_FUNCTION_H
