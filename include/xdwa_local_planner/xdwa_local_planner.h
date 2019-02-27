//
// Created by shivesh on 7/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_XDWA_LOCAL_PLANNER_H
#define XDWA_LOCAL_PLANNER_XDWA_LOCAL_PLANNER_H

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/exceptions.hpp"

#include "xdwa_local_planner/trajectory.h"
#include "xdwa_local_planner/trajectory_generator.h"
#include "xdwa_local_planner/trajectory_score_function.h"

namespace xdwa_local_planner {
class XDWALocalPlanner : public rclcpp::Node {
 public:
  XDWALocalPlanner();
  ~XDWALocalPlanner();

  void pluginLoader(std::string type);

  std::string getCostmapTopic() {
    return costmap_topic_;
  }

  std::vector<std::vector<double>> getRobotFootprint() {
    return footprint_;
  }

 private:
  void computeTwist();

  bool getRobotPose();

  bool getLocalGoal();

  bool goalReached();

  void velocityCallback(nav_msgs::msg::Odometry::SharedPtr msg);

  bool computeBestTrajectory(std::shared_ptr<Trajectory> &best_traj);

  std::vector<std::shared_ptr<Trajectory>> getBestTrajectories(std::vector<std::shared_ptr<Trajectory>> trajectories);

  double control_freq_;
  std::string global_frame_, base_frame_;
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool compute_twist_stop_;
  std::thread compute_twist_thread_;

  tf2::Duration duration = tf2::Duration(std::chrono::seconds(1));
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  double transform_tolerance_;

  geometry_msgs::msg::PoseStamped::SharedPtr pose_, goal_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  std::string odom_topic_;
  bool vel_init_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::string goal_topic_;

  int depth_;
  int num_best_traj_;
  int num_steps_;

  double sim_time_, sim_period_;

  std::shared_ptr<TrajectoryGenerator> tg_;
  std::shared_ptr<TrajectoryScorer> ts_;

  std::string cmd_vel_topic_;
  geometry_msgs::msg::Twist cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;

  std::string costmap_topic_;
  std::vector<std::vector<double>> footprint_;
  std::vector<std::string> plugins_list_;
  pluginlib::ClassLoader<xdwa_local_planner::TrajectoryScoreFunction> plugin_loader_;
};
}

#endif //XDWA_LOCAL_PLANNER_XDWA_LOCAL_PLANNER_H
