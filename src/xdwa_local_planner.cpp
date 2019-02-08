//
// Created by shivesh on 7/12/18.
//

#include "xdwa_local_planner/xdwa_local_planner.h"

namespace xdwa_local_planner {
XDWALocalPlanner::XDWALocalPlanner() :
    Node("costmap_ros"),
    control_freq_(20.0),
    global_frame_("map"),
    base_frame_("base_link"),
    xy_goal_tolerance_(0),
    yaw_goal_tolerance_(0),
    transform_tolerance_(1.0),
    odom_topic_("/odom"),
    vel_init_(false),
    goal_topic_("/goal"),
    depth_(0),
    num_best_traj_(5),
    num_steps_(5),
    cmd_vel_topic_("/cmd_vel"),
    costmap_topic_("/map"),
    plugin_loader_("xdwa_local_planner", "xdwa_local_planner::TrajectoryScoreFunction") {

  buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  tg_ = std::make_shared<TrajectoryGenerator>();
  ts_ = std::make_shared<TrajectoryScorer>();

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>
      (odom_topic_, std::bind(&XDWALocalPlanner::velocityCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
      (goal_topic_, std::bind(&XDWALocalPlanner::computeTwist, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_);

  tg_->generateSamples();

  plugins_list_.emplace_back("xdwa_local_planner::CostmapScoreFunction");
  plugins_list_.emplace_back("xdwa_local_planner::GoalDistScoreFunction");
  for (const std::string &type : plugins_list_) {
    pluginLoader(type);
  }
}

XDWALocalPlanner::~XDWALocalPlanner() {}

void XDWALocalPlanner::pluginLoader(std::string type) {
  RCLCPP_INFO(this->get_logger(), "Loading class %s", type.c_str());
  try {
    std::shared_ptr<TrajectoryScoreFunction> plugin = plugin_loader_.createSharedInstance(type);
    ts_->loadPlugin(plugin);
    plugin->initialize((rclcpp::Node::SharedPtr) this, buffer_, goal_, pose_, getCostmapTopic(), getRobotFootprint());
  }
  catch (pluginlib::LibraryLoadException &e) {
    RCLCPP_ERROR(this->get_logger(), "Class %s does not exist", type.c_str());
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Could not load class %s", type.c_str());
  }
}

void XDWALocalPlanner::computeTwist(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
  *goal_ = *goal;
  pose_->pose = odom_.pose.pose;

  rclcpp::Rate rate(1.0);
  while (!getLocalGoal()) {
    rate.sleep();
  }
  rclcpp::Rate control_rate(control_freq_);
  while (!goalReached()) {
    if (!getRobotPose()) {
      RCLCPP_INFO(this->get_logger(), "Could not get robot pose");
      control_rate.sleep();
      continue;
    }
    rclcpp::Time start = get_clock()->now();
    std::shared_ptr<Trajectory> best_traj;
    if (computeBestTrajectory(best_traj)) {
      cmd_vel_.linear.set__x(best_traj->vel_x_[0]);
      cmd_vel_.linear.set__y(best_traj->vel_y_[0]);
      cmd_vel_.angular.set__z(best_traj->vel_theta_[0]);
      cmd_vel_pub_->publish(cmd_vel_);
    } else {
      RCLCPP_INFO(this->get_logger(), "XDWA Local Planner failed to produce a valid path.");
    }

//    control_rate.sleep();
    rclcpp::Time finish = get_clock()->now();
    double time_taken = (finish.nanoseconds() - start.nanoseconds()) / 1e9;
    if (time_taken > 1.0 / control_freq_) {
      RCLCPP_WARN(this->get_logger(),
                  "Control loop failed. Desired frequency is %fHz. The loop actually took %f seconds",
                  control_freq_, time_taken);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Goal Reached");
}

bool XDWALocalPlanner::getRobotPose() {
  pose_->header.frame_id = base_frame_;
  pose_->header.stamp = get_clock()->now();
  rclcpp::Time start = get_clock()->now();
  try {
    geometry_msgs::msg::TransformStamped tfp = buffer_->lookupTransform("odom", global_frame_, tf2::TimePointZero);
    tf2::doTransform(*pose_, *pose_, tfp);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return false;
  }
  rclcpp::Time finish = get_clock()->now();
  if ((finish.seconds() - start.seconds()) > transform_tolerance_) {
    RCLCPP_WARN(this->get_logger(),
                "XDWA Local Planner %s to %s transform timed out. Current time: %d, global_pose stamp %d, tolerance %d",
                global_frame_.c_str(),
                base_frame_.c_str(),
                finish.seconds(),
                pose_->header.stamp,
                transform_tolerance_);
  }
  return true;
}

bool XDWALocalPlanner::getLocalGoal() {
  rclcpp::Time start = get_clock()->now();
  try {
    geometry_msgs::msg::TransformStamped tfp = buffer_->lookupTransform("odom", goal_->header.frame_id, tf2::TimePointZero);
    tf2::doTransform(*goal_, *goal_, tfp);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return false;
  }
  rclcpp::Time finish = get_clock()->now();
  if ((finish.seconds() - start.seconds()) > transform_tolerance_) {
    RCLCPP_WARN(this->get_logger(),
                "XDWA Local Planner %s to odom transform timed out. Current time: %d, global_pose stamp %d, tolerance %d",
                goal_->header.frame_id.c_str(),
                finish.seconds(),
                goal_->header.stamp,
                transform_tolerance_);
  }
  return true;
}

bool XDWALocalPlanner::goalReached() {
  return hypot(goal_->pose.position.x - pose_->pose.position.x, goal_->pose.position.y - pose_->pose.position.y)
      < xy_goal_tolerance_;
}

bool XDWALocalPlanner::computeBestTrajectory(std::shared_ptr<Trajectory> &best_traj) {
  if (!tg_->samples_generated_)
    return false;

  std::vector<std::shared_ptr<Trajectory>> trajectories;
  for (auto &vsample : tg_->vsamples_) {
    std::shared_ptr<Trajectory> tj = std::make_shared<Trajectory>();
    tj->cost_ = 0;
    tj->num_points_ = 0;
    tj->num_points_scored_ = 0;
    if (tg_->generateTrajectory(vsample,
                                pose_->pose.position.x,
                                pose_->pose.position.y,
                                tf2::getYaw(pose_->pose.orientation),
                                odom_.twist.twist.linear.x,
                                odom_.twist.twist.linear.y,
                                odom_.twist.twist.angular.z,
                                sim_time_,
                                num_steps_,
                                tj)) {
      if (ts_->getTrajectoryScore(tj) >= 0) {
        tj->num_points_scored_ = tj->num_points_;
        trajectories.push_back(tj);
      }
    }
  }

  if (trajectories.empty())
    return false;

  trajectories = getBestTrajectories(trajectories);
  for (int i = 1; i < depth_; ++i) {
    std::vector<std::shared_ptr<Trajectory>> traj;
    for (auto &tj : trajectories) {
      for (auto &vsample: tg_->vsamples_) {
        if (tg_->generateTrajectory(vsample, tj->x_.back(), tj->y_.back(), tj->theta_.back(), tj->vel_x_.back(),
                                    tj->vel_y_.back(), tj->vel_theta_.back(), sim_time_, num_steps_, tj)) {
          if (ts_->getTrajectoryScore(tj) >= 0) {
            tj->num_points_scored_ = tj->num_points_;
            traj.push_back(tj);
          }
        }
      }
    }
    trajectories = traj;
    if (trajectories.empty())
      return false;
  }
  best_traj = trajectories[0];
  for (auto &traj: trajectories) {
    if (best_traj->cost_ > traj->cost_)
      best_traj = traj;
  }
  return true;
}

std::vector<std::shared_ptr<Trajectory>> XDWALocalPlanner::getBestTrajectories(std::vector<std::shared_ptr<Trajectory>> trajectories) {
  std::vector<std::shared_ptr<Trajectory>> best_traj;
  for (int i = 0; i < num_best_traj_ && i < trajectories.size(); ++i) {
    std::shared_ptr<Trajectory> tj = trajectories[i];
    int j = i;
    int index = i;
    for (std::shared_ptr<Trajectory> traj = trajectories[j]; j < trajectories.size() - 1; traj = trajectories[++j]) {
      if (tj->cost_ >= traj->cost_) {
        tj = traj;
      }
      index = j;
    }
    *trajectories[index] = *trajectories[i];
    *trajectories[i] = *tj;
    best_traj.push_back(tj);
  }
  return best_traj;
}

void XDWALocalPlanner::velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = *msg;
  vel_init_ = true;
}
}