//
// Created by shivesh on 7/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_XDWA_LOCAL_PLANNER_H
#define XDWA_LOCAL_PLANNER_XDWA_LOCAL_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/exceptions.hpp"

#include "xdwa_local_planner/trajectory.h"
#include "xdwa_local_planner/trajectory_generator.h"
#include "xdwa_local_planner/trajectory_score_function.h"

namespace xdwa_local_planner{
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
        void computeTwist(geometry_msgs::msg::PoseStamped::SharedPtr goal);

        bool getRobotPose();

        bool getLocalGoal(geometry_msgs::msg::PoseStamped::SharedPtr goal);

        bool goalReached(geometry_msgs::msg::PoseStamped::SharedPtr goal);

        void velocityCallback(nav_msgs::msg::Odometry::SharedPtr msg);

        bool computeBestTrajectory(std::shared_ptr<Trajectory> best_traj);

        std::vector<std::shared_ptr<Trajectory>> getBestTrajectories(std::vector<std::shared_ptr<Trajectory>> trajectories);

        double control_freq_;
        std::string global_frame_, base_frame_;
        double xy_goal_tolerance_, yaw_goal_tolerance_;

        tf2::Duration duration = tf2::Duration(std::chrono::seconds(1));
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener tfl;
        double transform_tolerance_;

        geometry_msgs::msg::PoseStamped pose_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        nav_msgs::msg::Odometry odom_;
        std::string odom_topic_;
        bool vel_init_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        std::string goal_topic_;

        int depth_;
        int num_best_traj_;
        int num_steps_;

        double sim_time_, sim_period_;

        TrajectoryGenerator tg_;
        TrajectoryScorer ts_;

        double pose_x_, pose_y_, pose_theta_;
        double vel_x_, vel_y_, vel_theta_;

        std::string cmd_vel_topic_;
        geometry_msgs::msg::Twist cmd_vel_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        std::vector<std::string> plugins_list_;
        pluginlib::ClassLoader<xdwa_local_planner::TrajectoryScoreFunction> plugin_loader_;
        std::string costmap_topic_;
        std::vector<std::vector<double>> footprint_;
    };
}

#endif //XDWA_LOCAL_PLANNER_XDWA_LOCAL_PLANNER_H
