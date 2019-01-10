//
// Created by shivesh on 7/12/18.
//

#include "xdwa_local_planner/xdwa_local_planner.h"

namespace xdwa_local_planner{
    XDWALocalPlanner::XDWALocalPlanner() :
            Node("costmap_ros"),
            control_freq_(10.0),
            global_frame_("map"),
            base_frame_("base_link"),
            xy_goal_tolerance_(0),
            yaw_goal_tolerance_(0),
            buffer_(get_clock()),
            tfl(buffer_),
            transform_tolerance_(1.0),
            odom_topic_("/odom"),
            vel_init_(false),
            goal_topic_("/goal"),
            cmd_vel_topic_("/cmd_vel")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>
                (odom_topic_, std::bind(&XDWALocalPlanner::velocityCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
                (goal_topic_, std::bind(&XDWALocalPlanner::computeTwist, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_);

        tg_.generateSamples();
    }

    XDWALocalPlanner::~XDWALocalPlanner(){

    }

    void XDWALocalPlanner::computeTwist(geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose = odom_.pose.pose;
        if (!getRobotPose(pose)) {
            RCLCPP_INFO(this->get_logger(), "Could not get robot pose");
            return;
        }

        rclcpp::Rate rate(1.0);
        while (!getLocalGoal(goal)) {
            rate.sleep();
        }

        rclcpp::Rate control_rate(control_freq_);
        while(!goalReached(goal)){
            rclcpp::Time start = get_clock()->now();
            std::shared_ptr<Trajectory> best_traj;
            if(computeBestTrajectory(best_traj)) {
                cmd_vel_.linear.x = best_traj->vel_x_[0];
                cmd_vel_.linear.y = best_traj->vel_y_[0];
                cmd_vel_.angular.z = best_traj->vel_theta_[0];
                cmd_vel_pub_->publish(cmd_vel_);
            }
            else {
                RCLCPP_INFO(this->get_logger(), "XDWA Local Planner failed to produce a valid path.");
            }

            control_rate.sleep();
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

    bool XDWALocalPlanner::getRobotPose(geometry_msgs::msg::PoseStamped &pose) {
        pose.header.frame_id = base_frame_;
        pose.header.stamp = get_clock()->now();
        rclcpp::Time start = get_clock()->now();
        try {
            rclcpp::Time time = rclcpp::Time(0);
            tf2::TimePoint tf2_time(std::chrono::nanoseconds(time.nanoseconds()));
            geometry_msgs::msg::TransformStamped tfp = buffer_.lookupTransform("odom", global_frame_, tf2_time);
            tf2::doTransform(pose, pose, tfp);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return false;
        }
        rclcpp::Time finish = get_clock()->now();
        if ((finish.seconds() - start.seconds()) > transform_tolerance_) {
            RCLCPP_WARN(this->get_logger(),
                        "XDWA Local Planner %s to %s transform timed out. Current time: %d, global_pose stamp %d, tolerance %d",
                        global_frame_.c_str(), base_frame_.c_str(), RCL_NS_TO_S(finish.nanoseconds()), pose.header.stamp,
                        transform_tolerance_);
        }
        return true;
    }

    bool XDWALocalPlanner::getLocalGoal(geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        rclcpp::Time start = get_clock()->now();
        try {
            rclcpp::Time time = rclcpp::Time(0);
            tf2::TimePoint tf2_time(std::chrono::nanoseconds(time.nanoseconds()));
            geometry_msgs::msg::TransformStamped tfp = buffer_.lookupTransform("odom", goal->header.frame_id, tf2_time);
            tf2::doTransform(goal, goal, tfp);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return false;
        }
        rclcpp::Time finish = get_clock()->now();
        if ((finish.seconds() - start.seconds()) > transform_tolerance_) {
            RCLCPP_WARN(this->get_logger(),
                        "XDWA Local Planner %s to odom transform timed out. Current time: %d, global_pose stamp %d, tolerance %d",
                        goal->header.frame_id.c_str(), RCL_NS_TO_S(finish.nanoseconds()), goal->header.stamp,
                        transform_tolerance_);
        }
        return true;
    }

    bool XDWALocalPlanner::goalReached(geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        return hypot(goal->pose.position.x - odom_.pose.pose.position.x, goal->pose.position.y - odom_.pose.pose.position.y) < xy_goal_tolerance_;
    }

    bool XDWALocalPlanner::computeBestTrajectory(std::shared_ptr<Trajectory> best_traj) {
        if(!tg_.samples_generated_)
            return false;

        std::vector<std::shared_ptr<Trajectory>> trajectories;
        for (auto &vsample : tg_.vsamples_) {
            std::shared_ptr<Trajectory> tj = std::make_shared<Trajectory>();
            if(tg_.generateTrajectory(vsample, pose_x_, pose_y_, pose_theta_, vel_x_, vel_y_, vel_theta_, sim_time_, num_steps_, tj)) {
                tj->cost_ = 0;
                tj->num_points_ = 0;
                tj->num_points_scored_ = 0;
                if(ts_.getTrajectoryScore(tj) >= 0){
                    tj->num_points_scored_ = tj->num_points_;
                    trajectories.push_back(tj);
                }
            }
        }
        if(trajectories.empty())
            return false;

        trajectories = getBestTrajectories(trajectories);

        for(int i = 1; i < depth_; ++i){
            std::vector<std::shared_ptr<Trajectory>> traj;
            for(auto &tj : trajectories){
                for(auto &vsample: tg_.vsamples_){
                    if(tg_.generateTrajectory(vsample, tj->x_.back(), tj->y_.back(), tj->theta_.back(), tj->vel_x_.back(),
                                              tj->vel_y_.back(), tj->vel_theta_.back(), sim_time_, num_steps_, tj)){
                        if(ts_.getTrajectoryScore(tj) >= 0) {
                            tj->num_points_scored_ = tj->num_points_;
                            traj.push_back(tj);
                        }
                    }
                }
            }
            trajectories = traj;
            if(trajectories.empty())
                return false;
        }

        best_traj = trajectories[0];
        for(auto &traj: trajectories){
            if(best_traj->cost_ > traj->cost_)
                best_traj = traj;
        }
        return true;
    }

    std::vector<std::shared_ptr<Trajectory>> XDWALocalPlanner::getBestTrajectories(std::vector<std::shared_ptr<Trajectory>> trajectories) {
        std::vector<std::shared_ptr<Trajectory>> best_traj;
        for(int i = 0; i < num_best_traj_ && i < trajectories.size(); ++i){
            auto tj = trajectories[i];
            int j = i;
            for(auto traj = trajectories[i]; j < trajectories.size(); traj = trajectories[++j]){
                if(tj->cost_ >= traj->cost_){
                    tj = traj;
                }
            }
            trajectories[j] = trajectories[i];
            trajectories[i] = tj;
            best_traj.push_back(tj);
        }
        return best_traj;
    }

    void XDWALocalPlanner::velocityCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        vel_init_ = true;
    }
}