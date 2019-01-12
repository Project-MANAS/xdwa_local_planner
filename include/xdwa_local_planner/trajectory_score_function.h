//
// Created by shivesh on 10/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_TRAJECTORY_SCORE_FUNCTION_H
#define XDWA_LOCAL_PLANNER_TRAJECTORY_SCORE_FUNCTION_H

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "xdwa_local_planner/trajectory.h"

namespace xdwa_local_planner{
    class TrajectoryScoreFunction{
    public:
        virtual void initialize(rclcpp::Node::SharedPtr node, std::string costmap_topic,
                                std::vector<std::vector<double>> footprint) = 0;

        virtual double scoreTrajectory(std::shared_ptr<Trajectory> tj) = 0;

        void setScale(double scale){
            scale_ = scale;
        }

        double getScale(){
            return scale_;
        }

    private:
        double scale_;
    };
}

#endif //XDWA_LOCAL_PLANNER_TRAJECTORY_SCORE_FUNCTION_H
