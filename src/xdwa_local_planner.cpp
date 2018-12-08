//
// Created by shivesh on 7/12/18.
//

#include "xdwa_local_planner/xdwa_local_planner.h"
namespace xdwa_local_planner{
    XDWALocalPlanner::XDWALocalPlanner(){
        tg.generateSamples();
    }

    XDWALocalPlanner::~XDWALocalPlanner(){

    }

    void XDWALocalPlanner::computeTwist() {
        if(!tg.generateTrajectory(pose_x_, pose_y_, pose_theta_, vel_x_, vel_y_, vel_theta_, &traj))
            std::cout<<"!!!I am stuck!!!";
    }
}