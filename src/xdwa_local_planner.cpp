//
// Created by shivesh on 7/12/18.
//

#include "xdwa_local_planner/xdwa_local_planner.h"

namespace xdwa_local_planner{
    XDWALocalPlanner::XDWALocalPlanner(){
        tg_.generateSamples();
    }

    XDWALocalPlanner::~XDWALocalPlanner(){

    }

    void XDWALocalPlanner::computeTwist() {
        std::shared_ptr<Trajectory> best_traj;
        if(!computeBestTrajectory(best_traj))
            std::cout<<"!!!I am stuck!!!";
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
}