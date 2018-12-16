//
// Created by shivesh on 8/12/18.
//

#include "xdwa_local_planner/trajectory_scorer.h"

namespace xdwa_local_planner{
    TrajectoryScorer::TrajectoryScorer() {}

    TrajectoryScorer::~TrajectoryScorer() {}

    double TrajectoryScorer::getTrajectoryScore(std::shared_ptr<Trajectory> tj) {
        double traj_cost = 0;
        for(auto critic : critics_list_){
            double scale = critic->getScale();
            if(scale == 0)
                continue;
            double cost = critic->scoreTrajectory(tj);
            if(cost == -1)
                return -1;
            traj_cost += cost * scale;
        }
        return traj_cost;
    }
}