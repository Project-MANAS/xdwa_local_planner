//
// Created by shivesh on 8/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_TRAJECTORY_SCORER_H
#define XDWA_LOCAL_PLANNER_TRAJECTORY_SCORER_H

#include <memory>

#include "xdwa_local_planner/trajectory.h"
#include "xdwa_local_planner/trajectory_score_function.h"

namespace xdwa_local_planner{
    class TrajectoryScorer{
    public:
        TrajectoryScorer();
        ~TrajectoryScorer();
        double getTrajectoryScore(std::shared_ptr<Trajectory> tj);
        std::vector<TrajectoryScoreFunction*> critics_list_;
    };
}

#endif //XDWA_LOCAL_PLANNER_TRAJECTORY_SCORER_H
