//
// Created by shivesh on 8/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_TRAJECTORY_SCORER_H
#define XDWA_LOCAL_PLANNER_TRAJECTORY_SCORER_H

#include "xdwa_local_planner/trajectory.h"

namespace xdwa_local_planner{
    class TrajectoryScorer{
    public:
        TrajectoryScorer();
        ~TrajectoryScorer();
        static double scoreTrajectory(Trajectory tj);
    };
}

#endif //XDWA_LOCAL_PLANNER_TRAJECTORY_SCORER_H
