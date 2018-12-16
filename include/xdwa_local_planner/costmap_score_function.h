//
// Created by shivesh on 10/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_COSTMAP_SCORE_FUNCTION_H
#define XDWA_LOCAL_PLANNER_COSTMAP_SCORE_FUNCTION_H

#include "xdwa_local_planner/trajectory.h"
#include "xdwa_local_planner/trajectory_score_function.h"

namespace xdwa_local_planner{
    class CostmapScoreFunction : public TrajectoryScoreFunction{
    public:
        CostmapScoreFunction();
        ~CostmapScoreFunction();

        double scoreTrajectory(std::shared_ptr<Trajectory> tj) override;

    private:
        bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
        double getCost(unsigned int mx, unsigned int my);

        int costmap_[10 * 10];
    };
}

#endif //XDWA_LOCAL_PLANNER_COSTMAP_SCORE_FUNCTION_H
