//
// Created by shivesh on 10/12/18.
//

#include "xdwa_local_planner/costmap_score_function.h"

namespace xdwa_local_planner{
    CostmapScoreFunction::CostmapScoreFunction() {}

    CostmapScoreFunction::~CostmapScoreFunction() {}

    double CostmapScoreFunction::scoreTrajectory(std::shared_ptr<Trajectory> tj) {
        double cost = 0;
        for(int i = tj->num_points_scored_; i < tj->num_points_; ++i){
            unsigned int mx, my;
            if(!worldToMap(tj->x_[i], tj->y_[i], mx, my))
                return -1;
            cost += getCost(mx, my);
            tj->num_points_scored_++;
        }
        return cost;
    }

    bool CostmapScoreFunction::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my){
        return 1;
    }

    double CostmapScoreFunction::getCost(unsigned int mx, unsigned int my) {
        return 0;
    }
}