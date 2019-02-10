//
// Created by shivesh on 8/12/18.
//

#include "xdwa_local_planner/trajectory_scorer.h"

namespace xdwa_local_planner {
TrajectoryScorer::TrajectoryScorer() {}

TrajectoryScorer::~TrajectoryScorer() {}

void TrajectoryScorer::loadPlugin(std::shared_ptr<xdwa_local_planner::TrajectoryScoreFunction> plugin) {
  critics_list_.push_back(plugin);
}

void TrajectoryScorer::getTrajectoryScore(std::shared_ptr<Trajectory> tj) {
  for (const auto &critic : critics_list_) {
    double scale = critic->scale_;
    if (scale == 0)
      continue;
    double cost = critic->scoreTrajectory(tj);
    if (cost == -1) {
      tj->cost_ = -1;
      break;
    }
    tj->cost_ += cost * scale;
  }
}
}