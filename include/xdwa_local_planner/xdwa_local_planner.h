//
// Created by shivesh on 7/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_XDWA_LOCAL_PLANNER_H
#define XDWA_LOCAL_PLANNER_XDWA_LOCAL_PLANNER_H

#include "xdwa_local_planner/trajectory.h"
#include "xdwa_local_planner/trajectory_generator.h"

namespace xdwa_local_planner{
    class XDWALocalPlanner{
    public:
        XDWALocalPlanner();
        ~XDWALocalPlanner();

    private:
        void computeTwist();
        bool computeBestTrajectory(std::shared_ptr<Trajectory> best_traj);
        std::vector<std::shared_ptr<Trajectory>> getBestTrajectories(std::vector<std::shared_ptr<Trajectory>> trajectories);

        int depth_;
        int num_best_traj_;
        int num_steps_;

        double sim_time_, sim_period_;

        TrajectoryGenerator tg_;
        TrajectoryScorer ts_;

        double pose_x_, pose_y_, pose_theta_;
        double vel_x_, vel_y_, vel_theta_;
    };
}

#endif //XDWA_LOCAL_PLANNER_XDWA_LOCAL_PLANNER_H
