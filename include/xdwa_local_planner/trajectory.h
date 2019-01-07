//
// Created by shivesh on 7/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_TRAJECTORY_H
#define XDWA_LOCAL_PLANNER_TRAJECTORY_H

#include <iostream>
#include <vector>

namespace xdwa_local_planner{
    class Trajectory{
    public:
        Trajectory();
        ~Trajectory();

        friend
        std::ostream & operator << (std::ostream &out, Trajectory traj);

        std::vector<double> x_, y_, theta_;
        std::vector<double> vel_x_, vel_y_, vel_theta_;
        double cost_;

        int num_points_, num_points_scored_;
    };
}

#endif //XDWA_LOCAL_PLANNER_TRAJECTORY_H
