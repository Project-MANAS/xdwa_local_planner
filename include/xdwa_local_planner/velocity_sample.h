//
// Created by shivesh on 7/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_VELOCITY_SAMPLE_H
#define XDWA_LOCAL_PLANNER_VELOCITY_SAMPLE_H

#include <iostream>

namespace xdwa_local_planner{
    class VelocitySample{
    public:
        VelocitySample();
        VelocitySample(double vsample_x, double vsample_y, double vsample_theta,
                       double vminsample_x, double vminsample_y, double vminsample_theta,
                       double vmaxsample_x, double vmaxsample_y, double vmaxsample_theta);

        ~VelocitySample();

        friend
        std::ostream & operator << (std::ostream &out, const VelocitySample &vs);

        double vsample_x_, vsample_y_, vsample_theta_;
        double vminsample_x_, vminsample_y_, vminsample_theta_;
        double vmaxsample_x_, vmaxsample_y_, vmaxsample_theta_;
    };
}

#endif //XDWA_LOCAL_PLANNER_VELOCITY_SAMPLE_H
