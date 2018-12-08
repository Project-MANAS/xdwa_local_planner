//
// Created by shivesh on 8/12/18.
//

#include "xdwa_local_planner/velocity_sample.h"

namespace xdwa_local_planner{
    VelocitySample::VelocitySample(double vsample_x, double vsample_y, double vsample_theta,
                                   double vminsample_x, double vminsample_y, double vminsample_theta,
                                   double vmaxsample_x, double vmaxsample_y, double vmaxsample_theta):
     vsample_x_(vsample_x),
     vsample_y_(vsample_y),
     vsample_theta_(vsample_theta),
     vminsample_x_(vminsample_x),
     vminsample_y_(vminsample_y),
     vminsample_theta_(vminsample_theta),
     vmaxsample_x_(vmaxsample_x),
     vmaxsample_y_(vmaxsample_y),
     vmaxsample_theta_(vmaxsample_theta)
    {}

    VelocitySample::~VelocitySample() {}

    std::ostream & operator << (std::ostream &out, const VelocitySample &vs){
        out<<vs.vsample_x_<<"\t"<<vs.vsample_y_<<"\t"<<vs.vsample_theta_<<std::endl;
        out<<vs.vminsample_x_<<"\t"<<vs.vminsample_y_<<"\t"<<vs.vminsample_theta_<<std::endl;
        out<<vs.vmaxsample_x_<<"\t"<<vs.vmaxsample_y_<<"\t"<<vs.vmaxsample_theta_<<std::endl;
        out<<std::endl;
        return out;
    }
}