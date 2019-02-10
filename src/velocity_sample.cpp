//
// Created by shivesh on 8/12/18.
//

#include "xdwa_local_planner/velocity_sample.h"

namespace xdwa_local_planner {
VelocitySample::VelocitySample(double vsample_x, double vsample_y, double vsample_theta) :
    vsample_x_(vsample_x),
    vsample_y_(vsample_y),
    vsample_theta_(vsample_theta) {}

VelocitySample::~VelocitySample() {}

std::ostream &operator<<(std::ostream &out, const VelocitySample &vs) {
  out << vs.vsample_x_ << "\t" << vs.vsample_y_ << "\t" << vs.vsample_theta_ << std::endl;
  out << std::endl;
  return out;
}
}