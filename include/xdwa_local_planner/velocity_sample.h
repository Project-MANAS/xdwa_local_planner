//
// Created by shivesh on 7/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_VELOCITY_SAMPLE_H
#define XDWA_LOCAL_PLANNER_VELOCITY_SAMPLE_H

#include <iostream>

namespace xdwa_local_planner {
class VelocitySample {
 public:
  VelocitySample();
  VelocitySample(double vsample_x, double vsample_y, double vsample_theta);

  ~VelocitySample();

  friend
  std::ostream &operator<<(std::ostream &out, const VelocitySample &vs);

  double vsample_x_, vsample_y_, vsample_theta_;
};
}

#endif //XDWA_LOCAL_PLANNER_VELOCITY_SAMPLE_H
