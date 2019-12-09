//
// Created by shivesh on 10/12/18.
//

#ifndef XDWA_LOCAL_PLANNER_COSTMAP_SCORE_FUNCTION_H
#define XDWA_LOCAL_PLANNER_COSTMAP_SCORE_FUNCTION_H

#include <utility>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "xdwa_local_planner/line_iterator.h"
#include "xdwa_local_planner/trajectory_score_function.h"

namespace xdwa_local_planner {
class CostmapScoreFunction : public TrajectoryScoreFunction {
 public:
  CostmapScoreFunction();
  ~CostmapScoreFunction();

  void initialize(rclcpp::Node::SharedPtr node,
                  std::shared_ptr<tf2_ros::Buffer> buffer,
                  geometry_msgs::msg::PoseStamped::SharedPtr goal,
                  geometry_msgs::msg::PoseStamped::SharedPtr pose,
                  std::string costmap_topic,
                  std::vector<std::array<double, 2>> footprint) override;

  double scoreTrajectory(std::shared_ptr<Trajectory> tj) override;

 private:
  bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my);
  double getCost(double x, double y, double theta);
  void calculateRadii();
  double footprintCost(double x, double y, double footprint[][2]);
  double lineCost(int x0, int x1, int y0, int y1) const;
  double pointCost(int x, int y) const;

  inline double distance(double x0, double y0, double x1, double y1) {
    return hypot(x1 - x0, y1 - y0);
  }

  double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);

  rclcpp::Node::SharedPtr node_;
  std::string costmap_topic_;
  std::vector<std::array<double, 2>> footprint_;
  unsigned long footprint_size_;
  double inscribed_radius_, circumscribed_radius_;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
};
}

#endif //XDWA_LOCAL_PLANNER_COSTMAP_SCORE_FUNCTION_H
