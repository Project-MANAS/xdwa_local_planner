//
// Created by shivesh on 10/12/18.
//

#include "xdwa_local_planner/costmap_score_function.h"

namespace xdwa_local_planner {
CostmapScoreFunction::CostmapScoreFunction() {}

CostmapScoreFunction::~CostmapScoreFunction() {}

void CostmapScoreFunction::initialize(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> buffer,
    geometry_msgs::msg::PoseStamped::SharedPtr goal,
    geometry_msgs::msg::PoseStamped::SharedPtr pose,
    std::string costmap_topic,
    std::vector<std::array<double, 2>> footprint
) {
  scale_ = 1;
  node_ = node;
  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>
      (costmap_topic, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
        costmap_ = costmap;
        for (signed char & i : costmap_->data) {
            i = std::max(static_cast<int8_t>(0), i);
        }
      });

  footprint_ = footprint;
  footprint_size_ = footprint_.size();

  calculateRadii();
}

double CostmapScoreFunction::scoreTrajectory(std::shared_ptr<Trajectory> tj) {
  double cost = 0;
  for (int i = tj->num_points_scored_; i < tj->num_points_; ++i) {
    unsigned int mx, my;
    if (!worldToMap(tj->x_[i], tj->y_[i], mx, my))
      return -1;
    auto temp_cost = getCost(tj->x_[i], tj->y_[i], tj->theta_[i]);
    if (temp_cost < 0) {
      cost = -1;
      break;
    }
    cost += temp_cost;
  }
  return cost;
}

bool CostmapScoreFunction::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) {
  double origin_x = costmap_->info.origin.position.x;
  double origin_y = costmap_->info.origin.position.y;
  double resolution = costmap_->info.resolution;
  unsigned int size_x = costmap_->info.width;
  unsigned int size_y = costmap_->info.height;
  if (wx < origin_x || wy < origin_y)
    return false;

  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
  my = static_cast<unsigned int>((wy - origin_y) / resolution);

  return mx < size_x && my < size_y;
}

double CostmapScoreFunction::getCost(double x, double y, double theta) {
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  double oriented_footprint[footprint_size_ - 1][2];
  for (unsigned long i = 0; i < footprint_size_; ++i) {
    oriented_footprint[i][0] = x + (footprint_[i][0] * cos_th - footprint_[i][1] * sin_th);
    oriented_footprint[i][1] = y + (footprint_[i][0] * sin_th + footprint_[i][1] * cos_th);
  }
  return footprintCost(x, y, oriented_footprint);
}

double CostmapScoreFunction::footprintCost(double x, double y, double footprint[][2]) {
  //used to put things into grid coordinates
  unsigned int cell_x, cell_y;

  //get the cell coord of the center point of the robot
  if (!worldToMap(x, y, cell_x, cell_y))
    return -1.0;

  //now we really have to lay down the footprint in the costmap grid
  unsigned int x0, x1, y0, y1;
  double line_cost;
  double footprint_cost = 0.0;

  //we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint_size_ - 1; ++i) {
    //get the cell coord of the first point
    if (!worldToMap(footprint[i][0], footprint[i][1], x0, y0))
      return -1.0;

    //get the cell coord of the second point
    if (!worldToMap(footprint[i + 1][0], footprint[i + 1][1], x1, y1))
      return -1.0;

    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    //if there is an obstacle that hits the line... we know that we can return false right away
    if (line_cost < 0)
      return -1.0;
  }

  //we also need to connect the first point in the footprint to the last point
  //get the cell coord of the last point
  if (!worldToMap(footprint[footprint_size_ - 1][0], footprint[footprint_size_ - 1][1], x0, y0))
    return -1.0;

  //get the cell coord of the first point
  if (!worldToMap(footprint[0][0], footprint[0][1], x1, y1))
    return -1.0;

  line_cost = lineCost(x0, x1, y0, y1);
  footprint_cost = std::max(line_cost, footprint_cost);

  if (line_cost < 0)
    return -1.0;

  //if all line costs are legal... then we can return that the footprint is legal
  return footprint_cost;
}

//calculate the cost of a ray-traced line
double CostmapScoreFunction::lineCost(int x0, int x1, int y0, int y1) const {
  double line_cost = 0.0;
  double point_cost;

  for (line_iterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY()); //Score the current point

    if (point_cost < 0)
      return -1;

    if (line_cost < point_cost)
      line_cost = point_cost;
  }

  return line_cost;
}

double CostmapScoreFunction::pointCost(int x, int y) const {
  auto cost = static_cast<unsigned char>(costmap_->data[y * costmap_->info.width + x]);
  //if the cell is in an obstacle the path is invalid
  //if(cost == LETHAL_OBSTACLE){
  if (cost == 254 || cost == 255) {
    return -1;
  }

  return cost;
}

void CostmapScoreFunction::calculateRadii() {
  inscribed_radius_ = std::numeric_limits<double>::max();
  circumscribed_radius_ = 0.0;

  if (footprint_size_ <= 2) {
    return;
  }

  for (unsigned long i = 0; i < footprint_size_ - 1; ++i) {
    // check the distance from the robot center point to the first vertex
    double vertex_dist = distance(0.0, 0.0, footprint_[i][0], footprint_[i][1]);
    double edge_dist = distanceToLine(0.0, 0.0, footprint_[i][0], footprint_[i][1],
                                      footprint_[i + 1][0], footprint_[i + 1][1]);
    inscribed_radius_ = std::min(inscribed_radius_, std::min(vertex_dist, edge_dist));
    circumscribed_radius_ = std::max(circumscribed_radius_, std::max(vertex_dist, edge_dist));
  }

  // we also need to do the last vertex and the first vertex
  double vertex_dist = distance(0.0, 0.0, footprint_[footprint_size_ - 1][0], footprint_[footprint_size_ - 1][1]);
  double edge_dist = distanceToLine(0.0, 0.0, footprint_[footprint_size_ - 1][0], footprint_[footprint_size_ - 1][1],
                                    footprint_[0][0], footprint_[0][1]);
  inscribed_radius_ = std::min(inscribed_radius_, std::min(vertex_dist, edge_dist));
  circumscribed_radius_ = std::max(circumscribed_radius_, std::max(vertex_dist, edge_dist));
}

double CostmapScoreFunction::distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1) {
  double A = pX - x0;
  double B = pY - y0;
  double C = x1 - x0;
  double D = y1 - y0;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;

  double xx, yy;

  if (param < 0) {
    xx = x0;
    yy = y0;
  } else if (param > 1) {
    xx = x1;
    yy = y1;
  } else {
    xx = x0 + param * C;
    yy = y0 + param * D;
  }

  return distance(pX, pY, xx, yy);
}
}

PLUGINLIB_EXPORT_CLASS(xdwa_local_planner::CostmapScoreFunction, xdwa_local_planner::TrajectoryScoreFunction)
