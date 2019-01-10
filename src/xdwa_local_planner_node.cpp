//
// Created by shivesh on 10/1/19.
//

#include "xdwa_local_planner/xdwa_local_planner.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto xdwa_local_planner_node = std::make_shared<xdwa_local_planner::XDWALocalPlanner>();
    rclcpp::spin(xdwa_local_planner_node);
    rclcpp::shutdown();
    return 0;
}


