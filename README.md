# xDWA
xDWA is a local planner aimed to build upon the Dynamic Window Approach adding ability to build paths which are better in the long term and can perform several navigation tasks including parking and overtaking.

## Publications
- **/cmd_vel (geometry_msgs/Twist)**  
      Velocity commands for robot
- **/trajectories (nav_msgs/Path)**  
      Best trajectories after simulation cycle

## Subscriptions
- **/move_base_simple/goal (geometry_msgs/PoseStamped)**  
      Goal for robot (can be given through RViz)
- **/odom (nav_msgs/Odometry)**  
      Current odometry of robot
- **/map (nav_msgs/OccupancyGrid)**  
      Costmap for obstacles avoidance

**Note**: Currently two trajectory scorers are implemented
1) Costmap Score Function
2) Goal Distance Score Function