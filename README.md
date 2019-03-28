# xDWA
xDWA is a local planner aimed to build upon the Dynamic Window Approach adding ability to build paths which are better in the long term and can perform several navigation tasks including parking and overtaking.

## Publications
- **/cmd_vel (geometry_msgs/Twist)**  
      Velocity commands for robot
- **/trajectories (nav_msgs/Path)**  
      Best trajectories after simulation cycle

## Subscriptions
- **/goal (geometry_msgs/PoseStamped)**  
      Goal for robot
- **/odom (nav_msgs/Odometry)**  
      Current odometry of robot

**Note**: This is a minimal version of the planner which publishes valid trajectories from the current position to a goal based on robot constraints without any obstacle avoidance.