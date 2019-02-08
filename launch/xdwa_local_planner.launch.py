import os
import sys
import os
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
    	launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),

		#LOCAL PLANNER
        launch_ros.actions.Node(
            package='xdwa_local_planner', node_executable='xdwa_local_planner_node', node_name='xdwa_local_planner',
            output='screen', parameters=[{'use_sim_time':use_sim_time}]),
    ])
 
