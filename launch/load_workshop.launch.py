import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'world' argument with a default value
        DeclareLaunchArgument(
            'world_file',
            default_value='/home/daisy/ros2_ws/src/workshop/world/workshop_model.world',
            description='Path to the Gazebo world file'
        ),

        # Execute the Gazebo process with the specified world file
        ExecuteProcess(
            cmd=['ros2', 'launch', 'gazebo_ros', 'gzserver.launch.py',
                 'world:=/home/daisy/ros2_ws/src/workshop/world/workshop_model.world'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'gazebo_ros', 'gzclient.launch.py'],
            output='screen'
        ),
    ])
