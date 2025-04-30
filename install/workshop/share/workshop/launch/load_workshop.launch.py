import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'world' argument with a default value
        DeclareLaunchArgument(
            'world_file',
            default_value='/home/daisy/ros2_ws/src/workshop/world/workshop.world',
            description='Path to the Gazebo world file'
        ),
        
        # Execute the Gazebo process with the specified world file
        #ExecuteProcess(
            #cmd=['gazebo', '--verbose', LaunchConfiguration('world_file')],
            #output='screen'
        #),#
    ])


