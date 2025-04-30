import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get paths
    workshop_world_path = '/home/daisy/ros2_ws/src/workshop/world/workshop.world'
    urdf_file_path = '/home/daisy/ros2_ws/src/Universal_Robots_ROS2_GZ_Simulation-ros2/ur_simulation_gz/urdf/ur_gz.urdf.xacro'
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Declare Launch Arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'world_file',
            default_value=workshop_world_path,
            description='Path to the Gazebo world file'
        ),
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur10',  # Change to match your robot model
            description='Type of Universal Robot (ur3, ur5, ur10, etc.)'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',  # 'false' if using real robot
            description='Use fake hardware instead of real UR robot'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.102',  # Change to your robot's IP if real
            description='IP address of the robot'
        )
    ]

    # Launch Gazebo with the world file
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', LaunchConfiguration('world_file')],
        output='screen'
    )

    # Spawn the UR robot inside Gazebo
    spawn_ur_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ur10_new',
            '-file', urdf_file_path,
            '-x', '0', '-y', '0', '-z', '1'  # Adjust spawn position if needed
        ],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [gazebo, spawn_ur_robot])

