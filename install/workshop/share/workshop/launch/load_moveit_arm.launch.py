import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    )

    # Define the package path for workshop and the URDF file location
    package_path = os.path.join(get_package_share_directory('workshop'))
    xacro_file = os.path.join(package_path, 'urdf', 'panda.urdf')  

    # Parse the xacro file and generate the robot description
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'workshop'],
        output='screen'
    )

    # Return the launch description with gazebo and the spawn node
    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])



    
    
























    # gazebo_world_path = os.path.join(
    #     get_package_share_directory('workshop'), 'world', 'workshop.world'
    # )

    # # MoveIt! configuration path
    # moveit_config_path = os.path.join(
    #     get_package_share_directory('moveit_resources_panda_moveit_config'), 'launch', 'demo.launch.py'
    # )

    # # Gazebo launch arguments and nodes
    # return LaunchDescription([
    #     # Declare the world file argument
    #     DeclareLaunchArgument(
    #         'world_file',
    #         default_value=gazebo_world_path,
    #         description='Path to the Gazebo world file'
    #     ),

    #     # Launch Gazebo server with the specified world file
    #     Node(
    #         package='gazebo_ros',
    #         executable='/usr/bin/gzserver',
    #         name='gzserver',
    #         arguments=['--verbose', LaunchConfiguration('world_file')],
    #         output='screen'
    #     ),
        
    #     # Launch Gazebo client for GUI
    #     Node(
    #         package='gazebo_ros',
    #         executable='gzclient',
    #         name='gzclient',
    #         output='screen'
    #     ),

    #     # Include the MoveIt! launch for Panda robot with RViz2
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(moveit_config_path),
    #         launch_arguments={'use_rviz': 'true'}.items()
    #     ),

    #     # Logging message to notify user of successful launch
    #     LogInfo(msg="Launching Gazebo with Panda MoveIt! and RViz2")
    # ])

