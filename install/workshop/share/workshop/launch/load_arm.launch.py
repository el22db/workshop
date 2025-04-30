from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value='ros2_ws/src/Universal_Robots_ROS2_GZ_Simulation-ros2/ur_simulation_gz/urdf/ur_gz.urdf.xacro'
        ),
        DeclareLaunchArgument('robot_name', default_value='ur10'),

        # Logging message
        LogInfo(msg="Spawning Robot: $(arg robot_name)"),

	# Use `gz service` instead of `spawn_entity.py`
	spawn_ur_robot = ExecuteProcess(
	    cmd=[
		'gz', 'service', '-s', '/world/default/create',
		'-reqtype', 'gz.msgs.EntityFactory',
		'-reptype', 'gz.msgs.Boolean',
		'-timeout', '2000',
		'-req',
		'sdf_filename: "' + ros2_ws/src/Universal_Robots_ROS2_GZ_Simulation-ros2/ur_simulation_gz/urdf/ur_gz.urdf.xacro + '", name: "ur10", pose: {position: {x: 0, y: 0, z: 1}}'
	    ],
	    output='screen'
)

    ])

