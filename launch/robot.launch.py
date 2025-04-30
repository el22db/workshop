import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    models_path = os.path.expanduser("~/ros2_ws/src/workshop/urdf")
    controllers_path = os.path.join(models_path, "controller.yaml")

    return LaunchDescription([
        # 1. Start robot_state_publisher (optional but useful)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                "robot_description": open(os.path.join(models_path, "custom_robot.urdf"), 'r').read()
            }]
        ),

        # 2. Start ros2_control_node
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {
                    "robot_description": open(os.path.join(models_path, "custom_robot.urdf"), 'r').read()
                },
                controllers_path
            ],
            output="screen"
        ),

        # 3. Spawn robot in Gazebo
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "run", "gazebo_ros", "spawn_entity.py",
                        "-file", os.path.join(models_path, "custom_robot.urdf"),
                        "-entity", "custom_robot",
                        "-x", "-1.3", "-y", "-0.8", "-z", "0.25"
                    ],
                    output="screen"
                ),
            ]
        ),

        # 4. Spawn controller
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2", "run", "controller_manager", "spawner",
                        "gripper_controller",
                        "--controller-manager", "/controller_manager"
                    ],
                    output="screen"
                ),
            ]
        ),
    ])
