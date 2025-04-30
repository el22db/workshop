import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    models_path = os.path.expanduser("~/ros2_ws/src/workshop/objects")
    shelf_z = 1.25
    world_name = 'workshop'

    return LaunchDescription([
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                        '-file', os.path.join(models_path, 'green_cube.sdf'),
                        '-entity', 'green_cube',
                        '-x', '-1.4', '-y', '-1.8', '-z', '1.25'
                    ],
                    output='screen'
                ),
            ]
        )
    ])
