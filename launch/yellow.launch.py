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
                        '-file', os.path.join(models_path, 'yellow_box.sdf'),
                        '-entity', 'yellow_box',
                        '-x', '-1.8', '-y', '-1.9', '-z', '1.25'
                    ],
                    output='screen'
                ),
            ]
        )
    ])
