from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Use gz sim (Gazebo Harmonic) instead of old gazebo_ros
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen',
            shell=False
        )
    ])
