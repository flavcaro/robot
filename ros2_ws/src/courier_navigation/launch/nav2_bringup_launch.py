from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start nav2 controller
        Node(
            package='nav2_bringup',
            executable='bringup_launch',
            name='nav2_bringup',
            output='screen'
        ),
        # Example: you could add more nodes here (AMCL, map server)
    ])
