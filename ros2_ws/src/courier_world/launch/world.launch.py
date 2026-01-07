from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'src/courier_world/worlds/grid_world.sdf'],
            output='screen'
        ),
        Node(
            package='courier_world',
            executable='spawn_random_obstacles.py',
            output='screen'
        ),
    ])
