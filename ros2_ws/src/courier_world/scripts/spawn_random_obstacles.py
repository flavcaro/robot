#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import random

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('spawn_random_obstacles')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.spawn_obstacles(10)

    def spawn_obstacles(self, count):
        for i in range(count):
            x = random.uniform(-4,4)
            y = random.uniform(-4,4)
            name = f'obstacle_{i}'

            req = SpawnEntity.Request()
            req.name = name
            req.xml = open('models/obstacle_box/model.sdf','r').read()
            req.reference_frame = 'world'
            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.position.z = 0

            self.cli.call_async(req)

def main():
    rclpy.init()
    node = ObstacleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
