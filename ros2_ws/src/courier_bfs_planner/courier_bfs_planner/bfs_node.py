#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from collections import deque
import math


class BFSPlannerNode(Node):
    def __init__(self):
        super().__init__('bfs_planner_node')
        
        # Grid definition (6x6 grid with 1m cells)
        self.grid_size = 6
        self.cell_size = 1.0
        
        # Define obstacles (grid coordinates)
        self.obstacles = [
            (2, 1),  # obstacle_1
            (3, 3),  # obstacle_2
            (1, 3),  # obstacle_3
            (4, 2),  # obstacle_4
        ]
        
        # Start and goal
        self.start = (0, 0)  # Green cell
        self.goal = (5, 5)   # Blue cell
        
        # Publisher for planned path
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Service or trigger to plan path
        self.timer = self.create_timer(2.0, self.plan_and_publish)
        
        self.get_logger().info('BFS Planner Node Started')
        self.get_logger().info(f'Start: {self.start}, Goal: {self.goal}')
        self.get_logger().info(f'Obstacles: {self.obstacles}')

    def is_valid(self, pos):
        """Check if position is valid (within grid and not obstacle)"""
        x, y = pos
        if x < 0 or x >= self.grid_size or y < 0 or y >= self.grid_size:
            return False
        if pos in self.obstacles:
            return False
        return True

    def bfs_search(self):
        """BFS algorithm to find shortest path"""
        queue = deque([(self.start, [self.start])])
        visited = {self.start}
        
        # 4-connected grid (up, down, left, right)
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        
        while queue:
            (x, y), path = queue.popleft()
            
            # Check if goal reached
            if (x, y) == self.goal:
                self.get_logger().info(f'Path found with {len(path)} waypoints')
                return path
            
            # Explore neighbors
            for dx, dy in directions:
                next_pos = (x + dx, y + dy)
                
                if self.is_valid(next_pos) and next_pos not in visited:
                    visited.add(next_pos)
                    queue.append((next_pos, path + [next_pos]))
        
        self.get_logger().error('No path found!')
        return None

    def grid_to_world(self, grid_pos):
        """Convert grid coordinates to world coordinates (cell centers)"""
        x, y = grid_pos
        world_x = x * self.cell_size + 0.5  # Cell centers at 0.5, 1.5, 2.5, etc.
        world_y = y * self.cell_size + 0.5
        return world_x, world_y

    def plan_and_publish(self):
        """Plan path using BFS and publish as nav_msgs/Path"""
        path_grid = self.bfs_search()
        
        if path_grid is None:
            return
        
        # Convert to ROS Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for grid_pos in path_grid:
            world_x, world_y = self.grid_to_world(grid_pos)
            
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = BFSPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
