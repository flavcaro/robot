#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose
from collections import deque

class BFSPlanner(Node):
    def __init__(self):
        super().__init__('bfs_planner_node')
        self.map = None

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.waypoints_pub = self.create_publisher(PoseArray, '/courier/waypoints', 10)

        # start and goal cells (grid indices)
        self.start_cell = (0, 0)
        self.goal_cell = (4, 4)

        self.get_logger().info("BFS planner ready")

    def map_callback(self, msg):
        self.map = msg
        self.plan_path()

    def plan_path(self):
        if self.map is None:
            return

        width = self.map.info.width
        height = self.map.info.height
        data = self.map.data

        start_idx = self.start_cell[1] * width + self.start_cell[0]
        goal_idx = self.goal_cell[1] * width + self.goal_cell[0]

        path = self.bfs(start_idx, goal_idx, width, height, data)
        self.publish_path(path)

    def bfs(self, start_idx, goal_idx, width, height, data):
        neighbors = [(1,0),(-1,0),(0,1),(0,-1)]
        queue = deque([start_idx])
        visited = {start_idx: None}

        while queue:
            current = queue.popleft()
            if current == goal_idx:
                break

            x = current % width
            y = current // width

            for dx, dy in neighbors:
                nx, ny = x+dx, y+dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                n_idx = ny * width + nx
                if data[n_idx] > 50:  # obstacle
                    continue
                if n_idx not in visited:
                    visited[n_idx] = current
                    queue.append(n_idx)

        # reconstruct path
        cur = goal_idx
        path = []
        while cur is not None:
            path.append(cur)
            cur = visited.get(cur)
        path.reverse()
        return path

    def grid_to_world(self, idx):
        x_idx = idx % self.map.info.width
        y_idx = idx // self.map.info.width
        wx = self.map.info.origin.position.x + x_idx * self.map.info.resolution
        wy = self.map.info.origin.position.y + y_idx * self.map.info.resolution
        return wx, wy

    def publish_path(self, idx_list):
        pa = PoseArray()
        pa.header.frame_id = "map"
        for idx in idx_list:
            wx, wy = self.grid_to_world(idx)
            p = Pose()
            p.position.x = wx
            p.position.y = wy
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.waypoints_pub.publish(pa)
        self.get_logger().info(f"Published {len(pa.poses)} waypoints")

def main():
    rclpy.init()
    node = BFSPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
