import rclpy
from rclpy.node import Node

class BFSPlannerNode(Node):
    def __init__(self):
        super().__init__('bfs_node')
        self.get_logger().info("BFS Planner Node started")
        # Initialize BFS pathfinding here

def main():
    rclpy.init()
    node = BFSPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
