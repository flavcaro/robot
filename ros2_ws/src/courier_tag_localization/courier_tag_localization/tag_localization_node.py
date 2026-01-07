import rclpy
from rclpy.node import Node

class TagLocalizationNode(Node):
    def __init__(self):
        super().__init__('tag_localization_node')
        self.get_logger().info("Tag Localization Node started")
        # Initialize AprilTag detection here

def main():
    rclpy.init()
    node = TagLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
