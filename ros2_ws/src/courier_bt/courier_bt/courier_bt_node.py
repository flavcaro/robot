#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import py_trees
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class WaitForPath(py_trees.behaviour.Behaviour):
    """Wait for BFS planner to publish path"""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        
    def update(self):
        if self.node.path_received:
            self.node.get_logger().info("✓ Path received from BFS planner")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class NavigateToGoal(py_trees.behaviour.Behaviour):
    """Navigate to goal using Nav2"""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.goal_sent = False
        
    def initialise(self):
        self.goal_sent = False
        
    def update(self):
        if not self.goal_sent:
            # Send goal to Nav2
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = 5.0
            goal_msg.pose.pose.position.y = 5.0
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.w = 1.0
            
            self.node.get_logger().info("→ Navigating to goal (5, 5)...")
            self.node.nav_action_client.send_goal_async(goal_msg)
            self.goal_sent = True
            
        # Check if robot reached goal (simplified - check position)
        if self.node.at_goal:
            self.node.get_logger().info("✓ Reached goal position")
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING

class GrabPackage(py_trees.behaviour.Behaviour):
    """Simulate package pickup"""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.start_time = None
        
    def initialise(self):
        self.start_time = time.time()
        self.node.get_logger().info("📦 Grabbing package...")
        
    def update(self):
        if time.time() - self.start_time > 2.0:
            self.node.has_package = True
            self.node.get_logger().info("✓ Package grabbed!")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class NavigateToStart(py_trees.behaviour.Behaviour):
    """Navigate back to start using Nav2"""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.goal_sent = False
        
    def initialise(self):
        self.goal_sent = False
        
    def update(self):
        if not self.goal_sent:
            # Send start position to Nav2
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = 0.0
            goal_msg.pose.pose.position.y = 0.0
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.w = 1.0
            
            self.node.get_logger().info("← Returning to start (0, 0)...")
            self.node.nav_action_client.send_goal_async(goal_msg)
            self.goal_sent = True
            
        # Check if robot reached start (simplified)
        if self.node.at_start:
            self.node.get_logger().info("✓ Returned to start - Mission complete!")
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING

class CourierBT(Node):
    def __init__(self):
        super().__init__('courier_bt_node')
        
        # State
        self.path_received = False
        self.has_package = False
        self.at_goal = False
        self.at_start = True
        
        # Subscribe to BFS planner path
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        # Nav2 action client
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Position tracking (simplified - subscribe to /odom or /amcl_pose in real system)
        self.timer = self.create_timer(0.5, self.check_position)
        
        # Build behavior tree
        self.tree = self.create_tree()
        
        self.get_logger().info("🤖 Courier Behavior Tree initialized")
        self.get_logger().info("Waiting for path from BFS planner...")
    
    def path_callback(self, msg):
        """Receive path from BFS planner"""
        if not self.path_received:
            self.path_received = True
            self.get_logger().info(f"Received path with {len(msg.poses)} waypoints")
    
    def check_position(self):
        """Simplified position checking - in real system, use odometry/AMCL"""
        # This is a placeholder - integrate with actual robot pose
        # For simulation purposes, we'll mark goals as reached after some time
        pass
    
    def create_tree(self):
        """Create behavior tree for courier mission"""
        root = py_trees.composites.Sequence("CourierMission", memory=False)
        
        # Mission sequence
        wait_path = WaitForPath("WaitForPath", self)
        nav_to_goal = NavigateToGoal("NavigateToGoal", self)
        grab = GrabPackage("GrabPackage", self)
        nav_to_start = NavigateToStart("NavigateToStart", self)
        
        root.add_children([wait_path, nav_to_goal, grab, nav_to_start])
        
        return py_trees.trees.BehaviourTree(root)
    
    def tick_tree(self):
        """Tick the behavior tree"""
        self.tree.tick()

def main():
    rclpy.init()
    node = CourierBT()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick_tree()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
