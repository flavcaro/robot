#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
import time
from geometry_msgs.msg import PoseArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class CourierBT(Node):
    def __init__(self):
        super().__init__('courier_bt_node')

        self.navigator = BasicNavigator()
        self.waypoints = []
        self.current_index = 0
        self.has_object = False

        self.sub = self.create_subscription(
            PoseArray, "/courier/waypoints", self.waypoints_callback, 10
        )

        self.tree = self.create_tree()
        self.get_logger().info("Courier BT ready")

    def waypoints_callback(self, msg):
        self.waypoints = msg.poses
        self.current_index = 0

    # --- BT actions ---
    def move_to_next_waypoint(self):
        if self.current_index >= len(self.waypoints):
            return py_trees.common.Status.SUCCESS

        pose = self.waypoints[self.current_index]
        self.navigator.goToPose(pose)
        result = self.navigator.waitUntilNavComplete()

        if result == TaskResult.SUCCEEDED:
            self.current_index += 1
            return py_trees.common.Status.RUNNING
        else:
            self.get_logger().warn("Nav failed, retrying...")
            return py_trees.common.Status.FAILURE

    def pickup_action(self):
        self.get_logger().info("Picking up object...")
        time.sleep(4.0)
        self.has_object = True
        return py_trees.common.Status.SUCCESS

    def deliver_action(self):
        self.get_logger().info("Delivering object...")
        time.sleep(4.0)
        self.has_object = False
        return py_trees.common.Status.SUCCESS

    # --- Build BT ---
    def create_tree(self):
        root = py_trees.composites.Sequence("Mission Root")

        move_seq = py_trees.behaviours.Running(name="Navigate Waypoints")
        move_seq.tick = lambda: self.move_to_next_waypoint()

        pickup = py_trees.behaviours.Running(name="Pickup Object")
        pickup.tick = lambda: self.pickup_action()

        deliver = py_trees.behaviours.Running(name="Deliver Object")
        deliver.tick = lambda: self.deliver_action()

        root.add_children([move_seq, pickup, move_seq, deliver])
        return py_trees.trees.BehaviourTree(root)

    def tick_tree(self):
        self.tree.tick()

def main():
    rclpy.init()
    node = CourierBT()
    rate = node.create_rate(1.0)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        node.tick_tree()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
