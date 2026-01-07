import rclpy
from rclpy.node import Node
import py_trees

class CourierBTNode(Node):
    def __init__(self):
        super().__init__('courier_bt_node')
        self.get_logger().info("Courier BT Node started")

        # Create a simple example behavior tree
        root = py_trees.composites.Sequence("Mission Root")

        # Example: Move to pickup
        move_to_pickup = py_trees.behaviours.Success("MoveToPickup")
        root.add_child(move_to_pickup)

        # Example: Collect object
        collect_object = py_trees.behaviours.Success("CollectObject")
        root.add_child(collect_object)

        # Example: Return home
        return_home = py_trees.behaviours.Success("ReturnHome")
        root.add_child(return_home)

        self.behaviour_tree = py_trees.trees.BehaviourTree(root)
        self.behaviour_tree.setup(timeout=15)

        # Tick periodically
        self.timer = self.create_timer(1.0, self.tick_tree)

    def tick_tree(self):
        self.behaviour_tree.tick()

def main():
    rclpy.init()
    node = CourierBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
