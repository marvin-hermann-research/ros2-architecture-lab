import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MustWalkEvaluator(Node):
    """
    ROS2 Node to inject 'must_walk' intention into the behavior tree.

    Listens on '/must_walk_command' (std_msgs/Bool) and writes the value
    to the blackboard key 'must_walk'.

    Enables external control (e.g. CLI, GUI, higher-level logic) to influence
    walking behavior independently of sensor-derived decisions.
    """

    def __init__(self):
        super().__init__("must_walk_evaluator")
        self._init_subscribers()
        self._init_blackboard()

        self.get_logger().info("Must Walk Evaluator Node has been started.")

    def _init_subscribers(self):
        self._must_walk_subscriber = self.create_subscription(
            Bool, "must_walk_command", self._must_walk_listener_callback, 10
        )

    def _init_blackboard(self):
        self._blackboard = py_trees.blackboard.Client(name="MustWalkWriterClient")
        self._blackboard.register_key(
            key="must_walk",
            access=py_trees.common.Access.WRITE
        )

    def _must_walk_listener_callback(self, data: Bool):
        # Write incoming Bool to blackboard as 'must_walk' flag.
        self._blackboard.must_walk = data.data
