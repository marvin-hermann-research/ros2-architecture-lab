from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class LegLeft(Node):
    def __init__(self):
        super().__init__("left_leg")
        self._subscriber = self.create_subscription(
            Float32MultiArray,
            "left_leg_instruction",
            self._listener_callback,
            10
        )
        self.get_logger().info("Left leg subscriber initialized.")

    def _listener_callback(self, instruction : Float32MultiArray):
        return