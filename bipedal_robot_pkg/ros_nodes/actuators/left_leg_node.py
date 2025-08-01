from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class LeftLegNode(Node):
    """
    Subscriber node representing the robot's left leg controller.

    This node listens for actuation commands on the "left_leg_instruction"
    topic and will later forward them to the hardware or simulation layer.
    """
    def __init__(self):
        super().__init__("left_leg")
        self._subscriber = self.create_subscription(
            Float32MultiArray,
            "left_leg_instruction",
            self._listener_callback,
            10
        )
        self.get_logger().info("Left leg subscriber initialized.")

    def _listener_callback(self, instruction: Float32MultiArray):
        # TODO: Implement execution logic for left leg instructions
        return
