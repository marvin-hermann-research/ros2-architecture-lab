from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class RightLegNode(Node):
    """
    Subscriber node representing the robot's right leg controller.

    This node listens for actuation commands on the "right_leg_instruction"
    topic and will later forward them to the hardware or simulation layer.
    """
    def __init__(self):
        super().__init__("right_leg")
        self._subscriber = self.create_subscription(
            Float32MultiArray,
            "right_leg_instruction",
            self._listener_callback,
            10
        )
        self.get_logger().info("Right leg subscriber initialized.")
        
    def _listener_callback(self, instruction: Float32MultiArray):
        # TODO: Implement execution logic for right leg instructions
        return
