from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class RightLeg(Node):
    """
    Subscriber node for the robot's right leg controller.

    This node listens on "right_leg_instruction" and is expected to
    execute or simulate physical motion accordingly.
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
