import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WalkForwardPublisher(Node):
    """
    ROS2 node for initiating forward movement commands.

    Designed to integrate with BT action nodes that issue
    forward locomotion instructions to the movement controller.

    Topic:
        - walk_forward (std_msgs/String): Publishes "Walking forward"
    """
    def __init__(self):
        super().__init__("walk_forward_publisher")
        self.publisher_ = self.create_publisher(String, "walk_forward", 10)
        self.get_logger().info("Walk Forward Publisher Node has been started.")

    def publish_message(self):
        msg = String()
        msg.data = "Walking forward"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: '{msg.data}'")
