import rclpy
from rclpy import Node
from std_msgs.msg import String

class IdlePublisher(Node):
    """
    ROS2 node responsible for publishing idle state signals.

    This publisher is typically used within behavior tree action nodes
    to signal that the robot should remain stationary or enter an idle
    state (e.g., animation or no movement phase).

    Topic:
        - idle_status (std_msgs/String): Publishes the string "Idling"
    """
    def __init__(self):
        super().__init__("idle_publisher")
        self._publisher = self.create_publisher(String, "idle_status", 10)
        self.get_logger().info("Idle Publisher Node has been started.")

    def publish_message(self):
        msg = String()
        msg.data = "Idling"
        self._publisher.publish(msg)
        self.get_logger().info(f"Published: '{msg.data}'")
