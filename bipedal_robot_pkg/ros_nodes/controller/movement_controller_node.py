import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import BatteryState
import py_trees

class MovementController(Node):
    def __init__(self):
        super().__init__("movement_controller")
        self._init_publishers()

    def _init_publishers(self):
        self._left_leg_publisher = self.create_publisher(
            Float32MultiArray,
            "left_leg_instruction",
            10
        )
        self.get_logger().info("Left leg publisher initialized.")
        self._right_leg_publisher = self.create_publisher(
            Float32MultiArray,
            "right_leg_instruction",
            10
        )
        self.get_logger().info("Right leg publisher initialized.")
        
def main(args = None):
    rclpy.init(args = args)
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()