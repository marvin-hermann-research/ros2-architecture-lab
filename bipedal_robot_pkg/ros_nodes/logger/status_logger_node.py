import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import BatteryState

class StatusLogger(Node):
    def __init__(self):
        super().__init__("status_logger")
        self._init_subscribers()
    
    def _init_subscribers(self):
        self._imu_sensor_subscriber = self.create_subscription(
            Imu,
            "imu_sensor_data",
            self._imu_listener_callback,
            10
        )
        self._laser_sensor_subscriber = self.create_subscription(
            LaserScan,
            "laser_sensor_data",
            self._laser_listener_callback,
            10
        )
        self._battery_monitor_subscriber = self.create_subscription(
            BatteryState,
            "battery_monitor_data",
            self._battery_listener_callback,
            10
        )
        self._left_leg_subscriber = self.create_subscription(
            Float32MultiArray,
            "left_leg_instruction",
            self._left_leg_listener_callback,
            10
        )
        self._right_leg_subscriber = self.create_subscription(
            Float32MultiArray,
            "right_leg_instruction",
            self._right_leg_listener_callback,
            10
        )
        self.get_logger().info("All logging subscribers initialized.")
    
    def _imu_listener_callback(self, data : Imu):
        return
    def _laser_listener_callback(self, data : LaserScan):
        return
    def _battery_listener_callback(self, data : BatteryState):
        return
    def _left_leg_listener_callback(self, instruction : Float32MultiArray):
        return
    def _right_leg_listener_callback(self, instruction : Float32MultiArray):
        return
    
def main(args = None):
    rclpy.init(args = args)
    node = StatusLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()