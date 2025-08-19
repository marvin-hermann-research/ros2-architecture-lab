import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, LaserScan, BatteryState

class StatusLogger(Node):
    """
    Passive Status Logger Node for robot state monitoring.

    - Subscribes to:
        * 'imu_sensor_data' (sensor_msgs/Imu)
        * 'laser_sensor_data' (sensor_msgs/LaserScan)
        * 'battery_monitor_data' (sensor_msgs/BatteryState)
        * 'left_leg_instruction' (std_msgs/Float32MultiArray)
        * 'right_leg_instruction' (std_msgs/Float32MultiArray)
    """
    def __init__(self):
        super().__init__("status_logger")
        self._init_subscribers()

    def _init_subscribers(self):
        self.create_subscription(Imu, "imu_sensor_data", self._imu_listener_callback, 10)
        self.create_subscription(LaserScan, "laser_sensor_data", self._laser_listener_callback, 10)
        self.create_subscription(BatteryState, "battery_monitor_data", self._battery_listener_callback, 10)
        self.create_subscription(Float32MultiArray, "left_leg_instruction", self._left_leg_listener_callback, 10)
        self.create_subscription(Float32MultiArray, "right_leg_instruction", self._right_leg_listener_callback, 10)

    def _imu_listener_callback(self, data: Imu):
        pass
    def _laser_listener_callback(self, data: LaserScan):
        pass
    def _battery_listener_callback(self, data: BatteryState):
        pass
    def _left_leg_listener_callback(self, instruction: Float32MultiArray):
        pass
    def _right_leg_listener_callback(self, instruction: Float32MultiArray):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = StatusLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
