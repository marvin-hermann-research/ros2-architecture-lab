import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import random
import math

class LaserSensorNode(Node):
    """
    Simulated LIDAR Node emitting LaserScan messages.

    - Field of view: ±70° (total 140°)
    - Angular resolution: 10° per point
    - Range: 0.5 - 2.0 meters
    - Frequency: ~10 Hz (0.00714s timer)
    - Publishes on topic: 'laser_sensor_data'

    Future improvements:
    - Simulate specific obstacle patterns
    - Add configurable noise characteristics
    """
    def __init__(self):
        super().__init__("laser_sensor")
        self._publisher = self.create_publisher(LaserScan, "laser_sensor_data", 10)
        self._init_scanner_parameters()
        self.timer = self.create_timer(0.00714, self._scan_step)  # 10 Hz

        self.get_logger().info("Laser Sensor Node has been started.")

    def _init_scanner_parameters(self):
        self._angle_max = math.radians(70)
        self._angle_min = -self._angle_max
        self._angle_increment = math.radians(10)
        self._range_min = 0.5
        self._range_max = 2.0
        self._scan_data = []
        self._current_angle_index = 0
        self._point_amount = int((self._angle_max - self._angle_min) / self._angle_increment) + 1

    def _scan_step(self):
        _distance = random.uniform(self._range_min, self._range_max)
        self._scan_data.append(_distance)
        self._current_angle_index += 1

        if self._current_angle_index >= self._point_amount:
            self._send_scan_data()
            self._scan_data = []
            self._current_angle_index = 0

    def _send_scan_data(self):
        try:
            scan_msg = LaserScan()
            scan_msg.header = Header()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "laser_frame"
            scan_msg.angle_min = self._angle_min
            scan_msg.angle_max = self._angle_max
            scan_msg.angle_increment = self._angle_increment
            scan_msg.range_min = self._range_min
            scan_msg.range_max = self._range_max
            scan_msg.scan_time = 0.1
            scan_msg.time_increment = 0.00714
            scan_msg.ranges = self._scan_data
            scan_msg.intensities = []
            self._publisher.publish(scan_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish LaserScan: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
