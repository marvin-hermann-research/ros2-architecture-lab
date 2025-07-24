import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import random
import math

class ImuSensor(Node):
    """
    Simulated IMU sensor publishing orientation and acceleration data.

    Provides a realistic but slightly noisy orientation using
    a Gaussian distribution and a stable Z-axis acceleration (~9.81 m/s²).
    """
    def __init__(self):
        super().__init__("imu_sensor")
        self._publisher = self.create_publisher(Imu, "imu_sensor_data", 10)
        self.timer = self.create_timer(0.05, self._publish_imu_data)  # 20 Hz

    def _publish_imu_data(self):
        _imu_msg = Imu()
        _imu_msg.header = Header()
        _imu_msg.header.stamp = self.get_clock().now().to_msg()
        _imu_msg.header.frame_id = "imu_frame"

        _roll = random.gauss(0.0, 0.01)
        _pitch = random.gauss(0.0, 0.01)
        _yaw = random.gauss(0.0, 0.01)
        _imu_msg.orientation = self._euler_to_quaternion(_roll, _pitch, _yaw)
        _imu_msg.orientation_covariance[0] = -1.0

        _imu_msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        _imu_msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        self._publisher.publish(_imu_msg)

    def _euler_to_quaternion(self, roll, pitch, yaw):
        # Standard Euler → Quaternion conversion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ImuSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
