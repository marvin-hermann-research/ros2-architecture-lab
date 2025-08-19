import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryMonitorNode(Node):
    """
    Simulated Battery Monitor Node broadcasting battery state.

    - Frequency: 1 Hz
    - Simulates linear discharge at 10 mV/s
    - Publishes on topic: 'battery_monitor_data' (sensor_msgs/BatteryState)
    - Used for behavior tree evaluation (e.g., 'can_walk')
    """
    def __init__(self):
        super().__init__("battery_monitor")
        self._publisher = self.create_publisher(BatteryState, "battery_monitor_data", 10)
        self._battery_max_voltage = 7.5
        self._battery_voltage = self._battery_max_voltage
        self._timer = self.create_timer(1.0, self._publish_battery_state)

        self.get_logger().info("Battery Monitor Node has been started.")

    def _publish_battery_state(self):
        try:
            msg = BatteryState()
            msg.voltage = self._battery_voltage
            msg.percentage = max(0.0, self._battery_voltage / self._battery_max_voltage)
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            self._publisher.publish(msg)

            self._battery_voltage -= 0.01  # simulate discharge rate
        except Exception as e:
            self.get_logger().error(f"Failed to publish battery state: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
