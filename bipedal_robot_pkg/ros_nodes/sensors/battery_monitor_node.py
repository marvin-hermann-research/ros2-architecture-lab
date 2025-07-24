import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatteryMonitor(Node):
    """
    Simulated battery publisher that broadcasts decreasing battery levels.

    Emulates a battery discharging at 10 mV/s. Used for evaluating
    behavior tree response thresholds (e.g. 'can_walk').

    Topic:
        - battery_monitor_data (sensor_msgs/BatteryState)
    """
    def __init__(self):
        super().__init__("battery_monitor")
        self._publisher = self.create_publisher(BatteryState, "battery_monitor_data", 10)
        self._battery_max_voltage = 7.5
        self._battery_voltage = self._battery_max_voltage
        self._timer = self.create_timer(1.0, self._publish_battery_state)

    def _publish_battery_state(self):
        msg = BatteryState()
        msg.voltage = self._battery_voltage
        msg.percentage = max(0.0, self._battery_voltage / self._battery_max_voltage)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self._publisher.publish(msg)

        self._battery_voltage -= 0.01  # simulate discharge rate

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
