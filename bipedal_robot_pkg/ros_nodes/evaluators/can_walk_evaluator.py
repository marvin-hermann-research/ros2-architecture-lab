import py_trees
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, BatteryState, Imu
from std_msgs.msg import Header

class CanWalkEvaluator(Node):
    """
    Evaluates sensor data and determines if walking is feasible.

    This ROS2 node listens to IMU, LIDAR and battery input and
    publishes a boolean flag 'can_walk' to the behavior tree's blackboard.

    Evaluation Criteria:
        - Stable IMU (Z-axis acceleration ~9.81)
        - No obstacle detected in front
        - Battery level above 30%
    """
    def __init__(self):
        super().__init__("can_walk_evaluator")
        self._init_subscribers()
        self._init_blackboard()
        self._init_flags()

    def _init_flags(self):
        self._imu_grounded = False
        self._battery_ok = False
        self._no_obstacle = False

    def _init_blackboard(self):
        self._blackboard = py_trees.blackboard.Client(name="CanWalkWriterClient")
        self._blackboard.register_key(
            key="can_walk",
            access=py_trees.common.Access.WRITE
        )

    def _init_subscribers(self):
        self._imu_sensor_subscriber = self.create_subscription(
            Imu, "imu_sensor_data", self._imu_listener_callback, 10
        )
        self._laser_sensor_subscriber = self.create_subscription(
            LaserScan, "laser_sensor_data", self._laser_listener_callback, 10
        )
        self._battery_monitor_subscriber = self.create_subscription(
            BatteryState, "battery_monitor_data", self._battery_listener_callback, 10
        )

    def _imu_listener_callback(self, data: Imu):
        acc_z = data.linear_acceleration.z
        self._imu_grounded = 9.3 <= acc_z <= 10.3
        self._evaluate_can_walk()

    def _laser_listener_callback(self, data: LaserScan):
        mid_index = len(data.ranges) // 2
        window = data.ranges[mid_index - 1 : mid_index + 3]
        self._no_obstacle = all(distance >= 0.5 for distance in window)
        self._evaluate_can_walk()

    def _battery_listener_callback(self, data: BatteryState):
        self._battery_ok = data.percentage >= 0.3
        self._evaluate_can_walk()

    def _evaluate_can_walk(self):
        can_walk = self._imu_grounded and self._battery_ok and self._no_obstacle
        if self._blackboard.can_walk != can_walk:
            self._blackboard.can_walk = can_walk
