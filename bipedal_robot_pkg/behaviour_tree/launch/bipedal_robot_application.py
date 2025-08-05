import rclpy
from rclpy.executors import MultiThreadedExecutor
from py_trees_ros.trees import BehaviourTree

# Import custom ROS 2 nodes responsible for publishing actions
from bipedal_robot_pkg.ros_nodes.action_publishers import (
    idle_publisher,
    walk_forward_publisher
)

# Import actuator nodes for physical limb control
from bipedal_robot_pkg.ros_nodes.actuators import (
    left_leg_node,
    right_leg_node,
)

# Import high-level motion controller
from bipedal_robot_pkg.ros_nodes.controller import (
    movement_controller_node
)

# Import nodes responsible for runtime evaluation of locomotion conditions
from bipedal_robot_pkg.ros_nodes.evaluators import (
    can_walk_evaluator,
    must_walk_evaluator
)

# Import sensor interface nodes
from bipedal_robot_pkg.ros_nodes.sensors import (
    battery_monitor_node,
    imu_sensor_node,
    laser_sensor_node
)

# Import the tree factory used to build the behavior tree structure
from bipedal_robot_pkg.behaviour_tree.nodes import tree_factory


class BipedalRobotApplication:
    """
    The main application class responsible for initializing and managing
    the entire bipedal robot system, including behavior tree logic and
    all peripheral ROS 2 nodes.
    """
    def __init__(self):
        # === 1. Initialize the ROS 2 runtime ===
        rclpy.init()

        # === 2. Instantiate all ROS 2 nodes ===
        # Each node handles a specific subsystem such as actuation, sensing, or control
        self._nodes = {
            "idle_publisher": idle_publisher.IdlePublisher(),
            "walk_forward_publisher": walk_forward_publisher.WalkForwardPublisher(),
            "left_leg_node": left_leg_node.LeftLegNode(),
            "right_leg_node": right_leg_node.RightLegNode(),
            "movement_controller_node": movement_controller_node.MovementControllerNode(),
            "can_walk_evaluator": can_walk_evaluator.CanWalkEvaluator(),
            "must_walk_evaluator": must_walk_evaluator.MustWalkEvaluator(),
            "battery_monitor_node": battery_monitor_node.BatteryMonitorNode(),
            "imu_sensor_node": imu_sensor_node.ImuSensorNode(),
            "laser_sensor_node": laser_sensor_node.LaserSensorNode()
        }

        # === 3. Construct the behavior tree via a factory pattern ===
        # The tree defines high-level decision logic for locomotion
        self._factory = tree_factory.TreeFactory(self._nodes)
        root = self._factory.create_locomotion_tree()

        # === 4. Initialize the behavior tree ===
        # The 'BehaviourTree' integrates the tree into the ROS 2 timing system
        self._tree = BehaviourTree(
            root,
            False  # If True, prints ASCII representation of tree after each tick
        )
        self._tree.setup(timeout=15)

        # === 5. Register all other ROS 2 nodes with a MultiThreadedExecutor ===
        # Allows parallel execution of callbacks from all registered nodes
        self._executor = MultiThreadedExecutor()
        for node in self._nodes.values():
            self._executor.add_node(node)

    def run(self):
        """
        Launches the application by starting both the behavior tree and the ROS 2 executor.
        The behavior tree is periodically ticked, and the executor handles other callbacks.
        """
        try:
            # Periodically ticks the behavior tree every 100ms
            # Also internally invokes rclpy.spin() for the tree's own node context
            self._tree.tick_tock(period_ms=100)

            # Spins all other nodes via the executor (not automatically covered by tick_tock)
            self._executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        """
        Gracefully shuts down all nodes and the ROS 2 system to release resources properly.
        """
        for node in self._nodes.values():
            node.destroy_node()
        rclpy.shutdown()  

if __name__ == "__main__":
    app = BipedalRobotApplication()
    app.run()