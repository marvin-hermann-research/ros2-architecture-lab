import rclpy
from rclpy.executors import MultiThreadedExecutor
from py_trees_ros.trees import BehaviourTree

from bipedal_robot_pkg.ros_nodes.action_publishers import (
    idle_publisher,
    walk_forward_publisher
)
from bipedal_robot_pkg.ros_nodes.actuators import (
    left_leg_node,
    right_leg_node,
)
from bipedal_robot_pkg.ros_nodes.controller import (
    movement_controller_node
)
from bipedal_robot_pkg.ros_nodes.evaluators import (
    can_walk_evaluator,
    must_walk_evaluator
)
from bipedal_robot_pkg.ros_nodes.sensors import (
    battery_monitor_node,
    imu_sensor_node,
    laser_sensor_node
)

from bipedal_robot_pkg.behaviour_tree.nodes import tree_factory


class BipedalRobotApplication:
    def __init__(self):
        rclpy.init()

        # Instantiate all ROS2 nodes
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

        # Create the behavior tree by using the TreeFactory
        self._factory = tree_factory.TreeFactory(self._nodes)
        root = self._factory.create_locomotion_tree()

        # Using 
        self._tree = BehaviourTree(
            root,
            False
            )
        
        # TODO finish this class AFTER changing the master document
