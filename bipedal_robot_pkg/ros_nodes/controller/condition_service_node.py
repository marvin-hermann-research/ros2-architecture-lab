import rclpy
from rclpy.node import Node
import py_trees
from bipedal_robot_pkg.srv import SetCondition

class ConditionServiceNode(Node):

    def __init__(self):
        super().__init__("condition_service_node")