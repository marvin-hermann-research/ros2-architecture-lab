import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

class MovementControllerNode(Node):
    """
    Central controller responsible for executing locomotion instructions.

    This node subscribes to high-level commands such as "idle_status"
    and "walk_forward", and is responsible for translating them into
    low-level actuator instructions for both legs.

    Future Features:
        - Load movement sequences from YAML files
        - Parse and execute kinematic or trajectory scripts
    """
    def __init__(self):
        super().__init__("movement_controller")
        self._init_publishers()
        self._init_subscribers()

    def _init_publishers(self):
        self._left_leg_publisher = self.create_publisher(
            Float32MultiArray,
            "left_leg_instruction",
            10
        )
        self._right_leg_publisher = self.create_publisher(
            Float32MultiArray,
            "right_leg_instruction",
            10
        )
    
    def _init_subscribers(self):
        self._idle_publisher = self.create_subscription(
            String,
            "idle_status",
            self._idle_callback,
            10
        )
        self._walk_forward_publisher = self.create_subscription(
            String,
            "walk_forward",
            self._walk_forward_callback,
            10
        )

    def _idle_callback(self, instruction):
        # TODO: Implement behavior-specific idle instructions
        pass

    def _walk_forward_callback(self, instruction):
        # TODO: Generate and execute walk cycle commands
        pass
        
def main(args=None):
    rclpy.init(args=args)
    node = MovementControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
