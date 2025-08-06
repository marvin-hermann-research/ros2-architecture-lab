import yaml
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

class MovementControllerNode(Node):
    """
    Central controller responsible for executing locomotion instructions.

    This ROS2 node translates high-level behavior tree (BT) actions—such as
    'idle' or 'walk_forward'—into low-level joint actuation commands via
    pre-defined motion patterns stored in YAML files.

    Key Responsibilities:
        - Subscribes to BT-issued command topics
        - Loads YAML-based trajectory patterns
        - Publishes joint commands to leg actuators
        - Executes motion sequences using timer-driven scheduling

    Future Extensions:
        - Trajectory interpolation
        - Parametrized gait generation
        - Dynamic pattern composition
    """
    def __init__(self):
        super().__init__("movement_controller")
        self._init_publishers()
        self._init_subscribers()
        
        # Load the pattern index which maps names to individual YAML files
        with open('patterns.yaml', 'r') as file:
            self._patterns = yaml.safe_load(file)

        self._current_pattern = None                  # Currently active motion sequence
        self._pattern_start_time = None               # Reference time for pattern playback
        self._pattern_step_index = 0                  # Current index in step sequence
        self._timer = self.create_timer(0.05, self._pattern_timer_callback)  # ~20 Hz control loop

        self.get_logger().info("Movement Controller Node has been started.")

    def _init_publishers(self):
        """
        Create ROS publishers for the left and right leg actuator topics.
        Each publishes a `Float32MultiArray` message representing joint angles.
        """
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
        """
        Register subscriptions to high-level behavior topics emitted by the BT.
        Triggers execution of corresponding motor patterns.
        """
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
        """
        Callback triggered when the IdleBehaviour node becomes active.
        Loads and begins execution of the 'idle' motion pattern.
        """
        self._load_and_start_pattern("idle")

    def _walk_forward_callback(self, instruction):
        """
        Callback triggered when the WalkForwardBehaviour node becomes active.
        Loads and begins execution of the 'walk_forward' motion pattern.
        """
        self._load_and_start_pattern("walk_forward")
    
    def _load_and_start_pattern(self, pattern_name):
        """
        Loads a motion pattern from YAML and prepares it for timed execution.

        Args:
            pattern_name (str): Name of the pattern to load, as defined in patterns.yaml
        """
        path = self._patterns.get(pattern_name)
        if path is None:
            self.get_logger().error(f"Pattern '{pattern_name}' not found in index.")
            return
        
        with open(path, 'r') as f:
            self._current_pattern = yaml.safe_load(f)[pattern_name]
            self.get_logger().info(f"Loaded pattern '{pattern_name}' from {path}")
        
        self._pattern_start_time = time.time()
        self._pattern_step_index = 0

    def _pattern_timer_callback(self):
        """
        Timer-driven loop for executing motion pattern steps in real time.

        Steps are executed when their defined timestamp (relative to pattern start)
        is reached or surpassed. When all steps have been processed, the current
        pattern is cleared.
        """
        if self._current_pattern is None or self._pattern_start_time is None:
            return

        elapsed = time.time() - self._pattern_start_time

        # Iterate through all due steps in current pattern
        while (self._pattern_step_index < len(self._current_pattern) and
               self._current_pattern[self._pattern_step_index]["time"] <= elapsed):
            step = self._current_pattern[self._pattern_step_index]
            self._execute_joints(step["joints"])
            self._pattern_step_index += 1

        if self._pattern_step_index >= len(self._current_pattern):
            self._current_pattern = None
            self.get_logger().info("Pattern execution completed.")

    def _execute_joints(self, joints):
        """
        Sends joint commands to the robot legs based on current step.

        Args:
            joints (dict): Dictionary with joint names as keys and angles in degrees as values.
        """
        left_msg = Float32MultiArray()
        right_msg = Float32MultiArray()

        left_msg.data = [
            float(joints.get("hip_left", 0.0)),
            float(joints.get("knee_left", 0.0)),
        ]
        right_msg.data = [
            float(joints.get("hip_right", 0.0)),
            float(joints.get("knee_right", 0.0)),
        ]

        self._left_leg_publisher.publish(left_msg)
        self._right_leg_publisher.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MovementControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
