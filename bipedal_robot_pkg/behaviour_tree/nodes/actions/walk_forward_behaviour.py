import py_trees
from py_trees.common import Status

class WalkForwardBehaviour(py_trees.behaviour.Behaviour):
    """
    Behavior tree node to command forward walking locomotion.

    This node publishes a 'walk forward' command to the movement controller 
    via a ROS publisher. It continuously sends the command once activated 
    to ensure uninterrupted walking instructions.

    Status:
        - RUNNING on first activation indicating the start of walking.
        - SUCCESS after initial activation, denoting the walking state.
    
    Publishing Logic:
        - Publishes once immediately after activation.
        - Continues publishing every update to maintain the walking command.
        - Avoids premature publishing before node activation.
    """
    def __init__(self, ros_publisher_node):
        super().__init__(name="Walk Forward Behaviour")
        self._active = False
        self._ros_publisher = ros_publisher_node
        self._ros_publisher.get_logger().info("Walk Forward Behaviour initialized.")

    def initialise(self):
        # Reset active state when node is (re)initialized
        self._active = False

    def update(self):
        # First publish upon activation to start walking
        if not self._active:
            self._active = True
        
        try:
            # Continue publishing to maintain walk command
            self._ros_publisher.publish_message()

            self._ros_publisher.get_logger().info("Walk Forward Behaviour is active, publishing walk command.")
        except Exception as e:
                self._ros_publisher.get_logger().error(f"Error publishing walk command: {e}")
                
        # Node does not self-terminate; control is managed by the BT parent nodes
        return py_trees.common.Status.RUNNING
