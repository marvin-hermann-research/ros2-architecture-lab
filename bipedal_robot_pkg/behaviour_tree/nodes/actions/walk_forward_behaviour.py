import py_trees
from py_trees.common import Status

class WalkForwardBehaviour(py_trees.behaviour.Behaviour):
    """
    A behavior node that initiates forward locomotion.

    Upon activation, this node is intended to publish a walking command 
    to the movement controller (via ROS publisher, not yet integrated).

    Status:
        - RUNNING on first activation indicating the start of walking.    
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
        self._active = False

    def update(self):
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
