import py_trees
from py_trees.common import Status

class IdleBehaviour(py_trees.behaviour.Behaviour):
    """
    Behavior tree node representing the idle state of the robot.

    This node signals that the robot should remain in a minimal-motion 
    idle posture. It publishes an 'idle' command via a ROS publisher 
    when active, ensuring the command is continuously sent to maintain 
    the idle state.

    Status:
        - RUNNING on first activation to indicate ongoing action initiation.
        - SUCCESS after initial activation, reflecting that idle is active.
    
    Publishing Logic:
        - The first publish occurs once upon activation (when _active is False).
        - Subsequent publishes occur every update to keep the command active.
        - This avoids sending commands prematurely before activation.
    """
    def __init__(self, ros_publisher_node):
        super().__init__(name="Idle Behaviour")
        self._active = False
        self._ros_publisher = ros_publisher_node

    def initialise(self):
        # Reset the active flag on node reinitialization
        self._active = False

    def update(self):
        # On first update call after activation, publish once and mark active
        if not self._active:
            self._active = True
            self._ros_publisher.publish_message()
            return py_trees.common.Status.RUNNING
        
        # Continue publishing on every update to maintain idle command
        self._ros_publisher.publish_message()

        # This node does not self-terminate; it relies on BT parent control flow.
        return py_trees.common.Status.SUCCESS