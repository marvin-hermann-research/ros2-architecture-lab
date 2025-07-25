import py_trees
from py_trees.common import Status

class IdleBehaviour(py_trees.behaviour.Behaviour):
    """
    A behavior node that enters an idle state.
    
    Intended as a fallback action when no locomotion is required.
    This node will publish an 'idle' command via a ROS publisher 
    (to be implemented) when active. 

    Status:
        - RUNNING on first activation
        - SUCCESS after initial activation (placeholder logic)
    
    TODOs:
        - Integrate ROS publisher for idle signal
        - Implement logic to exit idle state or signal animation
    """
    def __init__(self, ros_publisher_node):
        super().__init__(name="Idle Behaviour")
        self._active = False
        # self._ros_publisher = ros_publisher_node  # [planned]

    def initialise(self):
        self._active = False

    def update(self):
        if not self._active:
            self._active = True
            return py_trees.common.Status.RUNNING
        
        # Placeholder: logic to determine exit condition and signal animation controller
        return py_trees.common.Status.SUCCESS
