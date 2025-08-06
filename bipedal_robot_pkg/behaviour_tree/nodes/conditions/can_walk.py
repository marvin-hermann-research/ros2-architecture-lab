import py_trees

class canWalk(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the agent *can* walk.

    This node queries the blackboard key 'can_walk', which is intended to be 
    maintained by a separate ROS node (e.g. a sensor-based evaluator).
    
    Returns:
        - SUCCESS if can_walk is True
        - FAILURE otherwise
    """
    def __init__(self, ros2_logger):
        super().__init__(name="Can Walk?")
        self._blackboard = py_trees.blackboard.Client(name="CanWalkClient")
        self._blackboard.register_key(
            key="can_walk",
            access=py_trees.common.Access.READ
        )
        
        self._ros2_logger = ros2_logger

    
    def update(self):
        if self._blackboard.can_walk:
            self._ros2_logger.info("Can Walk condition succeeded.")
            return py_trees.common.Status.SUCCESS
        else:
            self._ros2_logger.info("Can Walk condition failed.")
            return py_trees.common.Status.FAILURE
