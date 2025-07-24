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
    def __init__(self):
        super().__init__(name="Can Walk?")
        self._blackboard = py_trees.blackboard.Client(name="CanWalkClient")
        self._blackboard.register_key(
            key="can_walk",
            access=py_trees.common.Access.READ
        )
    
    def update(self):
        if self._blackboard.can_walk:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
