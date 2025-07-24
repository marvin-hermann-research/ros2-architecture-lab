import py_trees

class mustWalk(py_trees.behaviour.Behaviour):
    """
    Condition node that queries whether the agent *must* walk.

    Uses the PyTrees blackboard system to read the 'must_walk' flag.
    This flag should be externally written via a console interface or
    ROS node (to be implemented).

    Returns:
        - SUCCESS if must_walk is True
        - FAILURE otherwise
    """
    def __init__(self):
        super().__init__(name="Must Walk?")
        self._blackboard = py_trees.blackboard.Client(name="MustWalkClient")
        self._blackboard.register_key(
            key="must_walk",
            access=py_trees.common.Access.READ
        )

    def update(self):
        if self._blackboard.must_walk:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
