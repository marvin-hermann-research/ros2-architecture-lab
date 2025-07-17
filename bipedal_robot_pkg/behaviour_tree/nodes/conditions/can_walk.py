import py_trees

class canWalk(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name = "Can Walk?")
        self._blackboard = py_trees.blackboard.Client(name = "CanWalkClient")
        self._blackboard.register_key(
            key = "can_walk",
            access = py_trees.common.Access.READ
        )
    
    def update(self):
        if self._blackboard.can_walk:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE