import py_trees

class mustWalk(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name = "Must Walk?")
        self._blackboard = py_trees.blackboard.Client(name = "MustWalkClient")
        self._blackboard.register_key(
            key = "must_walk",
            access = py_trees.common.Access.READ
        )

    def update(self):
        if self._blackboard.must_walk:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

