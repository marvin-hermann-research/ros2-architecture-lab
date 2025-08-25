import py_trees

class BehaviourCommandInterface:
    """
    Command interface for manipulating BT condition nodes via CLI.

    This class writes behavior triggers (e.g., 'must_walk') to the
    blackboard. Intended for runtime testing or manual override.

    TODO:
        - Extend input parsing for multiple command types
        - Support CLI arguments and runtime condition changes
    """
    def __init__(self):
        self._blackboard = py_trees.blackboard.Client(name="BehaviourCommandInterfaceClient")
        self._blackboard.register_key(
            key="must_walk",
            access=py_trees.common.Access.WRITE
        )
        # Future: command interface to trigger specific BT states
