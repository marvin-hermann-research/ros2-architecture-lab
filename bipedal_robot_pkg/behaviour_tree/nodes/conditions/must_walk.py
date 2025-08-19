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
    def __init__(self, ros2_logger):
        super().__init__(name="Must Walk?")
        self._blackboard = py_trees.blackboard.Client(name="MustWalkClient")
        self._blackboard.register_key(
            key="must_walk",
            access=py_trees.common.Access.READ
        )
        
        self._ros2_logger = ros2_logger.get_logger()

    def update(self):
        try:
            if self._blackboard.must_walk:
                self._ros2_logger.info("Must Walk condition succeeded.")
                return py_trees.common.Status.SUCCESS
            else:
                self._ros2_logger.info("Must Walk condition failed.")
                return py_trees.common.Status.FAILURE
        except AttributeError as e:
            self._ros2_logger.error(f"Must Walk blackboard key missing: {e}")
            return py_trees.common.Status.FAILURE
        except Exception as e:
            self._ros2_logger.error(f"Error in Must Walk update: {e}")
            return py_trees.common.Status.FAILURE