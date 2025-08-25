import py_trees
from py_trees.composites import Selector
from py_trees.composites import Sequence
from bipedal_robot_pkg.behaviour_tree.nodes.conditions.can_walk import canWalk
from bipedal_robot_pkg.behaviour_tree.nodes.conditions.must_walk import mustWalk
from bipedal_robot_pkg.behaviour_tree.nodes.actions.walk_forward_behaviour import WalkForwardBehaviour
from bipedal_robot_pkg.behaviour_tree.nodes.actions.idle_behaviour import IdleBehaviour 

class TreeFactory:
    """
    Factory class for constructing behavior trees.

    This class provides an interface for generating modular tree structures
    used for controlling a bipedal robot. Trees are constructed using ROS node
    references and custom behavior/condition nodes.

    Attributes:
        ros_nodes (dict): Dictionary of ROS publishers, keyed by behavior type.
    """
    
    def __init__(self, ros_nodes: dict):
        self._ros_nodes = ros_nodes

    def create_locomotion_tree(self):
        """
        Constructs a locomotion tree with priority-based decision logic.

        Node priority:
            1. Walk Forward Sequence (if conditions met)
                - Can Walk?
                - Must Walk?
                - Walk Forward Action
            2. Idle Action (if no walking conditions are met)

        Returns:
            py_trees.composites.Selector: The root of the behavior tree.
        """
        try:
            root = Selector(
                name="Locomotion Root",
                memory=False
            )

            # Condition Nodes
            can_walk_condition = canWalk(self._ros_nodes["can_walk_evaluator"])
            must_walk_condition = mustWalk(self._ros_nodes["must_walk_evaluator"])

            # Action Nodes
            walk_forward_action = WalkForwardBehaviour(self._ros_nodes["walk_forward_publisher"])
            idle_action = IdleBehaviour(self._ros_nodes["idle_publisher"])

            # Add children in order of priority
            root.add_children([
                Sequence(
                    name = "Walk Forward Sequence",
                    memory=False,
                    children = [can_walk_condition, must_walk_condition, walk_forward_action]),
                idle_action
            ])

            return root
        except KeyError as e:
            print(f"[TreeFactory] Missing ROS node for tree construction: {e}")
        except Exception as e:
            print(f"[TreeFactory] Error creating locomotion tree: {e}")