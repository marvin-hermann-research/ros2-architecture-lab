import py_trees
from py_trees.composites import Selector
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
            1. Idle
            2. Can Walk?
            3. Must Walk?
            4. Walk Forward

        Returns:
            py_trees.composites.Selector: The root of the behavior tree.
        """
        root = Selector(
            name="Locomotion Root",
            memory=True
        )
        
        # Condition Nodes
        can_walk_condition = canWalk()
        must_walk_condition = mustWalk()
        
        # Action Nodes
        walk_forward_action = WalkForwardBehaviour(self._ros_nodes["walk_forward_publisher"])
        idle_action = IdleBehaviour(self._ros_nodes["idle_publisher"])

        # Add children in order of priority
        root.add_children([
            idle_action,
            can_walk_condition,
            must_walk_condition,
            walk_forward_action
        ])

        return root
