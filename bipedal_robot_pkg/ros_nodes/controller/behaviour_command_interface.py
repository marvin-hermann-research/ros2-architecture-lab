import py_trees

class BehaviourCommandInterface():
    def __init__(self):
        self._blackboard = py_trees.blackboard.Client(name = "BehaviourCommandInterfaceClient")
        self._blackboard.register_key(
            key = "must_walk",
            access = py_trees.common.Access.WRITE
        )
        '''
        todo logik einbauen welche mir per command eingabe in der konsole erlaubt gewissen verhaltens musster anzufordern wie "must walk" must walk backwards" must idle" must bla bla
        '''