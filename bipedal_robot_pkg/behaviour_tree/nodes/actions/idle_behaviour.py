import py_trees
from py_trees.common import Status

class IdleBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name = "Idle Behaviour")
        self._active = False
    
    def initialise(self):
        self._active = False
    
    def update(self):
        if not self._active:
            self._active = True
            return py_trees.common.Status.RUNNING
        
        #TODO logik einbauen wann nicht mehr ge idled wird und so und anmiation im movement controler start signalisieren

        return py_trees.common.Status.SUCCESS
