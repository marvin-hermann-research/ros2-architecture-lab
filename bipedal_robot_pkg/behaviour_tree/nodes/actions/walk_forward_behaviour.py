import py_trees
from py_trees.common import Status

class WalkForwardBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name = "Walk Forward Behaviour")
        self._active = False
    
    def initialise(self):
        self._active = False
    
    def update(self):
        if not self._active:
            self._active = True
            return py_trees.common.Status.RUNNING
        
        #TODO logik einbauen wie und wann nicht mehr aktiv hier eben movement controler signalisieren dass er läuft und handhaben wann laufen fertig oder abgebrochen

        return py_trees.common.Status.SUCCESS
