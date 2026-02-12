import py_trees
from maze_solver.helpers import *

class CheckKidnap(py_trees.behaviour.Behaviour):

    def __init__(self, name="Check Kidnap"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="kidnapped", access=py_trees.common.Access.READ)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)
        

    def update(self):
        if self.BB.get("kidnapped")[0]:
            self.BB.get("logger").info("System is kidnapped!")
            return py_trees.common.Status.SUCCESS
        self.BB.get("logger").info("System is NOT kidnapped!")
        return py_trees.common.Status.FAILURE