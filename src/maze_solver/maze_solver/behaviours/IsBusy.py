import py_trees
from maze_solver.helpers import *
from time import sleep

class IsBusy(py_trees.behaviour.Behaviour):

    def __init__(self, name="Is Busy"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="busy", access=py_trees.common.Access.READ)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)
        

    def update(self):
        if self.BB.get("busy"):
            self.BB.get("logger").info("System is busy!")
            sleep(0.050)
            return py_trees.common.Status.SUCCESS
        self.BB.get("logger").info("System is NOT busy!")
        return py_trees.common.Status.FAILURE