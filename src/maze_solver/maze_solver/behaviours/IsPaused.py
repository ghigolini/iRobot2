import py_trees
from maze_solver.helpers import *
from time import sleep

class IsPaused(py_trees.behaviour.Behaviour):

    def __init__(self, name="Is Paused"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="paused", access=py_trees.common.Access.READ)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)
        

    def update(self):
        if self.BB.get("paused")[0]:
            sleep(0.050)
            self.BB.get("logger").info("System is paused!")
            return py_trees.common.Status.SUCCESS
        self.BB.get("logger").info("System is NOT paused!")
        return py_trees.common.Status.FAILURE