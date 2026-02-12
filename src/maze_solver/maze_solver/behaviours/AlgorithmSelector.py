import py_trees
from maze_solver.helpers import *

class AlgorithmSelector(py_trees.behaviour.Behaviour):
    def __init__(self, name, mode):
        super().__init__(name)
        self.mode = mode
        
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="goal_position", access=py_trees.common.Access.READ)
        self.BB.register_key(key="paused", access=py_trees.common.Access.READ)
        self.BB.register_key(key="kidnapped", access=py_trees.common.Access.READ)
        self.BB.register_key(key="hazard", access=py_trees.common.Access.READ)
        self.BB.register_key(key="ir_sensors", access=py_trees.common.Access.READ)
        self.BB.register_key(key="lidar_scan", access=py_trees.common.Access.READ)
        self.BB.register_key(key="heading", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="reached_exit", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="last_action", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="visited", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="allow_visit_fallback", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="algorithm_mode", access=py_trees.common.Access.READ)
        self.BB.register_key(key="pledge_counter", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="heading_global", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)

    def update(self):
        self.BB.get("logger").info(self.BB.get("algorithm_mode"))
        self.BB.get("logger").info(self.mode)
        if self.BB.get("algorithm_mode") == self.mode:
            self.BB.get("logger").info("Algorithm is right")
            return py_trees.common.Status.SUCCESS
        else:
            self.BB.get("logger").info("Algorithm is NOT right")
            return py_trees.common.Status.FAILURE