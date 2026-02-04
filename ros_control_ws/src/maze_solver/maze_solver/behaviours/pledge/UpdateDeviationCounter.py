import py_trees
from maze_solver.helpers import *

#STEP 1 DEL PLEDGE
class UpdateDeviationCounter(py_trees.behaviour.Behaviour):
    def __init__(self, name="Update Counter"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="pledge_counter", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="pledge_counter_log", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="last_action", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="pose", access=py_trees.common.Access.READ)
        self.BB.register_key(key="heading", access=py_trees.common.Access.READ)
    
    def update(self):
        counter = self.BB.get("pledge_counter") if self.BB.exists("pledge_counter") else 0
        last_action = self.BB.get("last_action") if self.BB.exists("last_action") else ""

        # Map actions to counter changes
        action_deltas = {
            "Turn Left": -1,
            "Turn Right": 1,
            "Turn Back (2 times left)": -2,
            "Turn Back (2 times right)": 2
        }
        
        # Update counter based on last action
        for action, delta in action_deltas.items():
            if action in last_action:
                counter += delta
                break
        
        self.BB.set("pledge_counter", counter)
        self.BB.set("pledge_counter_log", f"Pledge counter: {counter}")
        
        return py_trees.common.Status.SUCCESS