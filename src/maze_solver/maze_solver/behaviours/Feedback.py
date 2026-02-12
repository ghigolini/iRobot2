import py_trees
from custom_msg.action import Solve

class Feedback(py_trees.behaviour.Behaviour):

    def __init__(self, name="Feedback"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="goal_handle", access=py_trees.common.Access.READ)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.READ)
        

    def update(self):
        goal_handle = self.BB.get("goal_handle")

        # Feedback msg
        feedback_msg = Solve.Feedback()
        feedback_msg.current_position = self.BB.get("current_position")

        # Publishing feedback msg
        goal_handle.publish_feedback(feedback_msg)

        return py_trees.common.Status.SUCCESS