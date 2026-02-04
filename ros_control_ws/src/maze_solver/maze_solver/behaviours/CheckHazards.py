import py_trees
from maze_solver.helpers import *
from custom_msg.srv import Stop

class CheckHazards(py_trees.behaviour.Behaviour):

    def __init__(self, name="Check Hazards"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="hazards", access=py_trees.common.Access.READ)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)
        self.BB.register_key(key="actuator_stop_client", access=py_trees.common.Access.READ)


    def update(self):
        self.BB.get("logger").info(f"Hazards: {self.BB.get('hazards')}")
        if len(self.BB.get("hazards")) > 0:
            self.BB.get("logger").info("System has hazards!")

            # STOP Wheels
            req = Stop.Request()
            req.stop = True
            future = self.BB.get("actuator_stop_client").call_async(req)
            future.add_done_callback(self._response_callback)

            return py_trees.common.Status.SUCCESS
        self.BB.get("logger").info("System has NO hazards!")
        return py_trees.common.Status.FAILURE

    def _response_callback(self, future):
        try:
            if future.result().status:
                self.BB.get("logger").info("Wheels STOPPED")
            else:
                self.BB.get("logger").warning("Service not responding")
        except Exception as e:
            self.BB.get("logger").warning("Service not responding")