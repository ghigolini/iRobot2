import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from maze_solver.helpers import *
from custom_msg.action import ActuatorMove
import math

class ChooseDirectionTremaux(Behaviour):
    def __init__(self, name="Move Forward Trémaux"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="chosen_direction", access=py_trees.common.Access.WRITE)
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
        self.BB.register_key(key="busy", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="actuator_movement_action_client", access=py_trees.common.Access.READ)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)
        self.BB.register_key(key="map", access=py_trees.common.Access.WRITE)

        self.BB.register_key(key="rotation_speed", access=py_trees.common.Access.READ)

    def update(self):
        if self.BB.get("reached_exit"):
            return Status.SUCCESS
        
        chosen_dir = self.BB.get("chosen_direction")
        
        if chosen_dir is None:
            return Status.FAILURE
        
        heading = self.BB.get("heading")
        
        if chosen_dir == heading:
            self.BB.get("logger").info("No rotation needed")
            return Status.SUCCESS
        
        self.BB.set("busy", True)

        # Rotation
        angle = heading - chosen_dir
        if angle == 270:
            angle = -90
        elif angle == -270:
            angle = 90
        angle = angle * math.pi / 180
        goal_msg = ActuatorMove.Goal()
        goal_msg.type = "ANGLE"
        goal_msg.distance = angle
        goal_msg.max_speed = self.BB.get("rotation_speed")

        actuator_movement_action_client = self.BB.get("actuator_movement_action_client")
        send_future = actuator_movement_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

        self.BB.set("heading", chosen_dir)  # Aggiorna orientamento

        self.BB.get("logger").info(f"Rotating angle: {angle}")

        return Status.FAILURE
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.BB.get("logger").warn("Goal rejected")
            self.BB.set("busy", False)
            return
        self.BB.get("logger").info("Rotation goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.end_movement_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.percentage
        self.BB.get("logger").info(f"Rotation percentage: {feedback}")

    def end_movement_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.BB.get("logger").info(f"Final status: {status}, result: {result}")
        self.BB.set("busy", False)