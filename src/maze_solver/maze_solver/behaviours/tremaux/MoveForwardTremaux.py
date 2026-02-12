import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from maze_solver.helpers import *
from custom_msg.action import ActuatorMove

class MoveForwardTremaux(Behaviour):
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
        self.BB.register_key(key="visits", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="map", access=py_trees.common.Access.WRITE)

        self.BB.register_key(key="movement_distance", access=py_trees.common.Access.READ)
        self.BB.register_key(key="movement_speed", access=py_trees.common.Access.READ)

    def update(self):
        if self.BB.get("reached_exit"):
            return Status.SUCCESS
        
        current_position = self.BB.get("current_position")
        chosen_dir = self.BB.get("chosen_direction")
        
        self.BB.set("busy", True)
        
        target = forward_cell(current_position, chosen_dir)
        
        self.BB.set("current_position", target)

        # Movement
        goal_msg = ActuatorMove.Goal()
        goal_msg.type = "DISTANCE"
        goal_msg.distance = self.BB.get("movement_distance")
        goal_msg.max_speed = self.BB.get("movement_speed")
        actuator_movement_action_client = self.BB.get("actuator_movement_action_client")

        # Send goal with feedback callback
        send_future = actuator_movement_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

        self.BB.get("logger").info(f"Move forward")

        self.BB.set("last_action", f"Move {chosen_dir}° → {target}")
        visited = self.BB.get("visited")
        visited.append(target)
        self.BB.set("visited", visited)
        
        if target == self.BB.get("goal_position"):
            self.BB.set("reached_exit", True)
        
        return Status.SUCCESS
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.BB.get("logger").warn("Goal rejected")
            self.BB.set("busy", False)
            return
        self.BB.get("logger").info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.end_movement_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.percentage
        self.BB.get("logger").info(f"Movement percentage: {feedback}")

    def end_movement_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.BB.get("logger").info(f"Final status: {status}, result: {result}")
        self.BB.set("busy", False)