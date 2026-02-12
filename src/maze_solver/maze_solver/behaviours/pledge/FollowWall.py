import py_trees
from maze_solver.helpers import *
import math

from custom_msg.action import ActuatorMove

#STEP 2 DEL PLEDGE
class FollowWall(py_trees.behaviour.Behaviour):
    def __init__(self, name="Follow Wall"):
        super().__init__(name)
        
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
        self.BB.register_key(key="actuator_movement_action_client", access=py_trees.common.Access.READ)
        self.BB.register_key(key="map", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="busy", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="logger", access=py_trees.common.Access.WRITE)

        self.BB.register_key(key="rotation_speed", access=py_trees.common.Access.READ)

    def update(self):
        current_pos = self.BB.get("current_position")
        heading = self.BB.get("heading")
        counter = self.BB.get("pledge_counter")
        last_action = self.BB.get("last_action")
        
        forward = forward_cell(current_pos, heading)
        
        # Check success condition: aligned with global heading and path is free
        if counter == 0 and heading == self.BB.get("heading_global") and is_free(self, forward):
            return py_trees.common.Status.SUCCESS
        
        # Get all adjacent cells
        left = left_cell(current_pos, heading)
        right = right_cell(current_pos, heading)
        back = backward_cell(current_pos, heading)
        backleft = backleft_cell(current_pos, heading)
        backright = backright_cell(current_pos, heading)
        
        # Determine turn priority based on pledge counter
        prioritize_left = counter > 0
        
        # Try movements in priority order
        new_heading, action = self._choose_direction(
            heading, forward, left, right, back, backleft, backright,
            last_action, prioritize_left
        )
        
        if action == "FAILURE":
            self.BB.set("last_action", "Wall ended → no free path")
            return py_trees.common.Status.FAILURE
        
        if heading == new_heading:
            return py_trees.common.Status.SUCCESS
        
        # ===================
        #      ROTATION
        # ===================
        self.BB.set("busy", True)
        angle = self.BB.get("heading") - new_heading
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
        
        self.BB.set("heading", new_heading)
        self.BB.set("last_action", action)
        
        return py_trees.common.Status.FAILURE
    
    def _choose_direction(self, heading, forward, left, right, back, backleft, backright, last_action, prioritize_left):
        """Choose direction based on pledge counter priority."""
        
        if prioritize_left:
            # Priority: left → forward → right → back
            if is_free(self, left) and not is_free(self, backleft) and "Turn Left" not in last_action:
                return (heading - 90) % 360, "Turn Left (wall follow)"
            if is_free(self, forward):
                return heading, "Continue forward"
            if is_free(self, right) and not is_free(self, backright):
                return (heading + 90) % 360, "Turn Right (wall follow)"
            if is_free(self, back):
                return (heading - 180) % 360, "Turn Back (2 times left)"
        else:
            # Priority: right → forward → left → back
            if is_free(self, right) and not is_free(self, backright) and "Turn Right" not in last_action:
                return (heading + 90) % 360, "Turn Right (wall follow)"
            if is_free(self, forward):
                return heading, "Continue forward"
            if is_free(self, left) and not is_free(self, backleft):
                return (heading - 90) % 360, "Turn Left (wall follow)"
            if is_free(self, back):
                return (heading - 180) % 360, "Turn Back (2 times left)"
        
        return heading, "FAILURE"
    
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