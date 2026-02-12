import py_trees
from py_trees.composites import Sequence, Selector
import time

from maze_solver.behaviours import *

class BehaviouralTree:
    def __init__(self,
                 global_heading,
                 cell_length,
                 rotation_speed,
                 movement_distance,
                 movement_speed,
                 angle,
                 k,
                 goal_handle,
                 actuator_movement_action_client,
                 actuator_stop_client,
                 clock,
                 logger):
        
        BB = py_trees.blackboard.Client(name="Behavioural")
        self._BB = BB
        
        BB.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        BB.register_key(key="goal_position", access=py_trees.common.Access.WRITE)
        BB.register_key(key="paused", access=py_trees.common.Access.WRITE)
        BB.register_key(key="kidnapped", access=py_trees.common.Access.WRITE)
        BB.register_key(key="hazards", access=py_trees.common.Access.WRITE)
        BB.register_key(key="ir_sensors", access=py_trees.common.Access.WRITE)
        BB.register_key(key="lidar_scan", access=py_trees.common.Access.WRITE)
        BB.register_key(key="heading", access=py_trees.common.Access.WRITE)
        BB.register_key(key="reached_exit", access=py_trees.common.Access.WRITE)
        BB.register_key(key="last_action", access=py_trees.common.Access.WRITE)
        BB.register_key(key="visited", access=py_trees.common.Access.WRITE)
        BB.register_key(key="allow_visit_fallback", access=py_trees.common.Access.WRITE)
        BB.register_key(key="algorithm_mode", access=py_trees.common.Access.WRITE)
        BB.register_key(key="pledge_counter", access=py_trees.common.Access.WRITE)
        BB.register_key(key="heading_global", access=py_trees.common.Access.WRITE)
        BB.register_key(key="chosen_direction", access=py_trees.common.Access.WRITE)
        BB.register_key(key="map", access=py_trees.common.Access.WRITE)
        BB.register_key(key="busy", access=py_trees.common.Access.WRITE)
        BB.register_key(key="actuator_movement_action_client", access=py_trees.common.Access.WRITE)
        BB.register_key(key="actuator_stop_client", access=py_trees.common.Access.WRITE)
        BB.register_key(key="goal_handle", access=py_trees.common.Access.WRITE)
        BB.register_key(key="clock", access=py_trees.common.Access.WRITE)
        BB.register_key(key="logger", access=py_trees.common.Access.WRITE)
        BB.register_key(key="maze_walls", access=py_trees.common.Access.WRITE)
        BB.register_key(key="visits", access=py_trees.common.Access.WRITE)

        BB.register_key(key="movement_distance", access=py_trees.common.Access.WRITE)
        BB.movement_distance = movement_distance
        BB.register_key(key="movement_speed", access=py_trees.common.Access.WRITE)
        BB.movement_speed = movement_speed
        BB.register_key(key="rotation_speed", access=py_trees.common.Access.WRITE)
        BB.rotation_speed = rotation_speed
        BB.register_key(key="angle", access=py_trees.common.Access.WRITE)
        BB.angle = angle
        BB.register_key(key="k", access=py_trees.common.Access.WRITE)
        BB.k = k
        BB.register_key(key="cell_length", access=py_trees.common.Access.WRITE)
        BB.cell_length = cell_length

        BB.current_position = [0, 0]
        BB.goal_position = goal_handle.request.end_position
        BB.paused = False
        BB.kidnapped = False
        BB.hazards = []
        BB.ir_sensors = []
        BB.lidar_scan = []
        BB.set("maze_walls", set()) # blocked cells {(r,c)}
        BB.heading = 0 # 0=N, 90=E, 180=S, 270=W
        BB.reached_exit = False
        BB.last_action = "Idle"
        BB.visited = [0, 0] # visited cells set
        BB.allow_visit_fallback = False  # allows moving into visited if no unvisited options exist
        BB.chosen_direction = None
        BB.visits = {}

        # Algorithm choise
        BB.algorithm_mode = goal_handle.request.algorithm  #algoritmo di default
        BB.pledge_counter = 0
        BB.heading_global = global_heading # keep tracks of the turns made
        BB.busy = False

        BB.actuator_movement_action_client = actuator_movement_action_client
        BB.actuator_stop_client = actuator_stop_client
        BB.goal_handle = goal_handle
        BB.clock = clock
        BB.logger = logger
        
        map = self.init_map()
        BB.set("map", map)
        
        self.build_tree()
        
    def init_map(self):
        map = {}
        map[(0, 0)] = "free"
        return map
    
    def PledgeSubTree(self, name="Pledge Algorithm"):
        root = Sequence(name, memory = False)
        update_deviation_counter = UpdateDeviationCounter("Update Counter")
        follow_wall = FollowWall("Follow Wall")
        move_forward = MoveForward("Move Forward")
        root.add_children([update_deviation_counter, follow_wall, move_forward])
        return root

    def TremauxSubTree(self, name="Tremaux Algorithm"):
        root = Sequence(name, memory=False)

        choose_unvisited_path = ChooseUnvisitedPath()
        mark_path = MarkPath()
        choose_direction = ChooseDirectionTremaux()
        move_forward = MoveForwardTremaux()
        
        root.add_children([choose_unvisited_path, mark_path, choose_direction, move_forward])
    
        return root

    def build_tree(self):
        root = py_trees.composites.Sequence("Maze Solver", memory=False)
        
        availability_selector = Selector("Avaiability selector", memory=False)
        is_paused = IsPaused()
        check_kidnap = CheckKidnap()
        check_hazards = CheckHazards()
        is_busy = IsBusy()
        check_exit = CheckExit()
        
        pledge_branch_sequence = Sequence("Pledge Branch", memory=False)
        algorithm_selector_P = AlgorithmSelector("Is Pledge", "PLEDGE")
        pledge_subtree = self.PledgeSubTree()
        
        tremaux_branch_sequence = Sequence("Tremaux Branch", memory=False)
        algorithm_selector_T = AlgorithmSelector("Is Tremaux", "TREMAUX")
        tremaux_subtree = self.TremauxSubTree()
        
        choose_algorithm = Selector("Choose Algorithm", memory=False)
        choose_algorithm.add_children([pledge_branch_sequence, tremaux_branch_sequence])
        pledge_branch_sequence.add_children([algorithm_selector_P, pledge_subtree])
        tremaux_branch_sequence.add_children([algorithm_selector_T, tremaux_subtree])
        
        exec_sequence = Sequence("Exec Sequence", memory=False)

        availability_selector.add_children([is_paused, check_kidnap, check_hazards, is_busy, check_exit, exec_sequence])

        root.add_children([availability_selector])

        #lidar_node = LidarMap()
        ir_node = IRMap()
        feedback = Feedback()
        exec_sequence.add_children([ir_node, choose_algorithm, feedback])

        self.tree = py_trees.trees.BehaviourTree(root)
        
    def loop(self):
        while(1):
            if(self._BB.reached_exit):
                return True
            self.tree.tick()
            time.sleep(1)
    
    def blackboard_updater(self, paused, kidnapped, hazard, ir_sensors, lidar_scan):
        self._BB.paused = paused
        self._BB.kidnapped = kidnapped
        self._BB.hazards = hazard
        self._BB.ir_sensors = ir_sensors
        self._BB.lidar_scan = lidar_scan
            
