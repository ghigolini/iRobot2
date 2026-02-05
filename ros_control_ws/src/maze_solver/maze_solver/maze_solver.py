import rclpy
from rclpy import qos
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading, time
import math


from irobot_create_msgs.msg import DockStatus, KidnapStatus, HazardDetectionVector, IrIntensityVector

from custom_msg.action import Solve, ActuatorMove, ActuatorDock
from custom_msg.srv import Stop, Pause

from sensor_msgs.msg import LaserScan

from maze_solver.behavioural import BehaviouralTree


class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        # ====================
        #    PARAMETERS
        # ====================
        # Dichiarazione opzionale (serve per avere default se manca nel YAML)
        self.declare_parameter('global_heading', 0)
        self.declare_parameter('goal', [1, 1])
        self.declare_parameter('cell_length', 1.0)
        self.declare_parameter('rotation_speed', 0.5)
        self.declare_parameter('movement_distance', 1.0)
        self.declare_parameter('movement_speed', 1.0)
        self.declare_parameter('angle', 0.5760)
        self.declare_parameter('k', 1.0)

        # Lettura effettiva (se nel YAML c’è un valore, sovrascrive il default)
        self.global_heading = self.get_parameter('global_heading').value
        self.goal = self.get_parameter('goal').value
        self.cell_length = self.get_parameter('cell_length').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.movement_distance = self.get_parameter('movement_distance').value
        self.movement_speed = self.get_parameter('movement_speed').value
        self.angle = self.get_parameter('angle').value
        self.k = self.get_parameter('k').value

        # ====================
        # States
        # ====================
        self._is_docked = False
        self._is_kidnapped = [False]
        self._hazard = []
        self._ir_sensors = []
        self._lidar_scan = []
        self._algorithm = ""
        self._is_paused = [False]

        # ====================
        # Topic subscription
        # ====================
        
        # DOCK STATUS
        dock_qos = qos.QoSProfile(
            depth = 10,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._dock_status_subscription = self.create_subscription(
            DockStatus,
            'dock_status',
            self.execute_dock_status_callback,
            dock_qos
        )
        self._dock_status_subscription

        # KIDNAP STATUS
        kidnap_status_qos = qos.QoSProfile(
            depth = 10,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._kidnap_status_subscription = self.create_subscription(
            KidnapStatus,
            'kidnap_status',
            self.execute_kidnap_status_callback,
            kidnap_status_qos
        )
        self._kidnap_status_subscription

        # LIDAR SCAN
        lidar_scan_qos = qos.QoSProfile(
            depth = 1,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._lidar_scan_subscription = self.create_subscription(
            LaserScan,
            'lidar/scan',
            self.execute_lidar_scan_callback,
            lidar_scan_qos
        )
        self._lidar_scan_subscription

        # HAZARD VECTOR
        hazard_detection_qos = qos.QoSProfile(
            depth = 10,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._hazard_detection_subscription = self.create_subscription(
            HazardDetectionVector,
            'hazard_detection',
            self.execute_hazard_detection_callback,
            hazard_detection_qos
        )
        self._hazard_detection_subscription

        # IR VECTOR
        ir_intensity_qos = qos.QoSProfile(
            depth = 10,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._ir_intensity_subscription = self.create_subscription(
            IrIntensityVector,
            'ir_intensity',
            self.execute_ir_intensity_callback,
            ir_intensity_qos
        )
        self._ir_intensity_subscription


        # ====================
        # Services
        # ====================

        self._actuator_e_stop_client = self.create_client(
            Stop,
            'actuator_power'
        )


        # ====================
        # Action client subscriptions
        # ====================
        self._actuator_movement_action_client = ActionClient(
            self,
            ActuatorMove,
            'actuator_movement'
        )

        self._actuator_dock_action_client = ActionClient(
            self,
            ActuatorDock,
            'actuator_dock'
        )

        # ====================
        # Service servers
        # ====================
        self._actuator_pause_client = self.create_service(
            Pause,
            'actuator_pause',
            self.paused_callback
        )


        # ====================
        # Action servers
        # ====================
        self._solve_action_server = ActionServer(
            self,
            Solve,
            'maze_solve',
            execute_callback=self.execute_solve_callback,
            goal_callback=self.goal_solve_callback,
            cancel_callback=self.cancel_solve_callback
        )
    

    # ====================
    # Callbacks
    # ====================

    # ====================
    # Topic Callbacks
    # ====================

    # DOCK STATUS CALLBACK
    def execute_dock_status_callback(self, msg):
        self._is_docked = msg.is_docked

    # KIDNAP STATUS CALLBACK
    def execute_kidnap_status_callback(self, msg):
        self._is_kidnapped = [msg.is_kidnapped]
        
    def execute_pause_status_callback(self, msg):
        self._is_paused = [msg.is_paused]

    # LIDAR SCAN CALLBACK
    def execute_lidar_scan_callback(self, msg):
        self._lidar_scan = msg
    
    # HAZARD VECTOR CALLBACK
    def execute_hazard_detection_callback(self, msg):
        self._hazard.clear()
        for hazard in msg.detections:
            self._hazard.append(hazard)
    
    # IR INTENSITY CALLBACK
    def execute_ir_intensity_callback(self, msg):
        self._ir_sensors.clear()
        for ir in msg.readings:
            if(ir.header.frame_id == "ir_intensity_front_center_left") :
                self.get_logger().info("IR DISTANCE:")
                self.get_logger().info(str(math.sqrt(self.k / ir.value)))
            self._ir_sensors.append(ir)

    
    # ====================
    # Service Callbacks
    # ====================
    def paused_callback(self, request, response):
        self._is_paused = request.pause
        response.status = True
        return response
        

    # ====================
    # Action Callbacks
    # ====================

    # Decide if accept or refuse the current goal
    def goal_solve_callback(self, goal_request):
        if (goal_request.algorithm in ("PLEDGE", "TREMAUX") and
            len(goal_request.end_position) == 2 and
            min(goal_request.end_position) >= 0):
            
            return GoalResponse.ACCEPT
        
        self.get_logger().info(f"End: {goal_request.end_position}")
        self.get_logger().info(f"Algorithm: {goal_request.algorithm}")

        
        return GoalResponse.REJECT

    # Decide if the goal is cancellable
    def cancel_solve_callback(self, goal_handle):
        self.get_logger().info('Cancel goal requested')
        return CancelResponse.ACCEPT
    
    # Execute the goal
    def execute_solve_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Setting algoritm type
        self._algorithm = goal_handle.request.algorithm

        # Check if docked, then undock
        if self._is_docked:
            msg_goal = ActuatorDock.Goal()
            msg_goal.type = "UNDOCK"

            # Sync undock status
            future_goal = self._actuator_dock_action_client.send_goal_async(msg_goal)
            rclpy.spin_until_future_complete(self, future_goal)

            goal_handle_undock = future_goal.result()

            if not goal_handle_undock.accepted:
                self.get_logger().warn("Goal rifiutato")
                return

            self.get_logger().info("Goal accettato")

            future_result = goal_handle_undock.get_result_async()
            rclpy.spin_until_future_complete(self, future_result)

            if not future_result.result().result.status:
               self.get_logger().warn("Goal rifiutato")
               return
        

        # ====================
        #    BEHAVIOUR
        # ====================
        
        behav_tree = BehaviouralTree(self.global_heading,
                                     self.cell_length,
                                     self.rotation_speed,
                                     self.movement_distance,
                                     self.movement_speed,
                                     self.angle,
                                     self.k,
                                     goal_handle,
                                     self._actuator_movement_action_client,
                                     self._actuator_e_stop_client,
                                     self.get_clock(),
                                     self.get_logger())
        
        # Updating Thread because this language has no pointers
        def updater():
            while not stop_event.is_set():
                behav_tree.blackboard_updater(self._is_paused,
                                            self._is_kidnapped,
                                            self._hazard,
                                            self._ir_sensors,
                                            self._lidar_scan)
                time.sleep(0.02)
        
        stop_event = threading.Event()
        update_thread = threading.Thread(target=updater)
        update_thread.start()

        status = behav_tree.loop()

        stop_event.set()
        update_thread.join()

        if(status):
            # Publishing goal status
            goal_handle.succeed()
        else:
            goal_handle.abort()

        # Result
        result = Solve.Result()
        result.status = status
        return result


def main(args=None):
    rclpy.init(args=args)

    maze_solver_action_server = MazeSolver()

    executor = MultiThreadedExecutor(num_threads=10)
    executor.add_node(maze_solver_action_server)
    executor.spin()


if __name__ == '__main__':
    main()