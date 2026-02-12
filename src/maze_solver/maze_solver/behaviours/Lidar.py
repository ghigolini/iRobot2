import py_trees
from time import sleep
from rclpy.time import Time
import math

from sensor_msgs.msg import LaserScan

class LidarMap(py_trees.behaviour.Behaviour):

    def __init__(self, name="LIDAR_MAP"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="lidar_scan", access=py_trees.common.Access.READ)
        self.BB.register_key(key="map", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="heading", access=py_trees.common.Access.READ)
        self.BB.register_key(key="clock", access=py_trees.common.Access.READ)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.READ)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)

        self.BB.register_key(key="angle", access=py_trees.common.Access.READ)
        self.BB.register_key(key="cell_length", access=py_trees.common.Access.READ)


    def update(self):
        # Check time
        # Bisogna scegliere tra Modello inverso quadratico distanza = sqrt(k / intensità)
        # Oppure Modello affine inverso distanza = a / (intensità - offset) + b
        # se si mappa in funzione della dimensione della cella
        # Basta controllare che d<=(5*cell_length)/8 per i raggi diagonali
        # Solo dopo una corretta calibrazione degli IR
        while(1):
            if(type(self.BB.get("lidar_scan")) == LaserScan):
                if((self.BB.get("clock").now() - Time.from_msg(self.BB.get("lidar_scan").header.stamp)).nanoseconds < 750_000_000):
                    self.BB.get("logger").info(f"LIDAR: {self.BB.get('lidar_scan')}")
                    angle_increment = self.BB.get("lidar_scan").angle_increment
                    map = self.BB.get("map")
                    direction = self.BB.get("heading")
                    chosen = ()

                    # First ray RIGHT
                    # bisogna leggere il VALUE dall'ir intesity di front_right (34 gradi dx)
                    first_ray = 1.5708 - self.BB.get("angle")
                    first_ray_distance = min(((5 * self.BB.get("cell_length")) / 8) / math.sin(self.BB.get("angle") + 0.01745), ((5 * self.BB.get("cell_length")) / 8) / math.sin(self.BB.get("angle") - 0.01745))
                    self.BB.get("logger").info(f"FIRST RAY DISTANCE MIN: {first_ray_distance}")
                    angle = 0
                    position = 0
                    while(angle < first_ray):
                        angle += angle_increment
                        position += 1

                    if(abs(angle - angle_increment - first_ray) < abs(angle - first_ray)):
                        position -= 1

                    if direction == 0:
                        chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1] + 1)
                    elif direction == 90:
                        chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1] + 1)
                    elif direction == 180:
                        chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1] - 1)
                    else:
                        chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1] - 1)

                    self.BB.get("logger").info(f"FIRST RAY DISTANCE: {self.BB.get('lidar_scan').ranges[position]}")
                    if(self.BB.get("lidar_scan").ranges[position] > first_ray_distance):
                        map[chosen] = "free"
                    else:
                        map[chosen] = "wall"

                    #Central ray
                    # bisogna leggere il VALUE dall'ir intesity di front_center_left (3 gradi sx)
                    central_ray = 1.5708
                    central_ray_distance = (self.BB.get("cell_length") * 9) / 8
                    self.BB.get("logger").info(f"CENTRAL RAY DISTANCE MIN: {central_ray_distance}")
                    angle = 0
                    position = 0
                    while(angle < central_ray):
                        angle += angle_increment
                        position += 1

                    if(abs(angle - angle_increment - central_ray) < abs(angle - central_ray)):
                        position -= 1

                    if direction == 0:
                        chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1])
                    elif direction == 90:
                        chosen = (self.BB.get("current_position")[0], self.BB.get("current_position")[1] + 1)
                    elif direction == 180:
                        chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1])
                    else:
                        chosen = (self.BB.get("current_position")[0], self.BB.get("current_position")[1] - 1)

                    self.BB.get("logger").info(f"CENTRAL RAY DISTANCE: {self.BB.get('lidar_scan').ranges[position]}")
                    if(self.BB.get("lidar_scan").ranges[position] > central_ray_distance):
                        map[chosen] = "free"
                    else:
                        map[chosen] = "wall"

                    #Last ray LEFT
                    # bisogna leggere il VALUE dall'ir intesity di left (38 gradi sx)
                    last_ray = 1.5708 + self.BB.get("angle")
                    last_ray_distance = min(((5 * self.BB.get("cell_length")) / 8) / math.sin(self.BB.get("angle") + 0.01745), ((5 * self.BB.get("cell_length")) / 8) / math.sin(self.BB.get("angle") - 0.01745))
                    self.BB.get("logger").info(f"LAST RAY DISTANCE MIN: {last_ray_distance}")
                    angle = 0
                    position = 0
                    while(angle < last_ray):
                        angle += angle_increment
                        position += 1

                    if(abs(angle - angle_increment - last_ray) < abs(angle - last_ray)):
                        position -= 1

                    if direction == 0:
                        chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1] - 1)
                    elif direction == 90:
                        chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1] + 1)
                    elif direction == 180:
                        chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1] + 1)
                    else:
                        chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1] - 1)

                    self.BB.get("logger").info(f"LAST RAY DISTANCE: {self.BB.get('lidar_scan').ranges[position]}")
                    if(self.BB.get("lidar_scan").ranges[position] > last_ray_distance):
                        map[chosen] = "free"
                    else:
                        map[chosen] = "wall"

                    self.print_map()

                    self.BB.get("logger").info("LIDAR Map ended!")

                    return py_trees.common.Status.SUCCESS

            sleep(0.200)


    def print_map(self):
        map = self.BB.get("map")
        if not map:
            self.BB.get("logger").info("Empty map")
            return

        righe = [coord[0] for coord in map.keys()]
        colonne = [coord[1] for coord in map.keys()]
        min_r, max_r = min(righe), max(righe)
        min_c, max_c = min(colonne), max(colonne)

        for r in range(max_r, min_r - 1, -1):
            row = []
            for c in range(min_c, max_c + 1):
                if (r, c) in map:
                    val = map[(r, c)]
                    row.append("#" if val == "wall" else "X" if val == "unmapped" else ".")
                else:
                    row.append("?")
            self.BB.get("logger").info(" ".join(row))