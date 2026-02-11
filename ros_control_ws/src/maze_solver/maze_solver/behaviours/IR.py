import py_trees
from time import sleep
from rclpy.time import Time
import math

class IRMap(py_trees.behaviour.Behaviour):

    def __init__(self, name="IR_MAP"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="ir_sensors", access=py_trees.common.Access.READ)
        self.BB.register_key(key="map", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="heading", access=py_trees.common.Access.READ)
        self.BB.register_key(key="clock", access=py_trees.common.Access.READ)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.READ)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)

        self.BB.register_key(key="k", access=py_trees.common.Access.READ)
        self.BB.register_key(key="cell_length", access=py_trees.common.Access.READ)
        

    def update(self):
        # Check time
        # Bisogna scegliere tra Modello inverso quadratico distanza = sqrt(k / intensità)
        # Oppure Modello affine inverso distanza = a / (intensità - offset) + b
        # se si mappa in funzione della dimensione della cella 
        # Basta controllare che d<=(5*cell_length)/8 per i raggi diagonali
        # Solo dopo una corretta calibrazione degli IR
        while(1):
            self.BB.get("logger").info(f"Reading IR")
            k = self.BB.get("k")
            ir_sensors = self.BB.get("ir_sensors")
            map = self.BB.get("map")
            direction = self.BB.get("heading")
            chosen = ()

            # First ray RIGHT
            # bisogna leggere il VALUE dall'ir intesity di front_right (34 gradi dx)
            right_ray_distance = []
            central_ray_distance = []
            left_ray_distance = []

            while(ir_sensors == []):
                sleep(0.02)
            t = 0
            while(t < 20):
                for r in ir_sensors:
                    if r.header.frame_id == "ir_intensity_front_right":
                        if(r.value == 0) :
                            right_ray_distance.append(2.0)
                        else :
                            right_ray_distance.append(math.sqrt(k / r.value))
                    elif r.header.frame_id == "ir_intensity_front_center_left":
                        if(r.value == 0) :
                            central_ray_distance.append(2.0)
                        else :
                            central_ray_distance.append(math.sqrt(k / r.value))
                    elif r.header.frame_id == "ir_intensity_left":
                        if(r.value == 0) :
                            left_ray_distance.append(2.0)
                        else :
                            left_ray_distance.append(math.sqrt(k / r.value))
                t += 1
                sleep(0.2)
            
            right_ray_distance = sum(right_ray_distance) / len(right_ray_distance)
            central_ray_distance = sum(central_ray_distance) / len(central_ray_distance)
            left_ray_distance = sum(left_ray_distance) / len(left_ray_distance)

            right_ray_distance_min = ((5 * self.BB.get("cell_length")) / 8) / math.cos(math.radians(34))
            self.BB.get("logger").info(f"RIGHT RAY DISTANCE MIN: {right_ray_distance_min}")
            
            if direction == 0:
                chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1] + 1)
            elif direction == 90:
                chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1] + 1)
            elif direction == 180:
                chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1] - 1)
            else:
                chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1] - 1)

            self.BB.get("logger").info(f"RIGHT RAY DISTANCE: {right_ray_distance}")
            if(right_ray_distance > right_ray_distance_min):
                map[chosen] = "free"
            else:
                map[chosen] = "wall"

            #Central ray
            # bisogna leggere il VALUE dall'ir intesity di front_center_left (3 gradi sx)

            central_ray_distance_min = ((5 * self.BB.get("cell_length")) / 8) / math.cos(math.radians(3))
            self.BB.get("logger").info(f"CENTRAL RAY DISTANCE MIN: {central_ray_distance_min}")

            if direction == 0:
                chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1])
            elif direction == 90:
                chosen = (self.BB.get("current_position")[0], self.BB.get("current_position")[1] + 1)
            elif direction == 180:
                chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1])
            else:
                chosen = (self.BB.get("current_position")[0], self.BB.get("current_position")[1] - 1)
            
            self.BB.get("logger").info(f"CENTRAL RAY DISTANCE: {central_ray_distance}")
            if(central_ray_distance > central_ray_distance_min):
                map[chosen] = "free"
            else:
                map[chosen] = "wall"
            
            #Last ray LEFT
            # bisogna leggere il VALUE dall'ir intesity di left (38 gradi sx)

            left_ray_distance_min = ((5 * self.BB.get("cell_length")) / 8) / math.cos(math.radians(38))
            self.BB.get("logger").info(f"LEFT RAY DISTANCE MIN: {left_ray_distance_min}")

            if direction == 0:
                chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1] - 1)
            elif direction == 90:
                chosen = (self.BB.get("current_position")[0] + 1, self.BB.get("current_position")[1] + 1)
            elif direction == 180:
                chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1] + 1)
            else:
                chosen = (self.BB.get("current_position")[0] - 1, self.BB.get("current_position")[1] - 1)

            self.BB.get("logger").info(f"LEFT RAY DISTANCE: {left_ray_distance}")
            if(left_ray_distance > left_ray_distance_min):
                map[chosen] = "free"
            else:
                map[chosen] = "wall"
            
            self.print_map()
            
            self.BB.get("logger").info("IR Map ended!")

            return py_trees.common.Status.SUCCESS
                

            # sleep(0.200)

    
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
