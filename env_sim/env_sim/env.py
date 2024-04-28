import colorsys
import random
import numpy as np
import os


import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


from geometry_msgs.msg import Twist
from heapq import heappop, heappush
from std_msgs.msg import ColorRGBA


### Path Finding
def find_path(grid, start, goal):
    """
    A* algorithm to find the shortest path on a 2D grid with obstacles.
    """


    def heuristic(position, goal):
        """
        Calculate the Manhattan distance heuristic between two points.
        """
        return abs(position[0] - goal[0]) + abs(position[1] - goal[1])


    rows, cols = grid.shape
    open_set = [(0, start)]  # Priority queue with initial cost and position
    came_from = {}  # Dictionary to store parent-child relationships
    g_score = {start: 0}  # Cost from start to each position


    while open_set:
        current_cost, current_pos = heappop(open_set)


        if current_pos == goal:
            # Reconstruct path
            path = []
            while current_pos in came_from:
                path.append(current_pos)
                current_pos = came_from[current_pos]
            path.append(start)
            path.reverse()
            return path


        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            new_pos = (current_pos[0] + dx, current_pos[1] + dy)
            if 0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols and grid[new_pos[0]][new_pos[1]] != 1:
                tentative_g_score = g_score[current_pos] + 1  # Assuming uniform cost


                if new_pos not in g_score or tentative_g_score < g_score[new_pos]:
                    g_score[new_pos] = tentative_g_score
                    f_score = tentative_g_score + heuristic(new_pos, goal)
                    heappush(open_set, (f_score, new_pos))
                    came_from[new_pos] = current_pos
        
    return None  # No path found

class Element:
    # Define TYPES as a class variable
    TYPES = {'Agent': 0, 'Obstacle': 1, 'Lake': 2, 'Food': 3, 'Fire': 9}


    __TYPE_NAMES = {0: 'Agent', 1: 'Obstacle', 2: 'Lake', 3: 'Food', 9: 'Fire'}


    def __init__(self, type, render_color, name="Element", position=None):
        if type not in self.TYPES.values():
            raise ValueError("Invalid element type.")


        self.type = type
        self.position = position
        self.name = name
        self.render_color = render_color

    def __str__(self):
        return f"Element: {self.__TYPE_NAMES[self.type]}"



class Food(Element):
    def __init__(self, name, position=None):
        # Use Element.TYPES to access the TYPES dictionary
        super().__init__(type=Element.TYPES['Food'], render_color=(0, 255, 0), name=name, position=position)

class Fire(Element):
    def __init__(self, name, position=None):
        # Use Element.TYPES to access the TYPES dictionary
        super().__init__(type=Element.TYPES['Fire'], render_color=(255, 165, 0), name=name, position=position)

        self.MAX_TIME = 10
        self.extinguished = False
        self.time = 0
        self.intensity = 1.0

    def update(self):
        self.time += 1
        self.intensity = 1.0 - self.time/(self.MAX_TIME+1)
        
        hsv = (30/360, self.intensity, 1.0)
        self.render_color = colorsys.hsv_to_rgb(*hsv)
        

        if self.time > self.MAX_TIME:
            self.extinguished = True


class Animal(Element):
    def __init__(self, name, MAX_HUNGER, MAX_THIRST, render_color=(0,0,0), position=None, mode="autonomous"):
        super().__init__(type=Element.TYPES['Agent'], render_color=render_color, name=name, position=position)
        self.MAX_HUNGER = MAX_HUNGER
        self.MAX_THIRST = MAX_THIRST

        self.mode = mode

        self.FOOD_SEARCH_THRESHOLD = 30
        self.WATER_SEARCH_THRESHOLD = 20


        self.MAX_VELOCITY = 1


        self.hunger = 0
        self.thirst = 0


        self.map_size = 20
        self.map = np.full((self.map_size, self.map_size), -1, dtype=int)


        self.dead = False
        self.goal_position = None
        self.current_behaviour = None
        self.goal_reached = False
        self.path = None
        self.next_step = None
      
  
    def update(self):
        if self.hunger > self.MAX_HUNGER or self.thirst > self.MAX_THIRST:
            self.dead = True
        
        self.hunger += 1
        self.thirst += 1

    def drink(self):
        self.thirst = 0

    def eat(self):
        self.hunger = 0
          


  
           
class Lake(Element):
    def __init__(self, name, position=None):
        super().__init__(type=Element.TYPES['Lake'], render_color=(0, 0, 255), name=name, position=position)
        self.water_level = 10
        self.MAX_WATER_LEVEL = 10

    def update_water_level(self, del_water):
        self.water_level = self.water_level + del_water
        if self.water_level < 0:
            self.water_level = 0
        elif self.water_level > self.MAX_WATER_LEVEL:
            self.water_level = self.MAX_WATER_LEVEL
        c = int(255*(self.MAX_WATER_LEVEL-self.water_level)/self.MAX_WATER_LEVEL)
        self.render_color = (c, c, 255)
        print(f"{self.name} water level: {self.water_level}")

    def can_drink(self):
        return self.water_level > 0
      




class Fox(Animal):
    def __init__(self, position=None, mode="autonomous"):
        if mode == "user_mode":
            super().__init__(name="Fox", MAX_HUNGER=70, MAX_THIRST=100, render_color=(255, 0, 0), position=position, mode=mode)
            self.FOOD_SEARCH_THRESHOLD = 25
            self.WATER_SEARCH_THRESHOLD = 40
        else:
            super().__init__(name="Fox", MAX_HUNGER=80, MAX_THIRST=200, render_color=(255, 0, 0), position=position, mode=mode)
            self.WATER_SEARCH_THRESHOLD = 100
        self.obs_size = 3
        self.recent_lions = []

    def update_map(self, GRID):
        x_start = max(0, self.position[0] - self.obs_size)
        y_start = max(0, self.position[1] - self.obs_size)
        x_end = min(20, self.position[0] + self.obs_size)
        y_end = min(20, self.position[1] + self.obs_size)


        self.recent_lions = []
        for i in range(x_start, x_end):
            for j in range(y_start, y_end):
                if GRID[i][j] is None:
                    self.map[i][j] = 0
                else:
                    if GRID[i][j].type == Element.TYPES['Obstacle'] or GRID[i][j].type == Element.TYPES['Fire']:
                        self.map[i][j] = 1
                    elif GRID[i][j].type == Element.TYPES['Lake']:
                        self.map[i][j] = 2
                    elif GRID[i][j].type == Element.TYPES['Food']:
                        self.map[i][j] = 3
                    elif 'Lion' in GRID[i][j].name:
                        self.map[i][j] = 4
                        self.recent_lions.append(GRID[i][j])
                    else:
                        self.map[i][j] = 0


        if self.goal_position is not None:
            # Find Path to goal
            # print(self.goal_position)
            obstacle_grid = np.copy(self.map)
            obstacle_grid[obstacle_grid == 2] = 1 # make lake as obstacle
            obstacle_grid[obstacle_grid > 1] = 0
            self.path = find_path(obstacle_grid, self.position, self.goal_position)
            if self.path is not None:
                if len(self.path) > 1:
                    self.next_step = self.path[1]
                else:
                    print("Reached Goal.")
                    self.next_step = None
                    self.goal_position = None
                    self.current_behaviour = None
            else:
                self.next_step = None
                self.goal_position = None
                self.current_behaviour = None    

    def set_goal(self):
        # if self.goal_position is not None:
        #     if self.map[self.goal_position[0]][self.goal_position[1]] == 0:
        #         self.current_behaviour = None
        
        # Explore
        if self.current_behaviour is None:
            unexplored_cells = np.argwhere(self.map == -1)
            if len(unexplored_cells) > 0:
                self.goal_position = tuple(unexplored_cells[random.randint(0, len(unexplored_cells) - 1)])
            else:
                free_cells = np.argwhere(self.map == 0)
                self.goal_position = tuple(free_cells[random.randint(0, len(free_cells) - 1)])
            self.current_behaviour = 'Exploration'


        # Go over map and find nearest food
        if self.hunger > self.FOOD_SEARCH_THRESHOLD:
            food_positions = np.argwhere(self.map == 3)
            if len(food_positions) > 0:
                distances = np.linalg.norm(food_positions - np.array(self.position), axis=1)
                nearest_food = food_positions[np.argmin(distances)]
                self.goal_position = tuple(nearest_food)
                self.current_behaviour = 'Food'
        
        # Go over map and find nearest water
        if self.thirst > self.WATER_SEARCH_THRESHOLD:
            water_positions = np.argwhere(self.map == 2)
            print("Thirsty, Water Positions: ", water_positions)
            if len(water_positions) > 0:
                distances = np.linalg.norm(water_positions - np.array(self.position), axis=1)
                nearest_water = water_positions[np.argmin(distances)]


                # Find free position near Lake
                nearest_water_free_pose = None
                min_dist = 100
                for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                    new_pos = (nearest_water[0] + dx, nearest_water[1] + dy)
                    if 0 <= new_pos[0] < self.map_size and 0 <= new_pos[1] < self.map_size and self.map[new_pos[0]][new_pos[1]] == 0:
                        dist = np.linalg.norm(np.array(self.position) - np.array(new_pos))
                        if dist < min_dist:
                            min_dist = dist
                            nearest_water_free_pose = new_pos


                if nearest_water_free_pose is not None:
                    self.goal_position = tuple(nearest_water_free_pose)
                    self.current_behaviour = 'Water'

    def update(self):
        super().update()


        print(f"{self.name} Hunger: {self.hunger}, Thirst: {self.thirst}")


        # priority of goals
        # 1. Escape Lion
        # 2. Water
        # 3. Food
        # 4. Unexplored
        # 5. Random


        # self.goal_position = (14,15)
        # self.current_behaviour = 'Exploration'

        if self.mode == "user_mode":
            self.set_goal()
        else:
            if len(self.recent_lions)>0:
                # Find vector sum of repel force from each lion (-ve of direction, maginute inversely proportional to distance)
                repel_force = np.zeros(2, dtype=np.float32)
                pos = np.array(self.position, dtype=np.float32)
                for lion in self.recent_lions:
                    lion_pos = np.array(lion.position, dtype=np.float32)
                    dir = pos - lion_pos
                    dist = np.linalg.norm(dir)
                    dir = dir/dist
                    repel_force += dir/dist # inverse of distance
                repel_force = 10*repel_force


                next_pos = np.array(self.position) + repel_force
                goal_pos = next_pos.astype(int)
                # check dimensions
                if goal_pos[0] <0:
                    goal_pos[0] = 0
                elif goal_pos[0]>=self.map_size:
                    goal_pos[0] = self.map_size-1
                if goal_pos[1] <0:
                    goal_pos[1] = 0
                elif goal_pos[1]>=self.map_size:
                    goal_pos[1] = self.map_size-1
                self.goal_position = tuple(goal_pos)
                self.current_behaviour = 'Escape'
                
                self.recent_lions = []
            else:
                self.set_goal()
                
            
        print(f"{self.name}, CurrentPos:{self.position}, Goal: {self.goal_position}, Behaviour: {self.current_behaviour}")


class Lion(Animal):
    def __init__(self, name="Lion", position=None, mode="autonomous"):
        super().__init__(name=name, MAX_HUNGER=80, MAX_THIRST=60, render_color=(255, 255, 0), position=position, mode=mode)
        self.obs_size = 6


    def update_map(self, GRID):
        x_start = max(0, self.position[0] - self.obs_size)
        y_start = max(0, self.position[1] - self.obs_size)
        x_end = min(self.map_size, self.position[0] + self.obs_size)
        y_end = min(self.map_size, self.position[1] + self.obs_size)


        for i in range(x_start, x_end):
            for j in range(y_start, y_end):
                if GRID[i][j] is None:
                    self.map[i][j] = 0
                else:
                    if 'Fox' in GRID[i][j].name:
                        self.map[i][j] = 3
                    elif GRID[i][j].type == Element.TYPES['Obstacle'] or GRID[i][j].type == Element.TYPES['Food'] or GRID[i][j].type == Element.TYPES['Agent'] or GRID[i][j].type == Element.TYPES['Fire']:
                        self.map[i][j] = 1
                    elif GRID[i][j].type == Element.TYPES['Lake']:
                        self.map[i][j] = 2
                    else:
                        self.map[i][j] = 0
        


        if self.goal_position is not None:
            # Find Path to goal
            # print(self.goal_position)
            obstacle_grid = np.copy(self.map)
            obstacle_grid[obstacle_grid == 2] = 1 # make lake as obstacle
            obstacle_grid[obstacle_grid > 1] = 0
            self.path = find_path(obstacle_grid, self.position, self.goal_position)
            if self.path is not None:
                if len(self.path) > 1:
                    self.next_step = self.path[1]
                else:
                    print("Reached Goal.")
                    self.next_step = None
                    self.goal_position = None
                    self.current_behaviour = None
            else:
                self.next_step = None
                self.goal_position = None
                self.current_behaviour = None

    def update(self):
        super().update()

        print(f"{self.name} Hunger: {self.hunger}, Thirst: {self.thirst}")

        if self.mode == "user_mode":
            pass
        else:

            # priority of goals
            # 1. Water
            # 2. Food
            # 3. Unexplored
            # 4. Random


            # self.goal_position = (14,15)
            # self.current_behaviour = 'Exploration'


            if self.current_behaviour is None:
                unexplored_cells = np.argwhere(self.map == -1)
                if len(unexplored_cells) > 0:
                    self.goal_position = tuple(unexplored_cells[random.randint(0, len(unexplored_cells) - 1)])
                else:
                    free_cells = np.argwhere(self.map == 0)
                    self.goal_position = tuple(free_cells[random.randint(0, len(free_cells) - 1)])
                self.current_behaviour = 'Exploration'


            # Go over map and find nearest food
            if self.hunger > self.FOOD_SEARCH_THRESHOLD:
                food_positions = np.argwhere(self.map == 3)
                if len(food_positions) > 0:
                    distances = np.linalg.norm(food_positions - np.array(self.position), axis=1)
                    nearest_food = food_positions[np.argmin(distances)]
                    self.goal_position = tuple(nearest_food)
                    self.current_behaviour = 'Food'
            
            # Go over map and find nearest water
            if self.thirst > self.WATER_SEARCH_THRESHOLD:
                water_positions = np.argwhere(self.map == 2)
                print("Thirsty, Water Positions: ", water_positions)
                if len(water_positions) > 0:
                    distances = np.linalg.norm(water_positions - np.array(self.position), axis=1)
                    nearest_water = water_positions[np.argmin(distances)]


                    # Find free position near Lake
                    nearest_water_free_pose = None
                    min_dist = 100
                    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                        new_pos = (nearest_water[0] + dx, nearest_water[1] + dy)
                        if 0 <= new_pos[0] < self.map_size and 0 <= new_pos[1] < self.map_size and self.map[new_pos[0]][new_pos[1]] == 0:
                            dist = np.linalg.norm(np.array(self.position) - np.array(new_pos))
                            if dist < min_dist:
                                min_dist = dist
                                nearest_water_free_pose = new_pos


                    if nearest_water_free_pose is not None:
                        self.goal_position = tuple(nearest_water_free_pose)
                        self.current_behaviour = 'Water'
            
        print(f"{self.name}, CurrentPos:{self.position}, Goal: {self.goal_position}, Behaviour: {self.current_behaviour}")




class Environment:
    def __init__(self, mode):

        self.mode = mode

        self.size = 20
        # Create a 2D grid dtype=object
        self.grid = np.empty((self.size, self.size), dtype=object)
        self.grid.fill(None)
        self.agents = {'Fox':None, 'Lion':[]}
        self.obstacles = [] # Trees, Rocks, etc.
        self.lakes = []
        self.foods = []
        self.fires = []

        self.MAX_FOOD_LIMIT = 20
        self.MAX_FIRE_LIMIT = 10


        self.temp = 25
        self.rain_prob = 0.01

        self.load_env_map()


        # Initialize ROS node
        rclpy.init()
        self.node = rclpy.create_node('environment_node')

        self.EXIT = False
        self.FPS = 5 #2 Hz


        self.create_markers()

        # Rviz Pubs
        self.occupancy_grid_publisher = self.node.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.visual_marker_publisher = self.node.create_publisher(MarkerArray, '/visual_markers', 10)
        self.agent_path_pubs = []
        for i in range(len(self.agents['Lion'])):
            self.agent_path_pubs.append(self.node.create_publisher(Path, f"/lion_path_{i}", 10))
        self.fox_path_publisher = self.node.create_publisher(Path, '/fox_path', 10)

        # Unity Pubs
        self.obstacle_pos_pub = self.node.create_publisher(Float32MultiArray, '/obstacle_positions', 10)
        self.food_pos_pub = self.node.create_publisher(Float32MultiArray, '/food_positions', 10)
        self.agent_pos_pub = self.node.create_publisher(Float32MultiArray, '/agent_positions', 10)
        self.lake_pos_pub = self.node.create_publisher(Float32MultiArray, '/lake_positions', 10)
        self.fire_pos_pub = self.node.create_publisher(Float32MultiArray, '/fire_positions', 10)

        # User Control
        # self.cmd_vel_subscriber = self.node.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.goal_subscriber = self.node.create_subscription(PointStamped, '/clicked_point', self.click_point_callback, 10)

    def click_point_callback(self, msg):
        if self.mode == "user_mode":
            x = int(msg.point.x)
            y = int(msg.point.y)
            self.agents['Fox'].goal_position = (x, y)
            self.agents['Fox'].current_behaviour = "Exploration"
            print(f"Fox Goal Set to: {x}, {y}")

    def load_env_map(self):
        file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "env_map.txt")
        type_counts = {'Fox': 0, 'Lion': 0, 'Lake': 0, 'Tree': 0, 'Food': 0}
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for i, line in enumerate(lines):
                if i >= self.size:
                    break  # Ensure we only read up to the size of the grid
                line = line.strip()[:self.size]  # Trim the line to match grid size
                for j, char in enumerate(line):
                    if char == '.':  # Free space
                        continue
                    elif char == 'A':  # Fox
                        type_counts['Fox'] += 1
                        agent = Fox(position=(i, j), mode=self.mode)
                        self.add_element(agent, (i, j))
                    elif char == 'L':  # Lion
                        type_counts['Lion'] += 1
                        lion = Lion(name=f"Lion_{type_counts['Lion']}", position=(i, j), mode=self.mode)
                        self.add_element(lion, (i, j))
                    elif char == 'W':  # Lake
                        type_counts['Lake'] += 1
                        lake = Lake(name=f"Lake_{type_counts['Lake']}", position=(i, j))
                        self.add_element(lake, (i, j))
                    elif char == 'F':  # Food
                        if type_counts['Food'] >= self.MAX_FOOD_LIMIT:
                            continue
                        type_counts['Food'] += 1
                        food = Food(name=f"Food_{type_counts['Food']}", position=(i, j))
                        self.add_element(food, (i, j))
                    elif char == 'T':  # Tree (Obstacle)
                        type_counts['Tree'] += 1
                        tree = Element(type=Element.TYPES['Obstacle'], render_color=(0, 0, 0), name=f"Tree_{type_counts['Tree']}", position=(i, j))
                        self.add_element(tree, (i, j))
                    else:
                        print(f"Ignoring invalid character '{char}' at position ({i}, {j}).")

    def get_agent_lst(self):
        agent_lst = []
        for key, value in self.agents.items():
            if key=="Fox" and value is not None:
                agent_lst.append(value)
            elif key=="Lion" and len(value)>0:
                agent_lst.extend(value)
        
        return agent_lst


    # def cmd_vel_callback(self, msg):
    #     dx = np.sign(msg.linear.x)
    #     dy = np.sign(msg.linear.y)


    #     print(f"dx: {dx}, dy: {dy}")
        
    #     self.agents['Fox'].position = (self.agents['Fox'].position[0] + dx, self.agents['Fox'].position[1] + dy)


    def create_markers(self):
        # Create markers for agents
        m_id = 0
        sx = 1.0
        sy = 1.0
        sz = 1.0


        self.markers = {'agent': [], 'food': [], 'obstacle': [], 'lake': [], 'fire': []}
        
        agents_list = self.get_agent_lst()
        for agent in agents_list:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = m_id
            # marker.type = Marker.CUBE
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(agent.position[0])
            marker.pose.position.y = float(agent.position[1])
            marker.pose.position.z = sz/2  # Assuming 2D environment
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = sx  # Adjust size as needed
            marker.scale.y = sy
            marker.scale.z = sz
            marker.color.r = agent.render_color[0] / 255.0
            marker.color.g = agent.render_color[1] / 255.0
            marker.color.b = agent.render_color[2] / 255.0
            marker.color.a = 1.0  # Fully opaque
            self.markers['agent'].append(marker)
            m_id += 1


        sx = 1.0
        sy = 1.0
        sz = 1.0
        for food in self.foods:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = m_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(food.position[0])+sx/2
            marker.pose.position.y = float(food.position[1])+sy/2
            marker.pose.position.z = sz/2  # Assuming 2D environment
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = sx  # Adjust size as needed
            marker.scale.y = sy
            marker.scale.z = sz
            marker.color.r = food.render_color[0] / 255.0
            marker.color.g = food.render_color[1] / 255.0
            marker.color.b = food.render_color[2] / 255.0
            marker.color.a = 1.0  # Fully opaque
            self.markers['food'].append(marker)
            m_id += 1

        for lake in self.lakes:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = m_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(lake.position[0])+sx/2
            marker.pose.position.y = float(lake.position[1])+sy/2
            marker.pose.position.z = sz/2  # Assuming 2D environment
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = sx  # Adjust size as needed
            marker.scale.y = sy
            marker.scale.z = sz
            marker.color.r = lake.render_color[0] / 255.0
            marker.color.g = lake.render_color[1] / 255.0
            marker.color.b = lake.render_color[2] / 255.0
            marker.color.a = 1.0  # Fully opaque
            self.markers['lake'].append(marker)
            m_id += 1
        
        sx = 1.0
        sy = 1.0
        sz = 4.0
        for obstacle in self.obstacles:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = m_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(obstacle.position[0])+sx/2
            marker.pose.position.y = float(obstacle.position[1])+sy/2
            marker.pose.position.z = sz/2
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = sx  # Adjust size as needed
            marker.scale.y = sy
            marker.scale.z = sz
            marker.color.r = obstacle.render_color[0] / 255.0
            marker.color.g = obstacle.render_color[1] / 255.0
            marker.color.b = obstacle.render_color[2] / 255.0
            marker.color.a = 1.0
            self.markers['obstacle'].append(marker)
            m_id += 1
        

        sx = 1.0
        sy = 1.0
        sz = 1.0
        for fire in self.fires:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = m_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(fire.position[0])+sx/2
            marker.pose.position.y = float(fire.position[1])+sy/2
            marker.pose.position.z = sz/2  # Assuming 2D environment
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = sx  # Adjust size as needed
            marker.scale.y = sy
            marker.scale.z = sz
            marker.color.r = fire.render_color[0] 
            marker.color.g = fire.render_color[1]
            marker.color.b = fire.render_color[2]
            marker.color.a = 1.0  # Fully opaque
            self.markers['fire'].append(marker)
            m_id += 1

    def publish_occupancy_grid(self):
        # Create an OccupancyGrid message
        grid_msg = OccupancyGrid()


        # Populate header
        grid_msg.header.stamp = self.node.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"


        # Populate metadata
        grid_msg.info.map_load_time = grid_msg.header.stamp
        grid_msg.info.resolution = 1.0  # Each cell is 1 meter
        grid_msg.info.width = self.size
        grid_msg.info.height = self.size


        # Initialize all cells as free space (-1)
        occupancy_values = [-1] * (self.size * self.size)
        
        fox = self.agents['Fox']
        if fox is not None:
            fox_x, fox_y = fox.position
            # iterate over observation size of fox grid


            x_start = max(0, fox_x - fox.obs_size)
            y_start = max(0, fox_y - fox.obs_size)
            x_end = min(20, fox_x + fox.obs_size)
            y_end = min(20, fox_y + fox.obs_size)


            for i in range(x_start, x_end):
                for j in range(y_start, y_end):
                    index = j * self.size + i
                    occupancy_values[index] = 40
        
        grid_msg.data = occupancy_values


        # Publish the grid message
        self.occupancy_grid_publisher.publish(grid_msg)



    def print_grid(self):
        for r in range(self.size):
            for c in range(self.size):
                if self.grid[r][c] is not None:
                    if self.grid[r][c].name == 'Fox':
                        print("A", end='')
                    else:
                        print(self.grid[r][c].name[0], end='')
                else:
                    print(".", end='')
            print()
        print("\n\n\n")


    def publish_unity_positions(self):

        scale = 3.8
        def pos_to_unity(pos, y=0):
            return scale*(pos[0]-(self.size//2-1)), y, scale*(pos[1]-(self.size//2-1))

        obstacle_positions = []
        for obstacle in self.obstacles:
            obstacle_positions.append(pos_to_unity(obstacle.position))
        obstacle_positions = np.array(obstacle_positions, dtype=np.float32).flatten()
        msg = Float32MultiArray()
        msg.data = obstacle_positions.tolist()
        self.obstacle_pos_pub.publish(msg)

        food_positions = []
        for food in self.foods:
            food_positions.append(pos_to_unity(food.position, y=0.5))
        food_positions = np.array(food_positions, dtype=np.float32).flatten()
        msg = Float32MultiArray()
        msg.data = food_positions.tolist()
        self.food_pos_pub.publish(msg)

        agent_positions = []
        for agent in self.get_agent_lst():
            agent_positions.append(pos_to_unity(agent.position, y=0.1))
        agent_positions = np.array(agent_positions, dtype=np.float32).flatten()
        msg = Float32MultiArray()
        msg.data = agent_positions.tolist()
        self.agent_pos_pub.publish(msg)

        lake_positions = []
        for lake in self.lakes:
            lake_positions.append(pos_to_unity(lake.position, y=2))
        lake_positions = np.array(lake_positions, dtype=np.float32).flatten()
        msg = Float32MultiArray()
        msg.data = lake_positions.tolist()
        self.lake_pos_pub.publish(msg)

        fire_positions = []
        for fire in self.fires:
            fire_positions.append(pos_to_unity(fire.position, y=0.1))
        fire_positions = np.array(fire_positions, dtype=np.float32).flatten()
        msg = Float32MultiArray()
        msg.data = fire_positions.tolist()
        self.fire_pos_pub.publish(msg)

    def publish_visual_markers(self):
        time_stamp = self.node.get_clock().now().to_msg()
        agents_list = self.get_agent_lst()
        for agent, marker in zip(agents_list, self.markers['agent']):
            marker.header.stamp = time_stamp
            marker.pose.position.x = float(agent.position[0])+0.5
            marker.pose.position.y = float(agent.position[1])+0.5
        
        for food, marker in zip(self.foods, self.markers['food']):
            marker.header.stamp = time_stamp
            marker.pose.position.x = float(food.position[0])+0.5
            marker.pose.position.y = float(food.position[1])+0.5


        for lake, marker in zip(self.lakes, self.markers['lake']):
            marker.header.stamp = time_stamp
            marker.pose.position.x = float(lake.position[0])+0.5
            marker.pose.position.y = float(lake.position[1])+0.5
        
        for fire, marker in zip(self.fires, self.markers['fire']):
            marker.header.stamp = time_stamp
            marker.pose.position.x = float(fire.position[0])+0.5
            marker.pose.position.y = float(fire.position[1])+0.5


        self.visual_marker_publisher.publish(MarkerArray(markers=self.markers['agent'] + self.markers['food'] + self.markers['obstacle']+self.markers['lake']+self.markers['fire']))

    def publish_agent_path(self):
        if self.agents['Fox'].path is not None:
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.node.get_clock().now().to_msg()
            for position in self.agents['Fox'].path:
                waypoint = PoseStamped()
                waypoint.pose.position.x = float(position[0])+0.5
                waypoint.pose.position.y = float(position[1])+0.5
                waypoint.pose.position.z = 0.5
                path_msg.poses.append(waypoint)
            self.fox_path_publisher.publish(path_msg)
        
        # Lion Paths
        for i, lion in enumerate(self.agents['Lion']):
            if lion.path is not None:
                path_msg = Path()
                path_msg.header.frame_id = "map"
                path_msg.header.stamp = self.node.get_clock().now().to_msg()
                for position in lion.path:
                    waypoint = PoseStamped()
                    waypoint.pose.position.x = float(position[0])+0.5
                    waypoint.pose.position.y = float(position[1])+0.5
                    waypoint.pose.position.z = 0.5
                    path_msg.poses.append(waypoint)
                self.agent_path_pubs[i].publish(path_msg)

    def update(self):
        print("\n\n-----------------")
        # Update Agents Map
        for agent in self.get_agent_lst():
            agent.update_map(self.grid)


        # For isualization in RVIZ
        self.publish_occupancy_grid()
        self.publish_visual_markers()
        self.publish_agent_path()


        # For Unity simulation
        self.publish_unity_positions()
        


        # self.print_grid()


        # check rain
        if self.rain_prob > random.random():
            print("Rain: True")
            for lake in self.lakes:
                lake.update_water_level(lake.MAX_WATER_LEVEL)
                flag_elements_updated = True
            
            # grow food randomly
            num_food_grow = self.MAX_FOOD_LIMIT-len(self.foods)
            for i in range(num_food_grow):
                pos = self.get_random_position()
                food = Food(name=f"Food_{i}", position=pos)
                self.add_element(food, pos)

            ## remove all fires
            for fire in self.fires:
                self.remove_element(fire)
        else:
            print("Rain: False")
        print("Water Level: ", self.lakes[0].water_level)
        
        if self.mode == "fire_mode":
            # set temperature randomly between 25-50
            self.temp = random.randint(25, 50)
            if self.temp > 30:
                print("Temperature is too high.")
                # Create fire randomly
                num_fires = int(((self.temp-30.0)/20.0)*(self.MAX_FIRE_LIMIT-len(self.fires)))
                for i in range(num_fires):
                    pos = self.get_random_position()
                    fire = Fire(name=f"Fire_{i}", position=pos)
                    self.add_element(fire, pos)
            # update fires
            for fire in self.fires:
                fire.update()
                if fire.extinguished:
                    self.remove_element(fire)
                    # print(f"Fire is extinguished.")



        # Update Lions
        flag_elements_updated = False
        for lion in self.agents['Lion']:
            lion.update()
            if lion.dead:
                self.remove_element(lion)
                flag_elements_updated = True
                print(f"{lion.name} is dead.")
            else:
                # Check for near water
                if lion.current_behaviour == 'Water':
                    for lake in self.lakes:
                        if np.linalg.norm(np.array(lion.position) - np.array(lake.position)) <= 1:
                            if lake.can_drink():
                                lake.update_water_level(-1)
                                lion.drink()
                                flag_elements_updated = True
                            else:
                                print("Lake is dry.")
                
                if lion.next_step is not None:
                    next_element = self.grid[lion.next_step[0]][lion.next_step[1]]
                    if next_element is not None:
                        if 'Fox' in next_element.name and lion.current_behaviour == 'Food':
                            lion.eat()
                            self.remove_element(next_element)
                            flag_elements_updated = True
                            print(f"FOX is eaten by Lion.")
                            self.EXIT = True
                        
                        if next_element.type != Element.TYPES['Obstacle'] and next_element.type != Element.TYPES['Lake'] and (next_element.type != Element.TYPES['Agent'] or 'Fox' in next_element.name):
                            self.grid[lion.position[0]][lion.position[1]] = None
                            lion.position = lion.next_step
                            self.grid[lion.position[0]][lion.position[1]] = lion
                    else:
                        self.grid[lion.position[0]][lion.position[1]] = None
                        lion.position = lion.next_step
                        self.grid[lion.position[0]][lion.position[1]] = lion
        
        if self.agents['Fox'] is not None:
            fox = self.agents['Fox']
            fox.update()
            if fox.dead:
                self.remove_element(fox)
                flag_elements_updated = True
                print(f"FOX is dead.")
                self.EXIT = True
            else:
                # Check for near water
                if fox.current_behaviour == 'Water':
                    for lake in self.lakes:
                        if np.linalg.norm(np.array(fox.position) - np.array(lake.position)) <= 1:
                            if lake.can_drink():
                                lake.update_water_level(-1)
                                fox.drink()
                                flag_elements_updated = True
                            else:
                                print("Lake is dry.")


                if fox.next_step is not None:
                    next_element = self.grid[fox.next_step[0]][fox.next_step[1]]
                    if next_element is not None:
                        if next_element.type == Element.TYPES['Food']:
                            fox.eat()
                            self.remove_element(next_element)
                            flag_elements_updated = True
                        if next_element.type != Element.TYPES['Obstacle'] and next_element.type != Element.TYPES['Lake']:
                            self.grid[fox.position[0]][fox.position[1]] = None
                            fox.position = fox.next_step
                            self.grid[fox.position[0]][fox.position[1]] = fox
                    else:
                        self.grid[fox.position[0]][fox.position[1]] = None
                        fox.position = fox.next_step
                        self.grid[fox.position[0]][fox.position[1]] = fox


        if flag_elements_updated:
            self.create_markers()

    def run(self):
        while rclpy.ok() and not self.EXIT:
            self.update()
            rclpy.spin_once(self.node, timeout_sec=1.0/self.FPS)
        
        self.node.destroy_node()
        rclpy.shutdown()


    def add_element(self, element, position):
        x= position[0]
        y = position[1]
        print(f"Adding {element} at position ({x}, {y}).")
        if 0 <= x < self.size and 0 <= y < self.size and self.grid[x][y] is None:
            self.grid[x][y] = element
            element.position = (x, y)
            if element.type == Element.TYPES['Agent']:
                if element.name == "Fox":
                    self.agents['Fox'] = element
                elif "Lion" in element.name:
                    self.agents['Lion'].append(element)
                else:
                    print("Invalid agent name.")
            elif element.type == Element.TYPES['Obstacle']:
                self.obstacles.append(element)
            elif element.type == Element.TYPES['Lake']:
                self.lakes.append(element)
            elif element.type == Element.TYPES['Food']:
                self.foods.append(element)
            elif element.type == Element.TYPES['Fire']:
                self.fires.append(element)
            return True
        print("Invalid position.")
        return False


    def remove_element(self, element):
        x, y = element.position
        self.grid[x][y] = None


        if element.type == Element.TYPES['Agent']:
            if element.name == "Fox":
                self.agents['Fox'] = None
            elif "Lion" in element.name:
                self.agents['Lion'].remove(element)
            else:
                print("Not removed Invalid agent name.")
        elif element.type == Element.TYPES['Obstacle']:
            self.obstacles.remove(element)
        elif element.type == Element.TYPES['Lake']:
            self.lakes.remove(element)
        elif element.type == Element.TYPES['Food']:
            self.foods.remove(element)
        elif element.type == Element.TYPES['Fire']:
            self.fires.remove(element)
        else:
            print("Element not found.")
            return False
        


    def get_random_position(self):
        # Return a free random position within the grid
        x = random.randint(1, self.size - 1)
        y = random.randint(1, self.size - 1)
        while self.grid[x][y] is not None:
            x = random.randint(1, self.size - 1)
            y = random.randint(1, self.size - 1)
        return (x, y)


    def info(self):
        # Print information about the environment (all possible information)
        print("----------Environment Information------------")
        print(f"Size: {self.size}")
        print(f"Agents: {len(self.agents)}")
        # for agent in self.agents:
        #     print(f"Agent Name: {agent.name}, Position: {agent.position}")
        print(f"Obstacles: {len(self.obstacles)}")
        # for obstacle in self.obstacles:
        #     print(f"Obstacle Name: {obstacle.name}, Position: {obstacle.position}")
        print(f"Lakes: {len(self.lakes)}")
        # for lake in self.lakes:
        #     print(f"Lake Name: {lake.name}, Position: {lake.position}")
        print(f"Food: {len(self.foods)}")
        # for food in self.foods:
        #     print(f"Food Name: {food.name}, Position: {food.position}")
        print(f"Temperature: {self.temp}")
        print(f"Rain: {self.rain_prob}")
        print("----------------------")


def main():
    # mode = 'user_mode'
    # mode = 'autonomous'
    mode = 'fire_mode'
    env = Environment(mode)
    env.info()
    env.run()


if __name__ == "__main__":
    main() 

