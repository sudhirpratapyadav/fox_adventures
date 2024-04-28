import random
import numpy as np
import os

import rclpy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

class Element:
    # Define TYPES as a class variable
    TYPES = {'Agent': 0, 'Obstacle': 1, 'Lake': 2, 'Food': 3}

    __TYPE_NAMES = {0: 'Agent', 1: 'Obstacle', 2: 'Lake', 3: 'Food'}

    def __init__(self, type, render_color, name="Element", position=None):
        if type not in self.TYPES.values():
            raise ValueError("Invalid element type.")

        self.type = type
        self.position = position
        self.name = name
        self.render_color = render_color
    
    def __str__(self):
        return f"Element: {self.__TYPE_NAMES[self.type]}"

# create class inheriting from Element
class Food(Element):
    def __init__(self, name, position=None):
        # Use Element.TYPES to access the TYPES dictionary
        super().__init__(type=Element.TYPES['Food'], render_color=(0, 255, 0), name=name, position=position)

class Animal(Element):
    def __init__(self, name, MAX_HUNGER, MAX_THIRST, render_color=(0,0,0), position=None):
        super().__init__(type=Element.TYPES['Agent'], render_color=render_color, name=name, position=position)
        self.MAX_HUNGER = MAX_HUNGER
        self.MAX_THIRST = MAX_THIRST
        self.MAX_VELOCITY = 1

        self.velocity = np.zeros(2)
        self.hunger = 0
        self.thirst = 0
        self.obs_size = 5

        self.map_size = 20
        self.map = np.zeros((20, 20), dtype=np.uint8)

    def eat(self, plant):
        print(f"{self.name} eats {plant.name}.")
        self.hunger = 0

    def drink(self):
        print(f"{self.name} drinks water.")
        self.thirst = 0

    def update_map(self, obs):
        x_start = max(0, self.position[0] - self.obs_size)
        y_start = max(0, self.position[1] - self.obs_size)
        x_end = min(self.map_size, self.position[0]  + self.obs_size)
        y_end = min(self.map_size, self.position[1] + self.obs_size)

        self.map[x_start:x_end, y_start:y_end] = obs[max(0, -x_start):min(self.obs_size,100 - x_start),max(0, -y_start):min(self.obs_size, 100 - y_start)]
             
class Lake(Element):
    def __init__(self, name, position=None):
        super().__init__(type=Element.TYPES['Lake'], render_color=(0, 0, 255), name=name, position=position)
        self.water_level = 5
        self.MAX_WATER_LEVEL = 10


class Fox(Animal):
    def __init__(self, position=None):
        super().__init__(name="Fox", MAX_HUNGER=10, MAX_THIRST=10, render_color=(255, 0, 0), position=position)


class Deer(Animal):
    def __init__(self, name="Deer", position=None):
        super().__init__(name=name, MAX_HUNGER=10, MAX_THIRST=10, render_color=(255, 255, 0), position=position)

    def run(self):
        print(f"{self.name} runs away from danger.")


class Environment:
    def __init__(self):

        self.size = 40
        # Create a 2D grid dtype=object
        self.grid = np.empty((self.size, self.size), dtype=object)
        self.grid.fill(None)
        self.main_agent = None
        self.agents = []
        self.obstacles = [] # Trees, Rocks, etc.
        self.lakes = []
        self.foods = []

        self.temp = 25
        self.rain = 0

        self.time = 0 # Time of day
        self.date = 0 # Day of the year

        self.load_env_map()

        # Initialize ROS node
        rclpy.init()
        self.node = rclpy.create_node('environment_node')

        self.markers = {'agent': [], 'food': [], 'obstacle': []}
        self.init_markers()
        
        self.occupancy_grid_publisher = self.node.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.visual_marker_publisher = self.node.create_publisher(MarkerArray, '/visual_markers', 10)
        
        self.obstacle_pos_pub = self.node.create_publisher(Float32MultiArray, '/obstacle_positions', 10)
        self.food_pos_pub = self.node.create_publisher(Float32MultiArray, '/food_positions', 10)

    def load_env_map(self):
        file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "env_map.txt")
        type_counts = {'Fox': 0, 'Deer': 0, 'Lake': 0, 'Tree': 0, 'Food': 0}
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
                        agent = Fox(position=(i, j))
                        self.add_element(agent, (i, j))
                    elif char == 'D':  # Deer
                        type_counts['Deer'] += 1
                        deer = Deer(name=f"Deer_{type_counts['Deer']}", position=(i, j))
                        self.add_element(deer, (i, j))
                    elif char == 'L':  # Lake
                        type_counts['Lake'] += 1
                        lake = Lake(name=f"Lake_{type_counts['Lake']}", position=(i, j))
                        self.add_element(lake, (i, j))
                    elif char == 'F':  # Food
                        type_counts['Food'] += 1
                        food = Food(name=f"Food_{type_counts['Food']}", position=(i, j))
                        self.add_element(food, (i, j))
                    elif char == 'T':  # Tree (Obstacle)
                        type_counts['Tree'] += 1
                        tree = Element(type=Element.TYPES['Obstacle'], render_color=(0, 0, 0), name=f"Tree_{type_counts['Tree']}", position=(i, j))
                        self.add_element(tree, (i, j))
                    else:
                        print(f"Ignoring invalid character '{char}' at position ({i}, {j}).")

    def init_markers(self):
        # Create markers for agents
        m_id = 0
        sx = 1.0
        sy = 1.0
        sz = 1.0
        for agent in self.agents:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = m_id
            # marker.type = Marker.CUBE
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(agent.position[0])+sx/2
            marker.pose.position.y = float(agent.position[1])+sy/2
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

        # Update occupied cells based on obstacle positions
        for obstacle in self.obstacles:
            x, y = obstacle.position
            index = y * self.size + x  # Convert 2D index to 1D index
            occupancy_values[index] = 100  # Set obstacle cells as occupied

            # Set occupancy data
            grid_msg.data = occupancy_values

        # Publish the grid message
        self.occupancy_grid_publisher.publish(grid_msg)
    
    def publish_obstacle_positions(self):
        obstacle_positions = []
        for obstacle in self.obstacles:
            obstacle_positions.append([obstacle.position[0]-(self.size//2-1), 1, obstacle.position[1]-(self.size//2-1)])
        obstacle_positions = np.array(obstacle_positions, dtype=np.float32).flatten()
        msg = Float32MultiArray()
        msg.data = obstacle_positions.tolist()
        self.obstacle_pos_pub.publish(msg)
    
    def publish_food_positions(self):
        food_positions = []
        for food in self.foods:
            food_positions.append([food.position[0]-(self.size//2-1), 1, food.position[1]-(self.size//2-1)])
        food_positions = np.array(food_positions, dtype=np.float32).flatten()
        msg = Float32MultiArray()
        msg.data = food_positions.tolist()
        self.food_pos_pub.publish(msg)
    
    def publish_visual_markers(self):
        time_stamp = self.node.get_clock().now().to_msg()
        for agent, marker in zip(self.agents, self.markers['agent']):
            marker.header.stamp = time_stamp
            marker.pose.position.x = float(agent.position[0])
            marker.pose.position.y = float(agent.position[1])
        
        for food, marker in zip(self.foods, self.markers['food']):
            marker.header.stamp = time_stamp
            marker.pose.position.x = float(food.position[0])
            marker.pose.position.y = float(food.position[1])

        self.visual_marker_publisher.publish(MarkerArray(markers=self.markers['agent'] + self.markers['food'] + self.markers['obstacle']))
    
    def update(self):
        # For isualization in RVIZ
        self.publish_occupancy_grid()
        self.publish_visual_markers() 

        # For Unity simulation
        self.publish_obstacle_positions()
        self.publish_food_positions()
        
    
    def run(self):

        FPS = 2.0 #2 Hz
        self.update_timer = self.node.create_timer(1/FPS, self.update)

        rclpy.spin(self.node)
        rclpy.shutdown()

    def add_element(self, element, position):
        x= position[0]
        y = position[1]
        print(f"Adding {element} at position ({x}, {y}).")
        if 0 <= x < self.size and 0 <= y < self.size and self.grid[x][y] is None:
            self.grid[x][y] = element
            element.position = (x, y)
            if element.type == Element.TYPES['Agent']:
                self.agents.append(element)
                if element.name == "Fox":
                    self.main_agent = element
            elif element.type == Element.TYPES['Obstacle']:
                self.obstacles.append(element)
            elif element.type == Element.TYPES['Lake']:
                self.lakes.append(element)
            elif element.type == Element.TYPES['Food']:
                self.foods.append(element)
            return True
        print("Invalid position.")
        return False

    def remove_element(self, element):
        x, y = element.position
        self.grid[x][y] = None

        if element.type == Element.TYPES['Agent']:
            self.agents.remove(element)
        elif element.type == Element.TYPES['Obstacle']:
            self.obstacles.remove(element)
        elif element.type == Element.TYPES['Lake']:
            self.lakes.remove(element)
        elif element.type == Element.TYPES['Food']:
            self.foods.remove(element)
        else:
            print("Element not found.")
            return False
        

    def random_position(self):
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
        print(f"Rain: {self.rain}")
        print(f"Time: {self.time}")
        print(f"Date: {self.date}")
        print("----------------------")

        

def main():
    size = 10

    env = Environment()
    env.info()
    env.run()
        

if __name__ == "__main__":
    main()  