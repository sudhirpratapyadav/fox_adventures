# FOX Adventures

#### Details of Project
1. **Name:** **Fox Adventures: Discovering Nature's Balance**
2. **Brief:** In this project, I would design and implement a virtual simulation of a natural ecosystem highlighting the interactions between various animals and their environment. The objective of this project is to create a realistic and dynamic simulation that demonstrates the behaviours of different organisms in response to environmental changes and interactions with each other. Also demonstrate planning and other capabilities to navigate the environment.
3. **Project Description:** 
	1. Virtual model of a natural ecosystem will be developed. It would include few plants, places (grassland, lake, etc.).
	2. It will also include virtual agents as animals (with different behaviours and prey-predator dynamics).
	3. There will be a main animal (Agent FOX) who can interact with environment (like drink water, eat, run from lion etc.). 
	4. The environment will be dynamic and stochastic. Interaction between environment and other animals will make it dynamic. Also environment state will change automatically according to factors such as rain, temperature etc. for example rain should increase water level in lake (which will be stochastic as well as dynamic).
	5. The environment will be inaccessible. Agent Fox will have limited view and need to represent environment in its own way from his observations.
	6. Agent Fox will be autonomous and  need to demonstrate virtual scenarios mentioned below.
4. **Virtual Scenarios:**
	1. Based on user input, agent FOX should be able to navigate to any part of the environment 
		- This requires agent should map the environment, plan the path, explore dead ends, avoid predators, and meeting his requirements like water/food etc in case he is hungry.
		1. There is a random bad event forest fire. The agent FOX should display autonomous behaviour to safely navigate out of this situation.
	
#### **Tools and Libraries:**
- We will use Unity in combination with ROS for this project. Unity will be used mainly for 3D simulation while ROS will be used for behavioural aspects to control the agent and other things.
#### Project Plan:
1. **Static Environment Design:**
    - Design the virtual environment using Unity, incorporating elements such as terrain, vegetation, water bodies, etc.
    - Add animals (static 3D models).
    - Add FOX agent (static 3D model).
2. **Adding Perception and Control via ROS-Unity Setup:**
	- Integrate ROS (Robot Operating System) with Unity to add perception and control in the simulation.
	- **Perception:** Add camera (or other sensor) to FOX agent so that agent can see and detect objects/places in environment.
		- We will assume few imaginary perception sensors which can directly capture state of environment but only up-to where camera can see.
		- So if Lion is visible in camera, fox will know that there is Lion at certain distance from him. Same for lake etc.
	- **Control:** Add control (user control using keyboard) so user can move the FOX and explore the static environment.
		- Maximum speed of FOX will be limited.
3. **Add Dynamic Aspect of Environment:** 
    - Implement dynamic environmental changes such as rain, temperature variations, and natural disasters like floods or forest fires.
    - Create models and behaviours for other virtual animals in the ecosystem, including predators, prey.
	    - Same Perception and control for other animals as given to FOX but with different range and behaviours.
4. **Implement Virtual Scenarios using ROS:**
    - Develop ROS nodes to implement **navigation algorithms** for Agent Fox to explore the environment, including planning and obstacle avoidance.
	    - It should include various behaviours such as running, drinking water, eating, and evading predators.
    - Implement **user input mechanisms:**
	    - for controlling Agent Fox, such as mouse clicks to specify destinations.
    - Implement over-all virtual scenarios outlined in the project description, such as navigating to a specified location based on user input and autonomously escaping from random bad events.
5. **Documentation and Presentation (Although it is ongoing process):**
    - Document the development process, including design decisions, algorithms, and implementation details.
    - Prepare a comprehensive presentation showcasing the virtual ecosystem simulation, its features, and the underlying technology stack.
#### Expected Outcomes:
- A realistic and dynamic virtual simulation of a natural ecosystem, demonstrating interactions between animals and their environment.
- Agent Fox character capable of autonomous behavior, including navigation, resource management, and response to environmental changes.
- Implementation of user-driven navigation and autonomous response to random bad events as specified in the project description.
- Integration of Unity with ROS for controlling behavioral aspects of the simulation.
- Documentation detailing the development process and a presentation highlighting the project's features and achievements.

#### Implementation for Phase-1 
- For initial phase (Phase 1) step-1 and step-2 of the project plan have been implemented (along with this partial report (step-5)).
- **Step-1: Static Environment Design**
	- Following is the screenshot static environment. I have used Open-Source 3D models available online to build this environment in Unity. Following models have been used
		- FOX: https://assetstore.unity.com/packages/3d/characters/animals/toon-fox-183005
		- Trees, rocks and ground: https://assetstore.unity.com/packages/3d/environments/landscapes/low-poly-simple-nature-pack-162153
		- Other Animals: https://assetstore.unity.com/packages/3d/characters/animals/animals-free-260727
		- Lake: https://assetstore.unity.com/packages/vfx/shaders/mobile-depth-water-shader-89541
		- FOX on right side is the main agent
- **Step-2: Adding Perception and Control via ROS-Unity Setup**
	- Check the following video for which demonstrate perception and control of FOX agent via ROS.
		- Link: https://www.youtube.com/watch?v=WbeyV2h6b30
	- Left Screen shows over-all scene, while right window is the camera (for perception) attached to the FOX agent. It moves along with FOX.
	- FOX is controlled via ROS to move around the scene.

#### Implementation for Phase-2
- For final phase (Phase 2) step-3 to step-5 of the project plan have been implemented.
- Scene in Unity-RVIZ Combined
	- ![[Pasted image 20240428225335.png]]
- **Step-3: Dynamic Environment Design**
	- This phase implements an environment simulation system using Python and ROS (Robot Operating System). It defines a class called `Environment` that represents an environment grid where agents interact with various elements such as obstacles, food, and fires. The environment is visualized using ROS visualization markers and can be controlled via user input or run autonomously.
	- **Main Features**
		1. **Grid Environment:** Represents a 20x20 grid environment where agents and elements can move and interact.
		2. **Environment Map Loading**: Reads an environment map from a text file and initializes elements (agents, obstacles, lakes, foods) accordingly based on characters in the map.
		3. **Agent Management:** Manages agents including a Fox and multiple Lions. Agents have behaviors such as exploring, hunting, and drinking.
		4. **Element Types:** Supports different types of elements including obstacles, lakes, food sources, and fires.
		5. **Visualization:** Utilizes ROS visualization markers to display agents and elements in the environment.
		6. **User Interaction:** Allows user input to set goals for the Fox agent.
		7. **Dynamic Environment:** Simulates environmental factors such as rain and temperature, affecting the behavior of agents and elements.
		8. **Unity Integration:** Publishes positions of elements for integration with Unity simulation.
		9. **Path Planning:** Computes and publishes paths for agents to follow.
		10. **Event Handling:** Handles events such as rain, temperature changes, and agent interactions with elements.
	- **Details of various Aspects**
		- Each object/thing on the grid is an element.
			- Each element has a position on the grid.
			- No two elements can occupy same position
		- Type of elements
			- Agent
				- General Attributes and behaviours
					- **Motion:** Each Agent can move in 4 possible directions up-down-left-right
					- **Perception:** Each agent can perceive in a square grid around its position up to some limit (OBS_SIZE)
					- **Has memory:** Map of environment (initially it is empty and is updated dynamically based on observation as agent moves)
					- **Gets hungry and thirsty:** With time agent gets hungry and thirst (different rate). If hunger and thirst go beyond max limits agent will die.
					- **Plan path:** Each agents decides goal and plan the path to achieve its goal.
				- **Fox**
					- This is main character (agent), if this **dies game/simulation is over.** Only one fox is there in env.
					- Fox has default obs_size of 3 units in each direction.
					- Fox needs to **eat food (grass)** in order to quench hunger and needs to drink water.
					- Need to avoid Lion.
					- Behaviour is described in step 4.
				- **Lion**
					- There can be many lions (by default 2 lions)
					- Lion has bigger observation size then fox (5 units).
					- Lion needs to **eat Fox** and drink water.
					- If any one lions eats the Fox fox will die and game is over.
			- Non-Agent
				- Dynamic
					-  **Food:** This is food for fox. This is represented by grass. Its position is **dynamic based on rain**. If Fox eats grass it will disappear. There is **max_limit on number of food elements in environment.**
					- **Lake:** Lake has some water level. When an agent drinks, it's water level decreases and during rain it is filled back.
					- **Fire:** This elements spreads randomly based on temperature. If agents touches fire it will die.
				- Static
					- **Obstacles:** Trees and rocks are static obstacles and no agent can cross them
		- **State update Loop and Conflict resolution**
			- **Perception:** Each agent's map is updated based on their location and observation size.
			- **Rain update:** Based on some probability rain occurs. Lake water level and new food is grown.
			- **Fire Update:** Base on temperature fire occurs at some places.
			- **Agent Updates:** First lion's position is updated (based on its plan). **If lion reaches fox, he eats it and game is over.** Then fox is updates based on its plan. If any agent is gets too hungry or thirsty it will die.
			- **Visualsitation:** Element locations, probable paths and other required things are published to unity as well as RVIZ for visualising the simulation.
- **Step-4: Implement Virtual Scenarios using ROS:**
	- **Decision Making by agents:**
		- Each agents has many behaviours. For decision making we follow **layered architecture** to define hierarchy of behaviours.
		- **FOX:** 
			- Based on current perception and fox state it decide a behaviour in following order.
			- (if Lion in observation) Escape Lion > (If thirsty) Drink Water > (If Hungry) Eat FOOD > Random Exploration
			- **Also sometimes lower behaviour may occur for example, fox is thirsty but it has not yet discovered lake so it will do random exploration.**
			- Note: Fox needs to do random exploration because as environment is dynamic and it it needs to memorise where it saw food/water last time.
		- **LION:** Lion also has similar behaviour except it needs to eat FOX thus its food is fox. And it not need to escape anything.
		- **Fire** is treated as obstacle by each agent.
	- Once a behaviour is decided and a goal is selected it needs to be executed using planning.
		- **Food and Water:** Goal is nearest food and water.
		- **Random:** Select random free-goal (**first preference is given to unexplored regions**).
		- **FOX escape Goal Selection:** If fox sees lion(s) it's goal is decided based on repulsion. Each lion will have some repulsive force on the fox (inversely proportional to distance).  Thus goal will be decided based on over-all repulsion. This is similar to **Potential Field method for path planning**.
	- **Path planning:** Once Goal is decided agent's use **A* algorithm**o find shortest path avoiding obstacles.
		- A* is a pathfinding algorithm that efficiently finds the shortest path between two points on a graph or grid by considering both the cost of reaching a node from the starting point (g) and an optimistic estimate of the remaining cost to reach the goal (h). It explores nodes with the lowest combined cost, making it particularly effective for navigation and route planning in various applications, including robotics and gaming.
		- We use this to plan path to the goal for given agent, while avoiding obstacles.
			- Lake, fire and enemies are treated as obstacles.
			- In case of Lion grass is also treated as obstacles (to make it more difficult for them).
	- **Experimental Results and Analysis**
		- 3 Modes
			- **User Mode:** In this mode, users can interact with the environment by setting goals for the Fox agent through clicking on points in the simulation.
				- **In this mode user can give goal to the fox by clicking on the map in rviz.** Fox will be in autonomous mode until user does not provide goal or goal is completed.
				- This mode does not support Lions (as this mode is for demonstrating path finding and other behaviours)
				- Video: https://www.youtube.com/watch?v=j6bJOL6Hmow
			- **Autonomous Mode:** Autonomous mode enables the agents to navigate and interact within the environment without direct user input. They make decisions based on their internal logic and sensory information.
				- In this mode Lions will try to eat FOX while fox try to save itself and also eat food and drink water.
				- Video (FOX wins): https://www.youtube.com/watch?v=wrA07U8xRD0
				- Video (LION wins): https://www.youtube.com/watch?v=NgFmQlD-6Mk
			- **Fire Mode:** Fire mode introduces a unique challenge where the environment reacts dynamically to changing temperature conditions. If the temperature exceeds a certain threshold, fires may spontaneously ignite, increasing the danger and complexity of the scenario.
				- In this mode Fire will randomly take place if temperature range is above some threshold.
				- https://www.youtube.com/watch?v=HaI-UzvJZsg
		- We experimented with aim to keep environment interesting
			- observation sizes: Usually increasing observation size is beneficial for agent. One interesting thing we found is agent will dynamically modify path as its observes obstacle where it initially thought it was empty.
			- Number of Lions: We found 2 Lions gives interesting dynamics. More result in fast collpase.
			- MAX Limits: Keeping hunger/thirst limits less results in faster collapse. Difference between limits of Lion and fox gives interesting dynamics.
