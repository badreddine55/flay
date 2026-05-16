*This project has been created as part of the 42 curriculum by badiyaf.*

# Flay In: Drone Pathfinding and Simulation

## Description

Flay In is a Python project that simulates multi-drone routing through a network of zones and connections. The goal is to route a fleet of drones from a single start hub to an end hub while respecting zone capacities, link capacities, and special zone types.

The project includes:
- a custom map parser for `.txt` map definitions
- a Dijkstra-based pathfinding engine that accounts for capacity, timing, and priority zones
- a simulation layer that prints turn-by-turn drone movements
- a visual playback mode with animated drones, capacity labels, and a turn counter

## Instructions

### Requirements
- Python 3
- `pygame`

### Installation
1. Install the required dependency:
	```bash
	pip install pygame
	```
2. Ensure the repository root contains `main.py`, `graphics.py`, and the `images/` folder.

### Execution
1. Choose a map file from `maps/`, for example:
	```bash
	python3 main.py maps/easy/01_linear_path.txt
	```
2. The program will:
	- parse the map file,
	- compute a route for each drone,
	- print the path plan,
	- run a simulation of drone movements,
	- launch a Pygame window showing the animated map.

### Map file format
Map files use the following structure:
```text
nb_drones: <positive int>
start_hub: NAME x y [max_drones=N color=COLOR]
end_hub: NAME x y [max_drones=N color=COLOR]
hub: NAME x y [zone=TYPE color=COLOR max_drones=N]
connection: ZONEA-ZONEB [max_link_capacity=N]
```
Supported zone types:
- `normal`
- `blocked`
- `restricted`
- `priority`

## Algorithm and Implementation

### Parsing and validation
The parser in `file_parser.py` performs the following tasks:
- reads map lines and skips comments and blank lines
- validates the first meaningful line is `nb_drones:` with a positive integer
- parses `start_hub`, `end_hub`, and `hub` declarations with optional metadata
- parses `connection:` lines with optional capacity metadata
- validates unique zone names, duplicate connections, and self-loops
- verifies that the end hub is reachable from the start hub using BFS while treating `blocked` zones as impassable

### Graph representation
The repository uses an adjacency list in `utils.py`.
Each zone becomes a node, and each connection becomes a bidirectional edge with a cost based on destination zone type.

Movement cost rules:
- `normal`: 1 turn
- `restricted`: 2 turns
- `blocked`: impassable
- `priority`: 1 turn with a lower effective cost to encourage selection

### Pathfinding strategy
The core pathfinding is implemented in `algorithm.py` using a Dijkstra-based search.
Key behavior includes:
- a priority queue on cumulative cost and arrival turn
- cost modifiers to prefer `priority` zones
- reservation caches for zones and links so multiple drones cannot exceed `max_drones` or `max_link_capacity`
- a wait option that allows a drone to stay in place for one turn when forward movement is blocked by capacity constraints
- path reconstruction handles restricted zones by creating transit waypoints for the additional turn cost

### Scheduling and simulation
`Simulation.run()` collects each drone's planned route and prints movements per turn in the console.
It helps verify the solution before visual playback.

## Visual Representation

The Pygame-based visualizer displays:
- a scalable window sized to the current screen resolution
- a background image loaded from `images/congruent_pentagon.png`
- hubs drawn as colored circles with their `max_drones` label
- connection lines drawn between connected hubs with capacity labels
- animated drone sprites from `images/drone.png`
- a live turn counter in the top-right corner

Visual features supporting the user experience:
- smooth interpolation between turns at 60 FPS
- random wobble offsets so overlapping drones remain distinguishable
- color-coded hub rendering based on zone metadata
- clear labels on connections and hubs for capacity and routing awareness

## Resources

- Python 3 documentation: https://docs.python.org/3/
- Pygame documentation: https://www.pygame.org/docs/
- Dijkstra algorithm overview: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
- Pathfinding and graph traversal concepts: https://www.redblobgames.com/pathfinding/a-star/introduction.html

### AI usage disclosure
I used Claude as a learning and code review tool throughout this project.

Specifically I used it to:
- Understand and break down things 
- Answer questions about design decisions and data structures

All code was written and understood by me.
Claude was used as a reviewer and explainer, not as a code generator.


## Notes

- The main entrypoint in this repository is `main.py`.
- The project uses a custom `.txt` map definition format stored under `maps/`.
