# Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

## Description
In this project a Drone flies a path in the 3D San Francisco urban, which gets calculated
using the A* path planning algorithm. The way points the Drone flies gets reflected by
nodes which are getting calculated by the algorithm. To reduce the computation and the
nodes, the path gets pruned applying collinear check and the Bresenham algorithm. 
The first one checks if there are three successive nodes that lie on the same line and 
the second one calculates computational friendly (does not multiplication and divisions) 
a ray from point 1 to point 2. This ensures that unnecessary nodes get discarted and paths 
that traverse obstacles are not taken into account.

## Rubric points

### 1. Explain the Starter Code
The main part in this project is the 3D motion which gets handled in the motion_planning.py 
and the planning_utils.py files. Especially, the plan_path method is of particular importance.
The methods and functions that are used are:

### motion_planning.py - plan_path()
1. The target altitude and the safety distance get set to 5 meters. 

2. The global home and current global positions are determined. The home position gets read from the
collision.csv file, whereas the current global position gets determined reading the current
longitude and latitude coordinates. 

3. The current global position gets converted by the global_to_local method to the local position, which
are the coordinates north, east and down relative to the home position.

4. Using the collision data from the collision.csv file, the target altitude and the safety distance 
a grid gets created which contains the obstacles. This process is described in detail below.

5. Based on north_offset and east_offset, the start position on the grid gets calculated. This gets
done by subtracting the offsets from the local position.

6. The global goal position gets provided by the command line using the ArgumentParser class. The process
to convert the global goal position to the local goal position is the same as shown in step 3.

7. The goal position on the grid gets calculated as described in step 5.

8. Using the created grid, the grid_start position, the grid_goal position and a heuristic function,
the path from start to goal on the grid gets calculated using the A* algorithm. The heuristic function 
which gets used is the euclidean distance and the calculation gets executed in the a_star function.
This process of calculating the path is described in detail below.

9. To reduce the calculations and the number of nodes on the path, pruning gets applied which discards
unnecessary nodes. 


### planning_utils.py - create_grid()
1. 


### planning_utils.py - a_star()
1. 


### 2. Implementing Your Path Planning Algorithm


### 3. Executing the flight






We've provided you with a functional yet super basic path planning implementation and in this step, your task is to explain how it works! Have a look at the code, particularly in the plan_path() method and functions provided in planning_utils.py and describe what's going on there. This need not be a lengthy essay, just a concise description of the functionality of the starter code.

