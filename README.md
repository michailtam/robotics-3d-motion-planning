# Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

## Description
In this project a Drone flies a path in the [FCND-Simulator](https://github.com/udacity/FCND-Simulator-Releases/releases) which was created by Udacity for the [Flying Car and Autonomous Flight Engineer](https://www.udacity.com/course/flying-car-nanodegree--nd787) Nano degree program. The simulator shows a 3D environment of the San Francisco urban and can be used to execute different operations like 3D motion planning. The path gets calculated using the [A* path planning algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm). The way points the Drone flies gets reflected by nodes which are getting calculated by this algorithm. To reduce the computation and the nodes, the path gets pruned applying [collinear](https://en.wikipedia.org/wiki/Collinearity) check and the [Bresenham line](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm) algorithm. The first one calculates using Linear Algebra if there are three successive nodes that lie on the same line (i.e. if the determinant is 0) and the second one calculates computational friendly (does not multiplication and divisions) a ray from one point to another. Both algorithms ensure that unnecessary nodes get discarted and paths that traverse obstacles are not taken into account.

## Rubric points

## 1. Code explanation
The main part in this project is the 3D motion which gets handled in the motion_planning.py 
and the planning_utils.py files. Especially, the plan_path method is of particular importance.
The methods and functions that are used are:

### motion_planning.py - plan_path()
1. The target altitude and the safety distance get set to 5 meters. 

2. The global home and current global positions are determined. The home position gets read from the
colliders.csv file, whereas the current global position gets determined reading the current
longitude and latitude coordinates. 

3. The current global position gets converted by the global_to_local method to the local position, which
are the coordinates north, east and down relative to the home position.

4. Using the collider data from the colliders.csv file, the target altitude and the safety distance 
a grid gets created which contains the obstacles. This process is described in detail below.

5. Based on north_offset and east_offset, the start position on the grid gets calculated. This gets
done by subtracting the offsets from the local position.

6. The global goal position gets provided by the command line using the ArgumentParser class. The process
to convert the global goal position to the local goal position is the same as shown in step 3.

7. The goal position on the grid gets calculated as described in step 5.

8. Using the created grid, the grid_start position, the grid_goal position and a heuristic function,
the path from start to goal on the grid gets calculated using the A* algorithm. The heuristic function 
which gets used is the euclidean distance and the calculation gets executed in the a_star function.
The process of calculating the path is described in detail below.

9. To reduce the calculations and the number of nodes on the path, pruning gets applied which discards
unnecessary nodes. The process of how pruning on the path gets achieved, is described in detail below.

10. At the end the way points the Drone has to fly are calculated by subtracting the north and east offsets
from the remaining nodes. After that, all the waypoints are send to the simulator.


### planning_utils.py - create_grid()
1. Based on the data from the colliders.csv file the minimum, maximum and the size of the north and east 
coordinates get calculated.

2. A grid containing only zeros (white color) gets created and populated with obstacles which are calculated
based on the colliders.csv data. Before each obstacle gets created, the height of the building plus the
safety distance is bigger than the altitude of the drone. This ensures that there is no building that
smaller than the flight altitude of the drone.


### planning_utils.py - a_star()
1. Tha method takes as parameters the created grid, the heuristic function, the start and end position on the
grid. The algorithm uses a [Priority queue](https://en.wikipedia.org/wiki/Priority_queue) to determine that
only short path distances are getting checked. The queue gets initialized with the start position and a cost of
zero. The start position gets also added to the visited set.

2. Now the below steps are executed until the queue is empty:
    - The item of the queue gets read
    - If the current node is not the start node then the current branch cost is set to zero
    - A path gets found if the current node is equal to the goal node
    - For each action the following steps are executed:
      - The next node gets determined
      - The queue cost gets calculated by the branch cost + the heuristic cost (F = G + H)
    - If the next node is not in the visited list, the node gets added to it and also to the
      branch dictionary and to the queue. The branch and the queue cost get added to the respective
      data structure

3. When a path gets found, the following steps follow:
    - The overall path cost and the goal get saved
    - Each node of the calculated path will be added to the path list
    

### planning_utils.py - prune_path()
1. The method takes as parameters the calculated path list and the grid

2. Now the below steps are executed until the index reaches two nodes before the length of the list:
  - The three next points are read from the list
  - A collinear check gets executed to determine if the three points lie on the same line. If this is
    the case, the middle node gets removed.
  - In the other case, the Bresenham algorithm gets applied to determine if point 1 can be connected to
    point 3 (i.e. there is no obstacle between). If this is the case, the point 2 gets removed.


## 2. Executing the flight
| **San Francisco - Satelite** |
| :---------------------------- |
| <img src="https://github.com/michailtam/3d-motion-planning/blob/master/misc/map.png" alt="San Francisco Map - Satelite" width="700" height="500" border="0" /> |
| **San Francisco - 3D** |
| <img src="https://github.com/michailtam/3d-motion-planning/blob/master/misc/high_up.png" alt="San Francisco Map - 3D" width="700" height="500" border="0" /> |


| **Path - Corner** |
| :---------------------------- |
| <img src="https://github.com/michailtam/3d-motion-planning/blob/master/misc/corner.png" alt="Corner" width="700" height="500" border="0" /> |
| **Path - Towards the goal** |
| <img src="https://github.com/michailtam/3d-motion-planning/blob/master/misc/towards_goal.png" alt="Towards the goal" width="700" height="500" border="0" /> |
| **Flight path** |
| <img src="https://github.com/michailtam/3d-motion-planning/blob/master/misc/flying_path.png" alt="Flight path" width="700" height="500" border="0" /> |

## 3. Video
<a href="https://www.youtube.com/watch?v=CxTT9g_G1x8" target="_blank">
<img src="https://github.com/michailtam/3d-motion-planning/blob/master/misc/flying_path.png" alt="3D Motion Planning" width="700" height="500" border="0" />