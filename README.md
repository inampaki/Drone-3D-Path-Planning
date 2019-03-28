## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)
---
# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

#### `motion_planning.py`
This file comprised of these parts mainly every part is defined in its own place:
* States : This class is derived from Enum for representing different states of a Drone.
* MotionPlanning : This class is derived from Drone class from udacidrone package / library. This class facilitates the event driven coding of Drone as it transits between stated and travels through all the way points. When flight state is Armed all the path planning is performed. Main steps od path planning are defining local and global state of drone, load obstacles data and create Grid using the data. Then search algorithm is performed over the grid to travel from start to goal state using a heuristic function (i.e. euclidean distance in this case other choices may also be used). When a path is found between start and goal states. This path is pruned using Bresenham technique. 
---
#### `planning_utils.py`
This is a collection of utlity functions and classes like:
* create_grid : This function creates a grid which repesents either a cell in the grid is ostacle or a fly zone
* Action : This Enum represents the directions drone can move, like Noth, East, West or Diagonal like North-East, North-West, etc
* valid_actions : This function removes all the moves from the valid valid_actions list which are not valid from a cell of grid.
* a_star : This function is the implementation of famous A star search algorithm to search the path between start state and goal state with low cost.
* heuristic : This function helps to determine the low cost path in A star
* prune_path : This function uses a helper function to remove all the points except extreme points between a free way

### Implementing Your Path Planning Algorithm
#### 1. Set your global home position
`
def get_long_lat():
    lat = None
    lon = None
    with open('./colliders.csv', newline='\n') as file_descripter:
        reader = csv.reader(file_descripter)
        first_row = next(reader)
    file_descripter.close()
    for cell in first_row:
        try:
            if cell.index("lat0") != -1:
                lat = float((cell.strip()).split(" ")[1])     
        except:
            try:
                if cell.index("lon0") != -1:
                    lon = float((cell.strip()).split(" ")[1])
            except:
                print('Nor a Lonitude nor a latitude')                  
    return(lat, lon)
`

The above listed function reads the first line from colliders.csv and returns the Geo coordinates after converting to float type. 

Following lines load the drone position and sets the Drone home position to what co-coridnates are read from Line-1 of dataset.

`
lat0, lon0 = get_long_lat()
self.set_home_position(lon0, lat0, 0.0)
`

#### 2. Set your current local position
`
local_position = global_to_local(global_position, self.global_home)
`

The above listed code snippet converts geo-cordinates to NED frame and defines the local position of drone.

#### 3. Set grid start position from local position
`grid_start = (int(np.ceil(local_position[0] - north_offset)), int(np.ceil(local_position[1] - east_offset))) `

Local co-ordinates computed in step 2 are then used to define the start state of Drone, instead of center point of Grid

#### 4. Set grid goal position from geodetic coords
`
if args.goal_lon and args.goal_lat:
    if args.goal_alt:
        goal = [float(args.goal_lon), float(args.goal_lat), float(args.goal_alt)]
    else:
        goal = [float(args.goal_lon), float(args.goal_lat), TARGET_ALTITUDE]
    local_goal = global_to_local(goal, self.global_home)
    grid_goal = (int(np.ceil(local_goal[0] - north_offset)),
                 int(np.ceil(local_goal[1] - east_offset)))
else:
    north_goal = np.random.choice(np.arange(10, 50), 1)[0]
    east_goal = np.random.choice(np.arange(10, 50), 1)[0]
    print('north_goal : ', north_goal)
    print('east_goal : ', east_goal)
    grid_goal = (-north_offset + north_goal, -east_offset + east_goal)
`

Above code snippets is used to check if goal state is provided through command line arguments then it is used otherwise it is chosen between 10-50 North and 10-50 East NED point from north_offset and east_offset.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
`
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))
    
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions
`

A star function uses valid_actions function to get valid moved from a cell so added diagonal moves in Enum Action with cost of sqrt(2) to every diagonal move. Hence if Diagonal move is valid A star will prefer to make it due to its lower cost.

#### 6. Cull waypoints 
`
pruned_path = bres_prune(grid, path)
def bres_prune(grid, path):
    """
    Use the Bresenham module to trim uneeded waypoints from path
    """
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i + 1]
        p3 = pruned_path[i + 2]
        if  all((grid[pp] == 0) for pp in bresenham(int(p1[0]), int(p1[1]), int(p3[0]), int(p3[1]))):
            pruned_path.remove(p2)
        else:
            i += 1
    return pruned_path
`

To prune the path for removing all the way points which are colinear used the Bresenham technique.
### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.