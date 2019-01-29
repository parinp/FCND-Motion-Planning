## Project: 3D Motion Planning

### Objective: Path Planning

The underlying objective of the given task in **README(Udacity).md** is to calculate the shortest path, in the form of waypoints, from a __starting position__ to a **goal location** while avoiding obstacles in the city which are provided in the form of a 'csv' file.  Using the  skills taught in the lesson, I have chosen and implemented 2 approaches:

**NOTE:** The following code is written in python 

1. [**2D Grid Map**](#2d-grid-map)
2. [**Probabilistic Roadmap**](#probabilistic-roadmap)

### 2D Grid Map

#### Grid Formation and Location

All of the relevant code is written in the file **motion_planning.py** and to make the programming less clustered, some of the functions are written in the file **planning_utils.py**.

##### Setting Quadrotor Location

First and foremost, for all of this to work, the quadrotor will have to be able to identify its location in longitude, latitude and altitude.  Additionally the quadrotor will also have to set its **home global position** where the quadrotor will use NED coordinated with reference to its home position to issue waypoints.

```python

file = 'colliders.csv'
latlon = []
latlon = open(file).readline().split()

lat_h,lon_h = float(latlon[1].replace(',','')),float(latlon[3])
self.set_home_position(lon_h,lat_h,0.0)
global_position = self.global_position

```
##### Creating the 2D grid

Next the map will be discretized into a 2D grid by utilizing the function **create_grid()**, found in **planning_utils.py**, which will return a grid with additional variavbles such as **north offset** and __east_offset__.  These 2 variables will be used in later code in order to correctly represent the position of the quadrotor onto the 2D that has just been created.

``` python

# creating a 2D grid from the colliders.csv
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
# Define a grid for a particular altitude and safety margin around obstacles
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)         

```
Based on the grid, the starting position is located and in order to make the program more robust, the goal position will be a user defined input of __longitude__ and __latitude__ which will then be identified as a point in the grid.

```python

# Acquiring start and goal position
grid_start_latlon = (self._longitude,self._latitude,self._altitude)
grid_start_l = global_to_local(grid_start_latlon,self.global_home)

grid_start = (int(grid_start_l[0])-north_offset,int(grid_start_l[1])-east_offset)
                     
goal_lon = input('Goal longitude: ')        #goal_lon = -122.396071
goal_lat = input('Goal latitude: ')         #goal_lat = 37.793077
goal_position = (float(goal_lon),float(goal_lat),0.0)
grid_goal_l = global_to_local(goal_position,self.global_home)

grid_goal = (int(grid_goal_l[0])-north_offset,int(grid_goal_l[1])-east_offset)

```

#### Probabilistic Roadmap

All of the relevant code is written in the file **motion_planning.py** and to make the programming less clustered, some of the functions are written in the file **planning_utils.py**.

