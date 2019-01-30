# Project: 3D Motion Planning

## Objective: Path Planning

The underlying objective of the given task in [README(udacity).md](README(udacity).md) is to calculate the shortest path, in the form of waypoints, from a __starting position__ to a **goal location** while avoiding obstacles in the city which are provided in the form of a 'csv' file, [colliders.csv](colliders.csv)  Using the  skills taught in the lesson, I have chosen and implemented 2 approaches:

**NOTE:** The following code is written in python 

1. [**2D Grid Map**](#2d-grid-map)
2. [**Probabilistic Roadmap**](#probabilistic-roadmap)

## 2D Grid Map

All of the relevant code is written in the file [motion_planning.py](motion_planning.py) and to make the programming less clustered, some of the functions are written in the file [planning_utils.py](planning_utils.py).

#### Setting Quadrotor Location

First and foremost, for all of this to work, the quadrotor will have to be able to identify its location in longitude, latitude and altitude.  Additionally the quadrotor will also have to set its **home global position** where the quadrotor will use NED coordinated with reference to its home position to issue waypoints.

```python

file = 'colliders.csv'
latlon = []
latlon = open(file).readline().split()

lat_h,lon_h = float(latlon[1].replace(',','')),float(latlon[3])
self.set_home_position(lon_h,lat_h,0.0)
global_position = self.global_position

```
#### Creating the 2D grid

Next the map will be discretized into a 2D grid by utilizing the function **create_grid()**, found in **planning_utils.py**, which will return a grid with additional variavbles such as **north offset** and __east_offset__.  These 2 variables will be used in later code in order to correctly represent the position of the quadrotor onto the 2D that has just been created.

``` python

# creating a 2D grid from the colliders.csv
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
# Define a grid for a particular altitude and safety margin around obstacles
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)         

```

#### Start and Goal
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

#### Finding path

**A Star** approach is used in order to find the shortest path from start to goal location with the help of normalization heuristic function.  More detail regarding these functions can be found in [planning_utils.py](planning_utils.py), however; this approach creates multiple redundant waypoints where in a straight line, there are more than 2 waypoints making the quadrotor stop way too often than needed.

```python

  path, _ = a_star(grid, heuristic, grid_start, grid_goal)
  pruned_path = prune_path(path,grid)
  print('Path length: ',len(pruned_path))
  print(pruned_path)
  # Convert path to waypoints
  waypoints = [[p[0] + north_offset, p[1]+east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]


```

As noticed, to have a smoother flight and transition a __prune path()__ method has been used in order for the drone to include only necessary waypoints.

```python

def point(p):
    return np.array([p[0],p[1],1.]).reshape(1,-1)

def collinearity_check(p1,p2,p3,epsilon=1):
    m = np.concatenate((p1,p2,p3),0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def in_collision(coords,grid):
    for coord in coords:
        x,y = coord
        if grid[x,y] == 1:
            return True
        
    return False

def prune_path(path,grid):
    pruned_path = [p for p in path]
    
    j=0
    while j < len(pruned_path)-2:
        p4=pruned_path[j]
        p5=pruned_path[j+2]
        cells = bresenham(p4[0],p4[1],p5[0],p5[1])
        if not in_collision(cells,grid):
            del pruned_path[j+1]
        else:
            j+=1   
    i=0
    while i < len(pruned_path)-2:
        p1=point(pruned_path[i])
        p2=point(pruned_path[i+1])
        p3=point(pruned_path[i+2])                   
        if collinearity_check(p1,p2,p3):
            del pruned_path[i+1]
        else:
            i+=1
    return pruned_path

```

## Probabilistic Roadmap

All of the relevant code is written in the file [motion_planning_prob.py](motion_planning_prob.py) and to make the programming less clustered, some of the functions are written in the file [planning_utils_prob.py](planning_utils_prob.py).

#### Creating Polygons

Similar to the [2D Grid Map](#2d-grid-map) the starting location is defined similarly but instead of creating a 2D Grid, polygons are created from the [colliders.csv](colliders.csv) file.

```python
  data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
  polygons = create_polygon(data,SAFETY_DISTANCE)

```
#### Feasible Points

Next random positions in the spcified size of the map [colliders.csv](colliders.csv) are chosen and cross checked to determine whether the points are in colliding or within an obstacle to result in a dataset of only feasible points.

```python

def Sample(data,Sample_Size,radius,tree,polygons):
    xmin = np.floor(np.min(data[:,0] - data[:,3]))
    xmax = np.ceil(np.max(data[:,0] + data[:,3]))
    ymin = np.floor(np.min(data[:,1] - data[:,4]))
    ymax = np.ceil(np.max(data[:,1] + data[:,4]))
    zmin = 0
    zmax = 10
    
    xval = np.random.uniform(xmin,xmax,Sample_Size)
    yval = np.random.uniform(ymin,ymax,Sample_Size)
    zval = np.random.uniform(zmin,zmax,Sample_Size)
    samples = list(zip(xval,yval,zval))
    
    usable = []
    for s in samples:
        in_collision = False
        idxs = list(tree.query_radius(np.array([s[0],s[1]]).reshape(1,-1),r=radius)[0])
        if len(idxs)>0:
            for idx in idxs:
                p = polygons[int(idx)]
                if(p[0].contains(Point(s)) and p[1]>=s[2]):
                   in_collision = True
        if not in_collision:
            usable.append(s)
                       
    print("Found ",len(usable),"feasible points")
    
    return usable

```

#### Generating Graph

After acquiring multiple feasible points, a graph needs to be generated to determine the feasible pathways or connections between these points.  The user defined **can_connect** can be identified in the [planning_utils_prob.py](planning_utils_prob.py) file.

```python

  radius = 2*np.max((data[:,3],data[:,4]))
  centers = np.array([(p[0].centroid.x , p[0].centroid.y) for p in polygons])
  obstructions = KDTree(centers,metric='euclidean')
  usable = Sample(data,700,radius,obstructions,polygons)
  usable.append(start_position)
        
  g = nx.Graph()
  nearest_tree = KDTree(usable)
  infeasable = 0
  neighbors = 10
  for n1 in usable:
  idxs = nearest_tree.query([n1],neighbors,return_distance = False)[0]
    for idx in idxs:
      n2 = usable[idx]
      if n2 == n1:
        continue
      if can_connect(n1,n2,polygons):
        weight = np.linalg.norm(np.array(n2)-np.array(n1))
        g.add_edge(n1,n2,weight=weight)

```

#### Path Planning

Similar the [2D Grid Map](#2d-grid-map), **A Star** and **heuristic** approach is used in order to identify the most optimal and cost effective path from starting position to a goal position and then sent as waypoints to the quadrotor to execute.

```python

  k = np.random.randint(len(g.nodes))
  path, _ = a_star(g,heuristic,start_position,goal_position)

  # Convert path to waypoints
  waypoints = [[int(p[0]), int(p[1]), TARGET_ALTITUDE, 0] for p in path]
        
```

#### License
