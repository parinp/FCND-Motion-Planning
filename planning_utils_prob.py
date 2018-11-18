from enum import Enum
from queue import PriorityQueue
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
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

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


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

    return valid_actions


def a_star(graph, h, start, goal):

    path = []
    path_cost = 0 ;
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        current_cost = item[0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node,next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node,goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (new_cost, current_node)
                    queue.put((new_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
        
    return path[::-1], path_cost

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(goal_position) - np.array(position))

def create_polygon(data,safety_distance):
    polygons = []
    for i in range(data.shape[0]):
        north,east,alt,d_north,d_east,d_alt = data[i,:]
        
        obstacle = [north - d_north - safety_distance,
                    north + d_north + safety_distance,
                    east - d_east - safety_distance,
                    east + d_east + safety_distance]
        corners = [(obstacle[0],obstacle[2]),
                   (obstacle[0],obstacle[3]),
                   (obstacle[1],obstacle[3]),
                   (obstacle[1],obstacle[2])]
        
        height = alt + d_alt + safety_distance
        p = Polygon(corners)
        polygons.append((p,height))
    
    print("Polygon formed")
    return polygons

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

def can_connect(n1,n2,polygons):                   
    l1 = LineString([n1,n2])
    for p in polygons:
        if p[0].crosses(l1) and p[1]>=min(n1[2],n2[2]):
            return False
                   
    return True

def nearest_point(samples,coordinates):
    near_tree = KDTree(samples)
    coords = [coordinates[0],coordinates[1],coordinates[2]]
    idx = near_tree.query([coords],k=2,return_distance=False)[0]
    return samples[int(idx[0])]
        
        
        

