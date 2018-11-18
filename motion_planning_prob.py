import argparse
import time
import msgpack
from enum import Enum, auto
import sys

import numpy as np

from planning_utils_prob import a_star, heuristic, create_grid, create_polygon, Sample, can_connect, nearest_point
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from sklearn.neighbors import KDTree
from shapely.geometry import Point
import networkx as nx


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5
        self.target_position[2] = TARGET_ALTITUDE
        
        file = 'colliders.csv'
        latlon = []
        latlon = open(file).readline().split()
        lat_h,lon_h = float(latlon[1].replace(',','')),float(latlon[3])
        self.set_home_position(lon_h,lat_h,0.0)
        global_position = self.global_position
        local_position = global_to_local(global_position,self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        start_position = (local_position[0],local_position[1],0.0)
        
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        polygons = create_polygon(data,SAFETY_DISTANCE)
        
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

        print("Finished Generating Graph")
        
        k = np.random.randint(len(g.nodes))
        goal_position = list(g.nodes)[k]
#         goal_lat_lon = (-122.398182,37.796185,0.0)
#         goal_lat_lon = (-122.396071,37.793077,0.0)    #this works 
#         goal_lat_lon = (-122.400903,37.794570,0.0)

        # User input of latitude and longitude
#         goal_lat = input('Goal Latitude: ')
#         goal_lon = input('Goal Longitude: ')
#         goal_lat_lon = (float(goal_lon) , float(goal_lat), 0.0)
#         goal = global_to_local(goal_lat_lon,self.global_home)
#         goal_position = nearest_point(usable,goal)
        print("Goal Position: ",goal_position)
        path, _ = a_star(g,heuristic,start_position,goal_position)

        # Convert path to waypoints
        waypoints = [[int(p[0]), int(p[1]), TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        print(self.waypoints)
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
