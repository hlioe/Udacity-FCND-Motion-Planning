import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid_and_edges, closest_point, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import csv
import networkx as nx
import numpy.linalg as LA
import matplotlib.pyplot as plt

plt.rcParams['figure.figsize'] = 12, 12



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

    def readrow0(self):
        desired_row=[0]
        with open('colliders.csv','r') as fin:
            reader=csv.reader(fin)
            result=[[s for s in row] for i,row in enumerate(reader) if i in desired_row]
        a=result[0]
        b="".join(a)
        c=b.split()
        lat0 = float(c[1])
        lon0 = float(c[3])

        return (lat0, lon0)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 10
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = self.readrow0()
        print ('lon0 {0}, lat0 {1}', lon0, lat0 )

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position = (lon0, lat0, 0.0)

        # TODO: retrieve current global position
        global_position =  (self._longitude, self._latitude, self._altitude)
 
        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(self.global_position, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        #print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Do a graph-based search
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center

        #grid_start_i = ( self.local_position[0], self.local_position[1] )
        #grid_start_i = (25, 100)
        #grid_start_i = (-north_offset, -east_offset)

        grid_start_i = ( -north_offset + int ( self.local_position[0] ), -east_offset + int ( self.local_position[1] ) )
        
        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert

        #grid_goal_i = ( self.local_position[0] + 100, self.local_position[1] + 100 )
        #grid_goal_i = (750, 370)
        #grid_goal_i = (35, 110)
        #grid_goal_i = (250, 250)
        #grid_goal_i = (-north_offset + 200, -east_offset + 200)

        #target_global_position =  (-122.400305, 37.791436, 0.0)
        #target_global_position =  (-122.394455, 37.793436, 0.0)
        target_global_position =  (self.global_position[0]-.003, self.global_position[1]+.001, 0.0)
        target_local_position = global_to_local(target_global_position, self.global_home)

        grid_goal_i = ( -north_offset + int ( target_local_position[0] ), -east_offset + int ( target_local_position[1] ) )        

        # Plot the graph
        plt.figure(1)
        plt.imshow(grid, origin='lower', cmap='Greys')
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    
        plt.plot(grid_start_i[1], grid_start_i[0], 'rx')
        plt.plot(grid_goal_i[1], grid_goal_i[0], 'rx')

        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        #plt.draw()
        plt.savefig('My_graph.png')

        # TODO: create the graph with the weight of the edges
        # set to the Euclidean distance between the points
        G = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            dist = LA.norm(np.array(p2) - np.array(p1))
            G.add_edge(p1, p2, weight=dist)


        grid_start = closest_point(G, grid_start_i)
        grid_goal = closest_point(G, grid_goal_i)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        #print('Local Start and Goal: ', grid_start, grid_goal)
        #path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(G, heuristic, grid_start, grid_goal)

        # Plot the path:
        plt.figure(2)
        plt.imshow(grid, origin='lower', cmap='Greys') 

        for e in edges:
            p1 = e[0]
            p2 = e[1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
            
        plt.plot([grid_start_i[1], grid_start[1]], [grid_start_i[0], grid_start[0]], 'r-')
        for i in range(len(path)-1):
            p1 = path[i]
            p2 = path[i+1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
        plt.plot([grid_goal_i[1], grid_goal[1]], [grid_goal_i[0], grid_goal[0]], 'r-')
            
        plt.plot(grid_start_i[1], grid_start_i[0], 'gx')
        plt.plot(grid_goal_i[1], grid_goal_i[0], 'gx')
        
        plt.xlabel('EAST', fontsize=20)
        plt.ylabel('NORTH', fontsize=20)
        #plt.draw()
        plt.savefig('My_path.png')

        pruned_path = prune_path(path)
        
        # Plot the pruned_path:
        plt.figure(3)
        plt.imshow(grid, origin='lower', cmap='Greys') 

        for e in edges:
            p1 = e[0]
            p2 = e[1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
            
        plt.plot([grid_start_i[1], grid_start[1]], [grid_start_i[0], grid_start[0]], 'r-')
        for i in range(len(pruned_path)-1):
            p1 = pruned_path[i]
            p2 = pruned_path[i+1]
            plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
        plt.plot([grid_goal_i[1], grid_goal[1]], [grid_goal_i[0], grid_goal[0]], 'r-')
            
        plt.plot(grid_start_i[1], grid_start_i[0], 'gx')
        plt.plot(grid_goal_i[1], grid_goal_i[0], 'gx')
        
        plt.xlabel('EAST', fontsize=20)
        plt.ylabel('NORTH', fontsize=20)
        #plt.draw()
        plt.savefig('My_pruned_path.png')

        # Convert pruned_path to waypoints
        waypoints = [[int(p[0]) + north_offset, int(p[1]) + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim
        self.send_waypoints()

        #plt.show()


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
