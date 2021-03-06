import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv
import time
import random

from planning_utils import a_star, heuristic, create_grid, prune_path, draw_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


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

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            row1 = next(reader)  # Reads the global home location (first line only)
        lon0, lat0 = float(row1[1].split()[1]), float(row1[0].split()[1])

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)   # Set the global home position

        # TODO: retrieve current global position
        current_global_pos = (self._longitude, self._latitude, self._altitude)
        
        # TODO: convert to current local position using global_to_local()
        local_start = global_to_local(current_global_pos, self.global_home)
        
        print('[INFO] global home {0}, position {1}, local position {2}'.format(self.global_home, current_global_pos, local_start))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        print('[INFO] Collider data loaded.')
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("[INFO] North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(local_start[0]-north_offset), int(local_start[1]-east_offset))
        
        # Set goal as some arbitrary position on the grid (find a position which is not occupied by an obstacle)
        global_goal = args.goal_global
        lat1, lon1 = global_goal.split(',')
        lat1, lon1 = float(lat1), float(lon1)
        global_goal = (lon1, lat1, 0.0)
        local_goal = global_to_local(global_goal, self.global_home)
        grid_goal = (int(local_goal[0]-north_offset), int(local_goal[1]-east_offset))

        # TODO: adapt to set goal as latitude / longitude position and convert
        print('[INFO] Goal set as latitude / longitude position and converted')
        
        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        t0 = time.time()
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print('[INFO] Path finding took {0} seconds'.format(time.time() - t0))
        print("[INFO] Path length without pruning is {} nodes".format(len(path)))
        
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        t0 = time.time()
        pruned_path = prune_path(path, grid)
        print("[INFO] Path length with pruning is {} nodes".format(len(pruned_path)))
        print('[INFO] Path pruning took {0} seconds'.format(time.time() - t0))

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]

        # Set self.waypoints
        if len(pruned_path) == 0:
            print("[WARNING] No path calculated!!!")
        else:
            self.waypoints = waypoints
            
            # TODO: send waypoints to sim (this is just for visualization of waypoints)
            self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("[INFO] starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":

    # import os
    # os.chdir('FCND-Motion-Planning')

    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_global', type=str, default='37.79362599197414, -122.39941897349433', help='The Geodetic goal position')
    args = parser.parse_args()

    # NOTE: Time out was changed from 600 to bypass the connection lost problem to the drone
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=600) # default 60
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()