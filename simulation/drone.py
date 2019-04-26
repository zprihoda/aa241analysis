import math
import numpy as np

"""
Drone class file
TODO: planner, estimater
"""

class drone:
    def __init__(self):
        # assume that drone start from the corner of the map and 50 meter height
        self.x = -160
        self.y = -160
        self.z = 50
        self.t = 0
        self.max_vel = 40
        self.input = True
        self.beacon_history = dict()
        self.estimated_beacon = dict()
        self.trajectory = [tuple((self.x, self.y, self.z))]

    def move(self):
        # move drone from planner data
        self.t += 1
        if self.input == False:
            # just random movement with maximum velocity or stay idle
            movement = np.random.normal(0, 1, 3)
            movement = self.max_vel * movement/math.sqrt(sum(i**2 for i in movement))

        else:
            # move by path planner
            movement = self.path_planner()

        # update position
        self.x = self.x + movement[0]
        self.y = self.y + movement[1]
        self.z = self.z + movement[2]
        # save trajectory
        self.trajectory.append(tuple((self.x, self.y, self.z)))

    def get_info(self, index, position):
        # save beacon signal in history
        if index not in self.beacon_history.keys():
            self.beacon_history[index] = [position]
        else:
            self.beacon_history[index].append(position)

    def estimate(self):
        # kalman_filter
        # Ex. average the noised positions

        for key in self.beacon_history.keys():
            beacon_index = key
            beacon_positions = self.beacon_history[beacon_index]

            estimated_position = [0, 0]
            for noised_position in beacon_positions:
                estimated_position[0] += noised_position[0]
                estimated_position[1] += noised_position[1]
            estimated_position = [x/len(beacon_positions) for x in estimated_position]

            self.estimated_beacon[beacon_index] = estimated_position

    def path_planner(self):
        # implement path planner(This generates next target position, but it can generate velocity whatever you want)
        # Ex. Go through map from the bottom to top
        if ((self.y + 160) / self.max_vel) % 2 == 0:
            if self.x == 160:
                movement = [0, self.max_vel, 0]
            else:
                movement = [self.max_vel, 0, 0]
        else:
            if self.x == -160:
                movement = [0, self.max_vel, 0]
            else:
                movement = [-self.max_vel, 0, 0]
        return movement


