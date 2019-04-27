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
        def kalman_filter(post_mean, post_cov, y_k, r_k):
            if post_mean.size == 1:
                return y_k, r_k
            else:
                K = np.matmul(post_cov, np.linalg.inv(r_k + post_cov))
                new_mean = post_mean + np.matmul(K, y_k - post_mean)
                new_cov = np.matmul(np.matmul((np.identity(2) - K), post_cov), np.transpose(np.identity(2) - K)) + np.matmul(np.matmul(K, r_k), np.transpose(K))
                new_cov = post_cov - np.matmul(K, post_cov)
                return new_mean, new_cov

        # kalman_filter
        # Ex. average the noised positions

        for key in self.beacon_history.keys():
            beacon_index = key
            beacon_positions = self.beacon_history[beacon_index]

            observation_times = len(beacon_positions)
            post_mean = np.array([0])
            post_cov = np.array([0])
            
            for i in range(observation_times):
                noised_position = np.array(beacon_positions[i])
                y_k = noised_position.reshape((2, 1))
                sigma = 2 + self.z/50
                r_k = np.identity(2) * (sigma) ** 2
                post_mean, post_cov = kalman_filter(post_mean, post_cov, y_k, r_k)
   
            estimated_position = post_mean.tolist()
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


