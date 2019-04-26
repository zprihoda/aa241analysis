import math
import numpy as np
import matplotlib.pyplot as plt
from drone import drone

"""
Beacon, Environment class file
"""

class beacon:
    def __init__(self):
        # beacon position
        self.x = None
        self.y = None
        # beacon number
        self.index = None

    def set_beacon_info(self, index, position):
        # set beacon information
        self.index = index
        self.x = position[0]
        self.y = position[1]

    def broadcast(self, drone):
        # broadcast noised information to drone

        # from location data
        h = drone.z
        x = drone.x
        y = drone.y

        # sensing diameter
        diameter = h * 5/7 + 28.57
        distance = math.sqrt((self.x - x)**2 + (self.y - y)**2)
        # sensing noise variance
        sigma = 2 + h/50

        if distance <= diameter/2:
            # if beacon within sensing boundary
            biased = [self.x + np.random.normal(0, sigma), self.y + np.random.normal(0, sigma)]
            drone.get_info(self.index, biased)
            return self.index, biased

class environment:
    def __init__(self):
        self.drone = None
        # list of beacons
        self.beacons = None
        self.num_beacon = None
        # true beacon information
        self.beacon_info = None

    def set_beacon(self, n):
        # generate N number of beacon not to be close to each other
        num = 0
        position = np.zeros((n, 2))
        while num < n:
            if num == 0:
                position[num, :] = (np.random.random((1, 2)) - 0.5) * 320
                num += 1
            else:
                position[num, :] = (np.random.random((1, 2)) - 0.5) * 320
                re_try = False
                for i in range(num):
                    if np.linalg.norm(position[num, :] - position[i, :]) < 50:
                        re_try = True
                if not re_try:
                    num += 1
        return position

    def set_environment(self, n):
        # scatter beacons on the map
        self.num_beacon = n

        beacon_position = self.set_beacon(self.num_beacon)
        beacons = []
        beacon_info = []
        for i in range(self.num_beacon):
            bea = beacon()
            position = beacon_position[i, :]
            info = tuple((i + 1, position))
            beacon_info.append(info)
            bea.set_beacon_info(i + 1, position)
            beacons.append(bea)
        self.beacons = beacons

        # true beacon information
        self.beacon_info = beacon_info
        self.drone = drone()

    def time_update(self):
        # one time step moving
        self.drone.move()
        for i in range(self.num_beacon):
            self.beacons[i].broadcast(self.drone)

    def plot(self):
        # plot beacon positions
        fig, axes = plt.subplots(1, 1)
        for i in range(self.num_beacon):
            beacon_position = self.beacon_info[i][1]
            if i == 0:
                axes.plot(beacon_position[0], beacon_position[1], 'ro', label = 'True Beacon')
            else:
                axes.plot(beacon_position[0], beacon_position[1], 'ro')
            axes.annotate("beacon" + str(i + 1), (beacon_position[0], beacon_position[1]))

        axes.grid()
        axes.legend(['True Beacon'])
        axes.set_xlabel('x (m)')
        axes.set_ylabel('y (m)')
        axes.set_xlim(-200, 250)
        axes.set_ylim(-200, 250)
        axes.set_title('Environments')

        # plot drone moving trajectory
        x_trajectory = []
        y_trajectory = []
        z_trajectory = []
        for i in range(len(self.drone.trajectory)):
            x_trajectory.append(self.drone.trajectory[i][0])
            y_trajectory.append(self.drone.trajectory[i][1])
            z_trajectory.append(self.drone.trajectory[i][2])

        axes.plot(x_trajectory, y_trajectory, 'bx-', label = 'Trajectory')

        # plot estimated beacon trajectory
        estimated_beacons = self.drone.estimated_beacon
        i = 0
        for key in estimated_beacons.keys():
            estimated_position = estimated_beacons[key]
            if i == 0:
                axes.plot(estimated_position[0], estimated_position[1], 'k^', label = 'Estimated Beacon')
            else:
                axes.plot(estimated_position[0], estimated_position[1], 'k^')
            i += 1
            # axes.annotate("est beacon" + str(key), (estimated_position[0], estimated_position[1]))
        plt.legend(loc = 'upper right', fontsize = 'small')
        plt.show()


    def print_result(self):
        # print Success
        estimated_beacons = self.drone.estimated_beacon
        success = False
        for beacon in self.beacon_info:
            beacon_index = beacon[0]
            beacon_position = beacon[1]
            if beacon_index in estimated_beacons.keys():
                estimated_position = estimated_beacons[beacon_index]
                distance = math.sqrt((beacon_position[0]- estimated_position[0])**2 + (beacon_position[1]- estimated_position[1])**2)

                if distance <= 1:
                    success = True
                    print("Success : Find Beacon_" + str(beacon_index))

        if success == False:
            print("Fail : No Beacon found")



