import numpy as np
import math
import matplotlib.pyplot as plt

class position:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def set_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class velocity:
    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.vz = 0

    def set_veloctiy(self, vx, vy, vz):
        self.vx = vx
        self.vy = vy
        self.vz = vz

class goal_line:
    def __init__(self):
        # for 2-D case
        self.coeff_a = 0
        self.coeff_b = 0
        self.coeff_c = 0
        self.coeff_d = 0
        self.direction = None

    def set_line_info(self, position_1, position_2):
        direction_vector = np.array([position_2.x - position_1.x, position_2.y - position_1.y, position_2.z - position_1.z])
        self.direction = direction_vector / np.linalg.norm(direction_vector)
        if self.direction[2] == 0:
            self.coeff_c = 0
        else:
            self.coeff_c = 1
        self.coeff_a = 1
        self.coeff_b = -(direction_vector[0] + direction_vector[2]) / direction_vector[1]
        self.coeff_d = - self.coeff_a * position_1.x - self.coeff_b * position_1.y - self.coeff_c * position_1.z
        print(self.coeff_a, self.coeff_b, self.coeff_c, self.coeff_d)

def controller(position, velocity, goal_line, v_max):
    error = - (goal_line.coeff_a * position.x + goal_line.coeff_b * position.y + goal_line.coeff_c * position.z + goal_line.coeff_d) / math.sqrt((goal_line.coeff_a)**2 + (goal_line.coeff_b)**2 + (goal_line.coeff_c)**2)
    normal_vector = np.array([goal_line.coeff_a, goal_line.coeff_b, goal_line.coeff_c])
    normal_vector = normal_vector / math.sqrt((goal_line.coeff_a)**2 + (goal_line.coeff_b)**2 + (goal_line.coeff_c)**2)
    direction_vector = goal_line.direction

    velocity = np.array([velocity.vx, velocity.vy, velocity.vz])

    error_dot = np.inner(velocity, normal_vector)
    print("pos_error : ", error, "vel_error : ", error_dot)
    k_e = 1
    k_e_dot = 0.5

    normal_output = k_e * error + k_e_dot * error_dot
    if normal_output > 3:
        normal_output = 3
    elif normal_output < -3:
        normal_output = -3

    direction_output = math.sqrt((v_max)**2 - (normal_output)**2)

    output = direction_vector * direction_output + normal_vector * normal_output

    return output

def main():
    """ Main func.
    """
    initial_position = position()
    initial_position.set_position(0, 0, 0)
    initial_velocity = velocity()
    initial_velocity.set_veloctiy(0, 0, 0)

    goal_position_1 = position()
    goal_position_1.set_position(0, 30, 0)

    goal_position_2 = position()
    goal_position_2.set_position(50, 20, 0)

    goal = goal_line()
    goal.set_line_info(goal_position_1, goal_position_2)

    v_max = 5

    position_c = initial_position
    velocity_c = initial_velocity

    fig, axes = plt.subplots(1, 1)
    axes.plot([goal_position_1.x, goal_position_2.x], [goal_position_1.y, goal_position_2.y], 'r-')
    axes.plot([position_c.x], [position_c.y], 'bo')
    for i in range(100):
        output = controller(position_c, velocity_c, goal, v_max)
        velocity_c.set_veloctiy(output[0], output[1], output[2])
        position_c.set_position(position_c.x + output[0], position_c.y + output[1], position_c.z + output[2])
        axes.plot([position_c.x], [position_c.y], 'bo')

    plt.show()

if __name__ == '__main__':
    main()


