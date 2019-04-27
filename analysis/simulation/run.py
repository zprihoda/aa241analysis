from environment import environment
from argparse import ArgumentParser

"""
Run simulation environment
Example Usage:
python run.py
"""

def main():

    # Generate environment
    env = environment()

    # number of beacon (can change)
    num_beacon = 10
    env.set_environment(num_beacon)

    # total time length (can change)
    times = 80
    for t in range(times):
        env.time_update()
    env.drone.estimate()

    # show the result
    env.print_result()
    env.plot()

if __name__ == '__main__':
    main()
