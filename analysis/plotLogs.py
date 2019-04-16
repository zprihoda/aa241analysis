import matplotlib.pyplot as plt
import numpy as np
import pyulog
import sys


"""
Plot data from ulg log file

Example Usage:
python plotLogs.py ../logs/log_59_2019-4-13-10-49-40.ulg

TODO: Implement plotAttitude (need to convert quaternions to euler angles)
TODO: Check units of data in plotMotorRpm
TODO: Add argumentparser
    arguments:
        filename: positional argument to replace using sys.argv
        --full : plot all possible data (action='store_true', default=false).  when false,
                 only generate some subset (TBD) of plots
        --save : Save plots to specified location (default=false)

TODO: Add other plots that we decide may be useful
"""


def plotPosition(dset_dict):
    local_dset = dset_dict['vehicle_local_position']
    setpoint_dset = dset_dict['vehicle_local_position_setpoint']

    t0 = local_dset['timestamp'][0]

    fig,axes = plt.subplots(3,1,sharex=True)

    axes[0].plot((local_dset['timestamp']-t0)/1e6,local_dset['x'])
    axes[0].plot((setpoint_dset['timestamp']-t0)/1e6,setpoint_dset['x'])
    axes[0].grid()
    axes[0].legend(['Estimated', 'Setpoint'])
    axes[0].set_ylabel('x (m)')

    axes[1].plot((local_dset['timestamp']-t0)/1e6,local_dset['y'])
    axes[1].plot((setpoint_dset['timestamp']-t0)/1e6,setpoint_dset['y'])
    axes[1].grid()
    axes[1].set_ylabel('y (m)')

    axes[2].plot((local_dset['timestamp']-t0)/1e6,local_dset['z'])
    axes[2].plot((setpoint_dset['timestamp']-t0)/1e6,setpoint_dset['z'])
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('z (m)')

    axes[0].set_title('Position')


def plotVelocity(dset_dict):
    local_dset = dset_dict['vehicle_local_position']
    setpoint_dset = dset_dict['vehicle_local_position_setpoint']

    t0 = local_dset['timestamp'][0]

    fig,axes = plt.subplots(3,1,sharex=True)

    axes[0].plot((local_dset['timestamp']-t0)/1e6,local_dset['vx'])
    axes[0].plot((setpoint_dset['timestamp']-t0)/1e6,setpoint_dset['vx'])
    axes[0].grid()
    axes[0].legend(['Estimated', 'Setpoint'])
    axes[0].set_ylabel('vx (m/s)')

    axes[1].plot((local_dset['timestamp']-t0)/1e6,local_dset['vy'])
    axes[1].plot((setpoint_dset['timestamp']-t0)/1e6,setpoint_dset['vy'])
    axes[1].grid()
    axes[1].set_ylabel('vy (m/s)')

    axes[2].plot((local_dset['timestamp']-t0)/1e6,local_dset['vz'])
    axes[2].plot((setpoint_dset['timestamp']-t0)/1e6,setpoint_dset['vz'])
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('vz (m/s)')

    axes[0].set_title('Velocity')


def plotTrajectory(dset_dict):

    local_dset = dset_dict['vehicle_local_position']
    setpoint_dset = dset_dict['vehicle_local_position_setpoint']

    fig,axes = plt.subplots(1,1)
    axes.plot(local_dset['x'],local_dset['y'])
    axes.plot(setpoint_dset['x'],setpoint_dset['y'])
    axes.set_xlabel('x (m)')
    axes.set_ylabel('y (m)')
    axes.set_title('Trajectory')


def plotAttitude(dset_dict):
    # TODO: convert quaternion to euler angles
    #   See example code at https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    attitude_dset = dset_dict['vehicle_attitude']
    setpoint_dset = dset_dict['vehicle_attitude_setpoint']

    t0 = attitude_dset['timestamp'][0]

    fig,axes = plt.subplots(3,1,sharex=True)

    raw_input('NotImplemented: Need to convert quaternion to euler angles')

    axes[0].plot((attitude_dset['timestamp']-t0)/1e6,attitude_dset['q[0]'])
    #axes[0].plot((setpoint_dset['timestamp']-t0)/1e6,setpoint_dset['x'])
    axes[0].grid()
    axes[0].legend(['Estimated', 'Setpoint'])
    axes[0].set_ylabel('x (m)')

    axes[0].set_title('Attitude')


def plotAttitudeRates(dset_dict):

    rates_dset = dset_dict['vehicle_attitude']
    setpoint_dset = dset_dict['vehicle_rates_setpoint']

    t0 = rates_dset['timestamp'][0]
    tf = rates_dset['timestamp'][-1]

    idx = np.logical_and(setpoint_dset['timestamp'] > t0, setpoint_dset['timestamp'] < tf)

    fig,axes = plt.subplots(3,1,sharex=True)

    axes[0].plot((rates_dset['timestamp']-t0)/1e6,rates_dset['rollspeed'])
    axes[0].plot((setpoint_dset['timestamp'][idx]-t0)/1e6,setpoint_dset['roll'][idx])
    axes[0].grid()
    axes[0].legend(['Estimated', 'Setpoint'])
    axes[0].set_ylabel('roll (rad/s)')

    axes[1].plot((rates_dset['timestamp']-t0)/1e6,rates_dset['pitchspeed'])
    axes[1].plot((setpoint_dset['timestamp'][idx]-t0)/1e6,setpoint_dset['pitch'][idx])
    axes[1].grid()
    axes[1].set_ylabel('pitch (rad/s)')

    axes[2].plot((rates_dset['timestamp']-t0)/1e6,rates_dset['yawspeed'])
    axes[2].plot((setpoint_dset['timestamp'][idx]-t0)/1e6,setpoint_dset['yaw'][idx])
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('yaw (rad/s)')

    axes[0].set_title('Attitude Rates')


def plotBatteryStatus(dset_dict):
    # plot voltage and current

    dset = dset_dict['battery_status']

    t0 = dset['timestamp'][0]
    tf = dset['timestamp'][-1]

    fig,axes = plt.subplots(1,1)
    axes.plot((dset['timestamp']-t0)/1e6,dset['current_a'])
    axes.plot((dset['timestamp']-t0)/1e6,dset['voltage_v'])
    axes.plot((dset['timestamp']-t0)/1e6,dset['remaining']*10)

    axes.grid()
    axes.legend(['Current (A)', 'Voltage (V)', 'Battery Remaining [0=empty, 10=full]'])
    axes.set_xlabel('t (s)')

    axes.set_title('Battery Status')


def plotMotorRpm(dset_dict):
    # plot motor rpm
    # TODO: not sure if actuator outputs correspond directly to rpm
    #       Can't find anyt documentation on these datasets

    dset = dset_dict['actuator_outputs']

    t0 = dset['timestamp'][0]
    tf = dset['timestamp'][-1]

    n_out = dset['noutputs']
    fig,axes = plt.subplots(1,1)

    for n in range(max(n_out)):
        axes.plot((dset['timestamp']-t0)/1e6,dset['output[{:}]'.format(n)])
    axes.grid()
    axes.legend(['Output {:}'.format(n) for n in range(max(n_out))])
    axes.set_xlabel('t (s)')
    axes.set_ylabel('rpm')

    axes.set_title('Motor RPM')


def loadDataset(filename):
    ulog = pyulog.ULog(filename)

    # combine datasets into dictionary
    dataset_dict = {}
    for d in ulog.data_list:
        # debug prints (useful for determining what data we have to plot)
        # will be removed as the tool is finalized
        print d.name
        print d.data.keys()
        print
        if d.name in dataset_dict.keys():
            print 'ignoring duplicate dataset: {:}'.format(d.name)
            continue
        dataset_dict[d.name] = d.data

    return dataset_dict


def main():

    # Load File
    filename = sys.argv[1]
    dataset_dict = loadDataset(filename)

    # Plot Stuff
    plotPosition(dataset_dict)
    plotVelocity(dataset_dict)
    plotTrajectory(dataset_dict)
    plotAttitudeRates(dataset_dict)
    plotBatteryStatus(dataset_dict)
    plotMotorRpm(dataset_dict)

    #plotAttitude(dataset_dict)

    plt.show()


if __name__ == '__main__':
    main()
