import matplotlib.pyplot as plt
import numpy as np
import os
import pyulog
import sys

from argparse import ArgumentParser

"""
Plot data from ulg log file

Example Usage:
python plotLogs.py ../logs/log_59_2019-4-13-10-49-40.ulg

TODO: Implement plotAttitude (need to convert quaternions to euler angles) - done
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

    axes[0].plot((local_dset['timestamp'])/1e6,local_dset['x'])
    axes[0].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['x'], where='post')
    axes[0].grid()
    axes[0].legend(['Estimated', 'Setpoint'])
    axes[0].set_ylabel('x (m)')

    axes[1].plot((local_dset['timestamp'])/1e6,local_dset['y'])
    axes[1].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['y'], where='post')
    axes[1].grid()
    axes[1].set_ylabel('y (m)')

    axes[2].plot((local_dset['timestamp'])/1e6,local_dset['z'])
    axes[2].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['z'], where='post')
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('z (m)')

    axes[0].set_title('Position')

    return fig


def plotVelocity(dset_dict):
    local_dset = dset_dict['vehicle_local_position']
    setpoint_dset = dset_dict['vehicle_local_position_setpoint']

    t0 = local_dset['timestamp'][0]

    fig,axes = plt.subplots(3,1,sharex=True)

    axes[0].plot((local_dset['timestamp'])/1e6,local_dset['vx'])
    axes[0].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['vx'], where='post')
    axes[0].grid()
    axes[0].legend(['Estimated', 'Setpoint'])
    axes[0].set_ylabel('vx (m/s)')

    axes[1].plot((local_dset['timestamp'])/1e6,local_dset['vy'])
    axes[1].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['vy'], where='post')
    axes[1].grid()
    axes[1].set_ylabel('vy (m/s)')

    axes[2].plot((local_dset['timestamp'])/1e6,local_dset['vz'])
    axes[2].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['vz'], where='post')
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('vz (m/s)')

    axes[0].set_title('Velocity')

    return fig


def plotAcceleration(dset_dict):
    sensor_dset = dset_dict['sensor_combined']

    t0 = sensor_dset['timestamp'][0]

    fig,axes = plt.subplots(3,1,sharex=True)

    axes[0].plot((sensor_dset['timestamp'])/1e6,sensor_dset['accelerometer_m_s2[0]'])
    axes[0].grid()
    axes[0].legend(['Sensored'])
    axes[0].set_ylabel('ax (m/s^2)')

    axes[1].plot((sensor_dset['timestamp'])/1e6,sensor_dset['accelerometer_m_s2[1]'])
    axes[1].grid()
    axes[1].set_ylabel('ay (m/s^2)')

    axes[2].plot((sensor_dset['timestamp'])/1e6,sensor_dset['accelerometer_m_s2[2]'])
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('ax (m/s^2)')

    axes[0].set_title('Acceleration')

    return fig


def plotTrajectory(dset_dict):

    local_dset = dset_dict['vehicle_local_position']
    setpoint_dset = dset_dict['vehicle_local_position_setpoint']

    fig,axes = plt.subplots(1,1)
    axes.plot(local_dset['x'],local_dset['y'])
    axes.plot(setpoint_dset['x'],setpoint_dset['y'])
    axes.grid()
    axes.legend(['Estimated', 'Setpoint'])
    axes.set_xlabel('x (m)')
    axes.set_ylabel('y (m)')
    axes.set_title('Trajectory')

    return fig


def plotAttitude(dset_dict):
    # TODO: convert quaternion to euler angles
    #   See example code at https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    attitude_dset = dset_dict['vehicle_attitude']
    setpoint_dset = dset_dict['vehicle_attitude_setpoint']

    t0 = attitude_dset['timestamp'][0]

    fig,axes = plt.subplots(3,1,sharex=True)

    q_0 = attitude_dset['q[0]']
    q_1 = attitude_dset['q[1]']
    q_2 = attitude_dset['q[2]']
    q_3 = attitude_dset['q[3]']
    roll = np.arctan2(2.0 * (q_0 * q_1 + q_2 * q_3), 1.0 - 2.0 * (q_1 * q_1 + q_2 * q_2))
    pitch = np.arcsin(2.0 *(q_0 * q_2 - q_3 * q_1))
    yaw = np.arctan2(2.0 * (q_0 * q_3 + q_1 * q_2), 1.0 - 2.0 * (q_2 * q_2 + q_3 * q_3))

    q_0 = setpoint_dset['q_d[0]']
    q_1 = setpoint_dset['q_d[1]']
    q_2 = setpoint_dset['q_d[2]']
    q_3 = setpoint_dset['q_d[3]']
    roll_set = np.arctan2(2.0 * (q_0 * q_1 + q_2 * q_3), 1.0 - 2.0 * (q_1 * q_1 + q_2 * q_2))
    pitch_set = np.arcsin(2.0 * (q_0 * q_2 - q_3 * q_1))
    yaw_set = np.arctan2(2.0 * (q_0 * q_3 + q_1 * q_2), 1.0 - 2.0 * (q_2 * q_2 + q_3 * q_3))

    axes[0].plot((attitude_dset['timestamp'])/1e6,roll)
    axes[0].plot((setpoint_dset['timestamp'])/1e6,roll_set)
    axes[0].grid()
    axes[0].legend(['Estimated', 'Setpoint'])
    axes[0].set_ylabel('roll (rad)')

    axes[1].plot((attitude_dset['timestamp'])/1e6,pitch)
    axes[1].plot((setpoint_dset['timestamp'])/1e6,pitch_set)
    axes[1].grid()
    axes[1].set_ylabel('pitch (rad)')

    axes[2].plot((attitude_dset['timestamp'])/1e6,yaw)
    axes[2].plot((setpoint_dset['timestamp'])/1e6,yaw_set)
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('yaw (rad)')

    axes[0].set_title('Attitude')

    return fig


def plotAttitudeRates(dset_dict):

    rates_dset = dset_dict['vehicle_attitude']
    setpoint_dset = dset_dict['vehicle_rates_setpoint']

    t0 = rates_dset['timestamp'][0]
    tf = rates_dset['timestamp'][-1]

    idx = np.logical_and(setpoint_dset['timestamp'] > t0, setpoint_dset['timestamp'] < tf)

    fig,axes = plt.subplots(3,1,sharex=True)

    axes[0].plot((rates_dset['timestamp'])/1e6,rates_dset['rollspeed'])
    axes[0].plot((setpoint_dset['timestamp'][idx])/1e6,setpoint_dset['roll'][idx])
    axes[0].grid()
    axes[0].legend(['Estimated', 'Setpoint'])
    axes[0].set_ylabel('roll rates (rad/s)')

    axes[1].plot((rates_dset['timestamp'])/1e6,rates_dset['pitchspeed'])
    axes[1].plot((setpoint_dset['timestamp'][idx])/1e6,setpoint_dset['pitch'][idx])
    axes[1].grid()
    axes[1].set_ylabel('pitch rates (rad/s)')

    axes[2].plot((rates_dset['timestamp'])/1e6,rates_dset['yawspeed'])
    axes[2].plot((setpoint_dset['timestamp'][idx])/1e6,setpoint_dset['yaw'][idx])
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('yaw rates (rad/s)')

    axes[0].set_title('Attitude Rates')

    return fig


def plotBatteryStatus(dset_dict):
    # plot voltage and current

    battery_set = dset_dict['battery_status']

    t0 = battery_set['timestamp'][0]

    fig,axes = plt.subplots(1,1)
    axes.plot((battery_set['timestamp'])/1e6,battery_set['current_a'])
    axes.plot((battery_set['timestamp'])/1e6,battery_set['voltage_v'])
    axes.plot((battery_set['timestamp'])/1e6,battery_set['remaining']*10)

    axes.grid()
    axes.legend(['Current (A)', 'Voltage (V)', 'Battery Remaining [0=empty, 10=full]'])
    axes.set_xlabel('t (s)')

    axes.set_title('Battery Status')

    return fig


def plotActuator(dset_dict):
    # plot actuator control

    actuator_set = dset_dict['actuator_controls_0']

    t0 = actuator_set['timestamp'][0]

    fig,axes = plt.subplots(1,1)
    axes.plot((actuator_set['timestamp'])/1e6,actuator_set['control[0]'])
    axes.plot((actuator_set['timestamp'])/1e6,actuator_set['control[1]'])
    axes.plot((actuator_set['timestamp'])/1e6,actuator_set['control[2]'])
    axes.plot((actuator_set['timestamp'])/1e6,actuator_set['control[3]'])

    axes.grid()
    axes.legend(['Roll', 'Pitch', 'Yaw', 'Thrust'])
    axes.set_xlabel('t (s)')

    axes.set_title('Actuator Status')

    return fig


# def plotMotorRpm(dset_dict):
#     # plot motor rpm
#     # TODO: not sure if actuator outputs correspond directly to rpm
#     #       Can't find anyt documentation on these datasets
#     #       (Ju) : what i know is output is not rpm.

#     dset = dset_dict['actuator_outputs']

#     t0 = dset['timestamp'][0]
#     tf = dset['timestamp'][-1]

#     n_out = dset['noutputs']
#     fig,axes = plt.subplots(1,1)

#     for n in range(max(n_out)):
#         axes.plot((dset['timestamp']-t0)/1e6,dset['output[{:}]'.format(n)])
#     axes.grid()
#     axes.legend(['Output {:}'.format(n) for n in range(max(n_out))])
#     axes.set_xlabel('t (s)')
#     axes.set_ylabel('rpm')

#     axes.set_title('Motor RPM')


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

def parseArgs():

    parser = ArgumentParser(description='Plot a given log file')
    parser.add_argument('file',help="Logfile to plot")
    parser.add_argument('--save','-s',action='store_true',help="Save plots.  Will save to ./plots/{filename}_{plot-type}.png")

    args = parser.parse_args()
    return args

def saveFigure(fig, log_file_name, plot_type):

    analysis_path = '/'.join(os.path.realpath(__file__).split('/')[0:-1])
    plot_dir = analysis_path + '/plots/'
    if not os.path.exists(plot_dir):
        os.makedirs(plot_dir)

    filename = (log_file_name.split('/')[-1]).split('.')[0] + '-' + plot_type + '.png'
    filepath = plot_dir+filename
    fig.savefig(filepath)


def main():

    args = parseArgs()

    # Load File
    filename = args.file
    dataset_dict = loadDataset(filename)

    # Plot Stuff
    pos_fig   = plotPosition(dataset_dict)
    vel_fig   = plotVelocity(dataset_dict)
    accel_fig = plotAcceleration(dataset_dict)
    traj_fig  = plotTrajectory(dataset_dict)
    att_fig   = plotAttitude(dataset_dict)
    rates_fig = plotAttitudeRates(dataset_dict)
    batt_fig  = plotBatteryStatus(dataset_dict)
    act_fig   = plotActuator(dataset_dict)
#     plotMotorRpm(dataset_dict)

    if args.save:
        saveFigure(pos_fig, filename, 'position')
        saveFigure(vel_fig, filename, 'velocity')
        saveFigure(accel_fig, filename, 'acceleration')
        saveFigure(traj_fig, filename, 'trajectory')
        saveFigure(att_fig, filename, 'attitude')
        saveFigure(rates_fig, filename, 'attitude-rates')
        saveFigure(batt_fig, filename, 'battery')
        saveFigure(act_fig, filename, 'actuator')
    else:
        plt.show()


if __name__ == '__main__':
    main()
