import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as sps
import os
import pyulog
import sys

from argparse import ArgumentParser

"""
Plot data from ulg log file

Example Usage:
python plotLogs.py ../logs/log_59_2019-4-13-10-49-40.ulg

TODO: Implement plotMotorRpm once that data is in the logs
"""


# set global legend font-size
plt.rc('legend',**{'fontsize':8})


def plotPosition(dset_dict):
    local_dset = dset_dict['vehicle_local_position']

    fig,axes = plt.subplots(3,1,sharex=True)

    axes[0].plot((local_dset['timestamp'])/1e6,local_dset['x'])
    axes[0].grid()
    axes[0].set_ylabel('x (m)')

    axes[1].plot((local_dset['timestamp'])/1e6,local_dset['y'])
    axes[1].grid()
    axes[1].set_ylabel('y (m)')

    axes[2].plot((local_dset['timestamp'])/1e6,local_dset['z'])
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('z (m)')

    axes[0].set_title('Position')

    if 'vehicle_local_position_setpoint' in dset_dict:
        setpoint_dset = dset_dict['vehicle_local_position_setpoint']
        axes[0].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['x'], where='post')
        axes[1].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['y'], where='post')
        axes[2].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['z'], where='post')

    axes[0].legend(['Estimated', 'Setpoint'])

    return fig


def plotVelocity(dset_dict):
    local_dset = dset_dict['vehicle_local_position']

    fig,axes = plt.subplots(3,1,sharex=True)

    axes[0].plot((local_dset['timestamp'])/1e6,local_dset['vx'])
    axes[0].grid()
    axes[0].set_ylabel('vx (m/s)')

    axes[1].plot((local_dset['timestamp'])/1e6,local_dset['vy'])
    axes[1].grid()
    axes[1].set_ylabel('vy (m/s)')

    axes[2].plot((local_dset['timestamp'])/1e6,local_dset['vz'])
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('vz (m/s)')

    axes[0].set_title('Velocity')

    if 'vehicle_local_position_setpoint' in dset_dict:
        setpoint_dset = dset_dict['vehicle_local_position_setpoint']
        axes[0].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['vx'], where='post')
        axes[1].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['vy'], where='post')
        axes[2].step((setpoint_dset['timestamp'])/1e6,setpoint_dset['vz'], where='post')

    axes[0].legend(['Estimated', 'Setpoint'])

    return fig


def plotAcceleration(dset_dict):
    sensor_dset = dset_dict['sensor_combined']

    fig,axes = plt.subplots(3,1,sharex=True)

    ksize = 101      # medfilt kernel size (must be odd. for no filter use ksize=None)

    axes[0].plot((sensor_dset['timestamp'])/1e6,sps.medfilt(sensor_dset['accelerometer_m_s2[0]'],kernel_size=ksize))
    axes[0].grid()
    axes[0].legend(['Filtered Sensor'])
    axes[0].set_ylabel('ax (m/s^2)')

    axes[1].plot((sensor_dset['timestamp'])/1e6,sps.medfilt(sensor_dset['accelerometer_m_s2[1]'],kernel_size=ksize))
    axes[1].grid()
    axes[1].set_ylabel('ay (m/s^2)')

    axes[2].plot((sensor_dset['timestamp'])/1e6,sps.medfilt(sensor_dset['accelerometer_m_s2[2]'],kernel_size=ksize))
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('az (m/s^2)')

    axes[0].set_title('Acceleration')

    return fig


def plotTrajectory(dset_dict):
    local_dset = dset_dict['vehicle_local_position']

    fig,axes = plt.subplots(1,1)
    axes.plot(local_dset['x'],local_dset['y'])
    axes.grid()
    axes.set_xlabel('x (m)')
    axes.set_ylabel('y (m)')
    axes.set_title('Trajectory')

    if 'vehicle_local_position_setpoint' in dset_dict:
        setpoint_dset = dset_dict['vehicle_local_position_setpoint']
        axes.plot(setpoint_dset['x'],setpoint_dset['y'],'.')

    axes.legend(['Estimated', 'Setpoint'])

    return fig


def plotAttitude(dset_dict):
    attitude_dset = dset_dict['vehicle_attitude']

    fig,axes = plt.subplots(3,1,sharex=True)

    q_0 = attitude_dset['q[0]']
    q_1 = attitude_dset['q[1]']
    q_2 = attitude_dset['q[2]']
    q_3 = attitude_dset['q[3]']
    roll = np.arctan2(2.0 * (q_0 * q_1 + q_2 * q_3), 1.0 - 2.0 * (q_1 * q_1 + q_2 * q_2))
    pitch = np.arcsin(2.0 *(q_0 * q_2 - q_3 * q_1))
    yaw = np.arctan2(2.0 * (q_0 * q_3 + q_1 * q_2), 1.0 - 2.0 * (q_2 * q_2 + q_3 * q_3))

    axes[0].plot((attitude_dset['timestamp'])/1e6,roll)
    axes[0].grid()
    axes[0].set_ylabel('roll (rad)')

    axes[1].plot((attitude_dset['timestamp'])/1e6,pitch)
    axes[1].grid()
    axes[1].set_ylabel('pitch (rad)')

    axes[2].plot((attitude_dset['timestamp'])/1e6,yaw)
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('yaw (rad)')

    axes[0].set_title('Attitude')

    if 'vehicle_attitude_setpoint' in dset_dict:
        setpoint_dset = dset_dict['vehicle_attitude_setpoint']
        q_0 = setpoint_dset['q_d[0]']
        q_1 = setpoint_dset['q_d[1]']
        q_2 = setpoint_dset['q_d[2]']
        q_3 = setpoint_dset['q_d[3]']
        roll_set = np.arctan2(2.0 * (q_0 * q_1 + q_2 * q_3), 1.0 - 2.0 * (q_1 * q_1 + q_2 * q_2))
        pitch_set = np.arcsin(2.0 * (q_0 * q_2 - q_3 * q_1))
        yaw_set = np.arctan2(2.0 * (q_0 * q_3 + q_1 * q_2), 1.0 - 2.0 * (q_2 * q_2 + q_3 * q_3))

        axes[0].plot((setpoint_dset['timestamp'])/1e6,roll_set)
        axes[1].plot((setpoint_dset['timestamp'])/1e6,pitch_set)
        axes[2].plot((setpoint_dset['timestamp'])/1e6,yaw_set)

    axes[0].legend(['Estimated', 'Setpoint'])

    return fig


def plotAttitudeRates(dset_dict):
    rates_dset = dset_dict['vehicle_attitude']

    t0 = rates_dset['timestamp'][0]
    tf = rates_dset['timestamp'][-1]

    fig,axes = plt.subplots(3,1,sharex=True)

    axes[0].plot((rates_dset['timestamp'])/1e6,rates_dset['rollspeed'])
    axes[0].grid()
    axes[0].set_ylabel('roll rates (rad/s)')

    axes[1].plot((rates_dset['timestamp'])/1e6,rates_dset['pitchspeed'])
    axes[1].grid()
    axes[1].set_ylabel('pitch rates (rad/s)')

    axes[2].plot((rates_dset['timestamp'])/1e6,rates_dset['yawspeed'])
    axes[2].grid()
    axes[2].set_xlabel('t (s)')
    axes[2].set_ylabel('yaw rates (rad/s)')

    axes[0].set_title('Attitude Rates')

    if 'vehicle_rates_setpoint' in dset_dict:
        setpoint_dset = dset_dict['vehicle_rates_setpoint']
        idx = np.logical_and(setpoint_dset['timestamp'] > t0, setpoint_dset['timestamp'] < tf)
        axes[0].plot((setpoint_dset['timestamp'][idx])/1e6,setpoint_dset['roll'][idx])
        axes[1].plot((setpoint_dset['timestamp'][idx])/1e6,setpoint_dset['pitch'][idx])
        axes[2].plot((setpoint_dset['timestamp'][idx])/1e6,setpoint_dset['yaw'][idx])

    axes[0].legend(['Estimated', 'Setpoint'])

    return fig


def plotBatteryStatus(dset_dict):
    # plot voltage and current

    battery_set = dset_dict['battery_status']

    fig,axes = plt.subplots(1,1)
    axes.plot((battery_set['timestamp'])/1e6,battery_set['current_a'])
    axes.plot((battery_set['timestamp'])/1e6,battery_set['voltage_v'])
    axes.plot((battery_set['timestamp'])/1e6,battery_set['remaining']*10)
    axes.plot((battery_set['timestamp'])/1e6,battery_set['discharged_mah']/100)

    axes.grid()
    axes.legend(['Current (A)', 'Voltage (V)', 'Battery [0=empty, 10=full]','Discharged [mAh/100]'])
    axes.set_xlabel('t (s)')

    axes.set_title('Battery Status')

    return fig


def plotActuator(dset_dict):
    # plot actuator control

    actuator_set = dset_dict['actuator_outputs']

    fig,axes = plt.subplots(1,1)

    axes.plot((actuator_set['timestamp'])/1e6,sps.medfilt(actuator_set['output[0]'],kernel_size=15),'C0')
    axes.plot((actuator_set['timestamp'])/1e6,sps.medfilt(actuator_set['output[1]'],kernel_size=15),'C1')
    axes.plot((actuator_set['timestamp'])/1e6,sps.medfilt(actuator_set['output[2]'],kernel_size=15),'C2')
    axes.plot((actuator_set['timestamp'])/1e6,sps.medfilt(actuator_set['output[3]'],kernel_size=15),'C3')

    axes.plot((actuator_set['timestamp'])/1e6,actuator_set['output[0]'],'C0',alpha=0.1)
    axes.plot((actuator_set['timestamp'])/1e6,actuator_set['output[1]'],'C1',alpha=0.1)
    axes.plot((actuator_set['timestamp'])/1e6,actuator_set['output[2]'],'C2',alpha=0.1)
    axes.plot((actuator_set['timestamp'])/1e6,actuator_set['output[3]'],'C3',alpha=0.1)


    axes.grid()
    axes.legend(['Top Right', 'Bottom Left', 'Top Left', 'Bottom Right'])
    axes.set_xlabel('t (s)')

    axes.set_title('Actuator Outputs')

    return fig


def loadDataset(filename):
    """Load datasets into a nested dictionary format"""

    # use these keys to differentiate duplicate datasets
    dup_dataset_key = {'actuator_outputs' : 'output[1]',
                       'battery_status'   : 'current_a',
                       'input_rc'         : 'values[0]'
                        }

    ulog = pyulog.ULog(filename)

    # combine datasets into dictionary
    dataset_dict = {}
    for d in ulog.data_list:
        if d.name in dataset_dict.keys():
            if d.name in dup_dataset_key:
                key = dup_dataset_key[d.name]
                dset_old = dataset_dict[d.name]

                std_old = np.std(dset_old[key])
                std_new = np.std(d.data[key])

                if std_new > std_old:       # compare variation of data
                    dataset_dict[d.name] = d.data   # replace old dataset
            else:
                print('WARNING: ignoring duplicate dataset: {:}'.format(d.name))
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
