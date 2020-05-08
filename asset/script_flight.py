'''
Obtain datasets from 3-5 crazyflie via multiple crazyradios
Requirements: optiTrack system and crazyflie python library
Author: Shushuai Li, MAVLab, TUDelft
'''
import logging
import time
from threading import Timer

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.drivers.crazyradio import Crazyradio

import ast
import sys
from NatNetClient import NatNetClient

import numpy as np
import math

# system configurations
boa = {"xmin":-3, "xmax":3, "ymin":-3, "ymax":3, "zmin":0, "zmax":4} # dictionary of the space boarder 
all_uris = [
        'radio://0/50/2M/E7E7E7E7E5',
        'radio://1/60/2M/E7E7E7E7E6',
        'radio://2/70/2M/E7E7E7E7E7',
        'radio://3/80/2M/E7E7E7E7E8',
        'radio://4/90/2M/E7E7E7E7E9',
]
optiTrackID = [1, 2, 3, 4, 5] # drone's ID in optiTrack
max_obj_vel = 1 # m/s
numRobots = len(all_uris)
maximum_robots = 5 # This value is related to the file_write function
pos_ot = np.zeros((maximum_robots, 3)) # 3D position from optiTrack
att_ot = np.zeros((maximum_robots, 4)) # 3D attitude from optiTrack
vel3d_obj = np.zeros((numRobots, 3)) # objective velocity
cflog = [[0] * 9 for i in range(maximum_robots)] # cflog[numRobots][9] -> ([Vx, Vy, r, h, d0, d1, d2, d3, d4])
timetick = 0
flightTime = 150 # [s]

def vel_obj(step, pos_ext):
    # generate 2D velocity for all robots
    global vel3d_obj

    if (step % 500 == 0):
        vel3d_obj[:, 0:2] = ( np.random.uniform(0, max_obj_vel*2, (numRobots, 2)) - max_obj_vel ) # random Vx and Vy, Vz = 0
        # To generate non-zero velocity using below code
        # vel2d = np.random.uniform(-max_obj_vel/2, max_obj_vel/2, (numRobots, 2))
        # vel3d_obj[:, 0:2] = vel2d + np.sign(vel2d) # random Vx and Vy, but Vz = 0

    vel_gain_wall = 1
    for i in range(numRobots):
        rob_pos = pos_ext[i,:] # cyberzoo: [front X, up Z, right Y]; CF: [front X, left Y, up Z].
        if boa["xmax"]-rob_pos[0]<1.0:
            vel3d_obj[i, 0] = -vel_gain_wall*abs(vel3d_obj[i, 0])
        if rob_pos[0]-boa["xmin"]<1.0:
            vel3d_obj[i, 0] = vel_gain_wall*abs(vel3d_obj[i, 0])
        if boa["ymax"]-rob_pos[2]<1.0:
            vel3d_obj[i, 1] = vel_gain_wall*abs(vel3d_obj[i, 1])
        if rob_pos[2]-boa["ymin"]<1.0:
            vel3d_obj[i, 1] = -vel_gain_wall*abs(vel3d_obj[i, 1])

    return vel3d_obj

def vel_rep_avo(pos_ext):
    # generate repulsive velocity from other robots
    vel3d_rep_avo = np.zeros((numRobots, 3))
    for i in range(numRobots):
        [vx, vy] = [0, 0]
        posI = [pos_ext[i,0], pos_ext[i,2]]
        for j in range(numRobots):
            if j!= i:
                posJ = [pos_ext[j,0], pos_ext[j,2]]
                distIJ = math.sqrt((posI[0]-posJ[0])**2+(posI[1]-posJ[1])**2)
                if distIJ < 1:
                    vx = vx + 1/(posI[0]-posJ[0])
                    vy = vy - 1/(posI[1]-posJ[1])
        vel3d_rep_avo[i,0:2] = 0.13*np.clip(np.array([vx, vy]), -10, 10)

    return vel3d_rep_avo

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
	pass

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    global pos_ot, att_ot
    if optiTrackID.count(id)==1:
        pos_ot[optiTrackID.index(id),:] = position
        att_ot[optiTrackID.index(id),:] = rotation

streamingClient = NatNetClient() # Create a new NatNet client
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
streamingClient.run() # Run perpetually on a separate thread.

file = open('./dat00.csv', 'w')
# file.write('timeTick,   cf0Vx, cf0Vy, cf0r, cf0h, cf0d0, cf0d1, cf0d2, cf0d3, cf0d4,\
#                         cf1Vx, cf1Vy, cf1r, cf1h, cf1d0, cf1d1, cf1d2, cf1d3, cf1d4,\
#                         cf2Vx, cf2Vy, cf2r, cf2h, cf2d0, cf2d1, cf2d2, cf2d3, cf2d4,\
#                         cf3Vx, cf3Vy, cf3r, cf3h, cf3d0, cf3d1, cf3d2, cf3d3, cf3d4,\
#                         cf4Vx, cf4Vy, cf4r, cf4h, cf4d0, cf4d1, cf4d2, cf4d3, cf4d4,\
#                         cf0px, cf0py, cf0pz, cf0q1, cf0q2, cf0q3, cf0q4,\
#                         cf1px, cf1py, cf1pz, cf1q1, cf1q2, cf1q3, cf1q4,\
#                         cf2px, cf2py, cf2pz, cf2q1, cf2q2, cf2q3, cf2q4,\
#                         cf3px, cf3py, cf3pz, cf3q1, cf3q2, cf3q3, cf3q4,\
#                         cf4px, cf4py, cf4pz, cf4q1, cf4q2, cf4q3, cf4q4\n')

logging.basicConfig(level=logging.ERROR) # Only output errors from the logging framework

class LoggingExample:

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """
        self._cf = Crazyflie(rw_cache='./cache')
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        print('Connecting to %s' % link_uri) # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)
        self.is_connected = True # Variable used to keep main loop occupied until disconnect
        time.sleep(4) # wait sometime for connection

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)
        self._lg_swarm = LogConfig(name='Swarmstate', period_in_ms=10)
        self._lg_swarm.add_variable('swarmstate.swaVx', 'float')
        self._lg_swarm.add_variable('swarmstate.swaVy', 'float')
        self._lg_swarm.add_variable('swarmstate.swaGz', 'float')
        self._lg_swarm.add_variable('swarmstate.swah', 'float')
        try:
            self._cf.log.add_config(self._lg_swarm)
            self._lg_swarm.data_received_cb.add_callback(self._swarm_log_data)
            self._lg_swarm.error_cb.add_callback(self._swarm_log_error)
            self._lg_swarm.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        self._lg_dist = LogConfig(name='Ranging', period_in_ms=10)
        self._lg_dist.add_variable('ranging.distance0', 'uint16_t')
        self._lg_dist.add_variable('ranging.distance1', 'uint16_t')
        self._lg_dist.add_variable('ranging.distance2', 'uint16_t')
        self._lg_dist.add_variable('ranging.distance3', 'uint16_t')
        self._lg_dist.add_variable('ranging.distance4', 'uint16_t')
        try:
            self._cf.log.add_config(self._lg_dist)
            self._lg_dist.data_received_cb.add_callback(self._dist_log_data)
            self._lg_dist.error_cb.add_callback(self._dist_log_error)
            self._lg_dist.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Distance log config, bad configuration.')           

        t = Timer(flightTime, self._cf.close_link) # Start a timer to disconnect flightTime seconds
        t.start()

    def _swarm_log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _swarm_log_data(self, timestamp, data, logconf):
        global cflog, timetick, pos_ot, att_ot
        if all_uris.count(self._cf.link_uri)==1:
            cfID = all_uris.index(self._cf.link_uri)
            cflog[cfID][0] = data['swarmstate.swaVx']
            cflog[cfID][1] = data['swarmstate.swaVy']
            cflog[cfID][2] = data['swarmstate.swaGz']
            cflog[cfID][3] = data['swarmstate.swah']
            if cfID==0:
                timetick = timestamp
                file.write('{},  {}, {}, {}, {}, {}, {}, {}, {}, {},  {}, {}, {}, {}, {}, {}, {}, {}, {},  {}, {}, {}, {}, {}, {}, {}, {}, {},\
                    {}, {}, {}, {}, {}, {}, {}, {}, {},  {}, {}, {}, {}, {}, {}, {}, {}, {},  {}, {}, {}, {}, {}, {}, {},  {}, {}, {}, {}, {}, {}, {},  \
                    {}, {}, {}, {}, {}, {}, {},  {}, {}, {}, {}, {}, {}, {},  {}, {}, {}, {}, {}, {}, {}\n'.format(timetick,\
                    cflog[0][0], cflog[0][1], cflog[0][2], cflog[0][3], cflog[0][4], cflog[0][5], cflog[0][6], cflog[0][7], cflog[0][8], \
                    cflog[1][0], cflog[1][1], cflog[1][2], cflog[1][3], cflog[1][4], cflog[1][5], cflog[1][6], cflog[1][7], cflog[1][8], \
                    cflog[2][0], cflog[2][1], cflog[2][2], cflog[2][3], cflog[2][4], cflog[2][5], cflog[2][6], cflog[2][7], cflog[2][8], \
                    cflog[3][0], cflog[3][1], cflog[3][2], cflog[3][3], cflog[3][4], cflog[3][5], cflog[3][6], cflog[3][7], cflog[3][8], \
                    cflog[4][0], cflog[4][1], cflog[4][2], cflog[4][3], cflog[4][4], cflog[4][5], cflog[4][6], cflog[4][7], cflog[4][8], \
                    pos_ot[0][0], pos_ot[0][1], pos_ot[0][2], att_ot[0][0], att_ot[0][1], att_ot[0][2], att_ot[0][3], \
                    pos_ot[1][0], pos_ot[1][1], pos_ot[1][2], att_ot[1][0], att_ot[1][1], att_ot[1][2], att_ot[1][3], \
                    pos_ot[2][0], pos_ot[2][1], pos_ot[2][2], att_ot[2][0], att_ot[2][1], att_ot[2][2], att_ot[2][3], \
                    pos_ot[3][0], pos_ot[3][1], pos_ot[3][2], att_ot[3][0], att_ot[3][1], att_ot[3][2], att_ot[3][3], \
                    pos_ot[4][0], pos_ot[4][1], pos_ot[4][2], att_ot[4][0], att_ot[4][1], att_ot[4][2], att_ot[4][3]))          


    def _dist_log_error(self, logconf, msg):
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _dist_log_data(self, timestamp, data, logconf):
        global cflog, timetick
        if all_uris.count(self._cf.link_uri)==1:
            cfID = all_uris.index(self._cf.link_uri)
            cflog[cfID][4] = data['ranging.distance0']
            cflog[cfID][5] = data['ranging.distance1']
            cflog[cfID][6] = data['ranging.distance2']
            cflog[cfID][7] = data['ranging.distance3']
            cflog[cfID][8] = data['ranging.distance4']   

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

inteErr = np.zeros(numRobots)
oldErr  = np.zeros(numRobots)
[kp, kd, ki] = [1.4, 0.001, 0.0001]
def pid_control(cf_id, err):
    global inteErr, oldErr
    inteErr[cf_id] = inteErr[cf_id] + err
    if inteErr[cf_id] > 1:
        inteErr[cf_id] = 1
    elif inteErr[cf_id] < -1:
        inteErr[cf_id] = -1
    cmd_vel = kp*err + kd*(err-oldErr[cf_id]) + ki*inteErr[cf_id]
    oldErr[cf_id] = err
    return cmd_vel

if __name__ == '__main__':

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    le = [LoggingExample(all_uris[i]) for i in range(numRobots)]

    for i in range(numRobots):
        le[i]._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        le[i]._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

    for y in range(300):
        for i in range(numRobots):
            le[i]._cf.commander.send_velocity_world_setpoint(0, 0, pid_control(i, 0.6-pos_ot[i,1]), 0)
            pass
        time.sleep(0.01)

    step = 0
    while True:
        try:
            step = step + 1
            le_vel = vel_obj(step, pos_ot) + vel_rep_avo(pos_ot)
            for i in range(numRobots):
                q = att_ot[i,:] / np.linalg.norm(att_ot[i,:])
                yaw = math.atan2( -2*(q[1]*q[3]-q[0]*q[2]), q[0]**2-q[1]**2-q[2]**2+q[3]**2)
                vXcf = le_vel[i,0] * math.cos(yaw) - le_vel[i,1] * math.sin(yaw)
                vYcf = le_vel[i,0] * math.sin(yaw) + le_vel[i,1] * math.cos(yaw)
                le[i]._cf.commander.send_velocity_world_setpoint(vXcf, vYcf, pid_control(i, 0.6-pos_ot[i,1] ), 0) # vx, vy, vz, yawrate
            time.sleep(0.01)
        except KeyboardInterrupt:
            print("stop")
            for y in range(5):
                for i in range(numRobots):
                    le[i]._cf.commander.send_velocity_world_setpoint(0, 0, -0.5, 0)
                time.sleep(0.1)
            time.sleep(3)
            file.close()
            raise