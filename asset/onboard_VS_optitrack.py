import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

from NatNetClient import NatNetClient
import numpy as np
pos2 = np.zeros(3) # 3D position from optiTrack
att2 = np.zeros(4) # 3D attitude from optiTrack
pos3 = np.zeros(3) # 3D position from optiTrack
att3 = np.zeros(4) # 3D attitude from optiTrack
rlxCF = 0
rlyCF = 0
rlyawCF = 0
# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
	pass
# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    global pos2, att2, pos3, att3
    if id==1:
        pos3[:] = position
        att3[:] = rotation
    if id==2:
        pos2[:] = position
        att2[:] = rotation
streamingClient = NatNetClient() # Create a new NatNet client
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
streamingClient.run() # Run perpetually on a separate thread.

class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='relative_pos', period_in_ms=200)
        self._lg_stab.add_variable('relative_pos.rlX0', 'float')
        self._lg_stab.add_variable('relative_pos.rlY0', 'float')
        self._lg_stab.add_variable('relative_pos.rlYaw0', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 10s
        t = Timer(1000, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback froma the log API when data arrives"""
        global rlxCF, rlyCF, rlyawCF
        rlxCF = data['relative_pos.rlX0']
        rlyCF = data['relative_pos.rlY0']
        rlyawCF = data['relative_pos.rlYaw0']

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    le = LoggingExample('radio://0/60/2M/E7E7E7E7E6')
    while le.is_connected:
    # while 1:
        time.sleep(0.3)
        q = att2 / np.linalg.norm(att2)
        yaw2 = -np.arctan2( -2*(q[1]*q[3]-q[0]*q[2]), q[0]**2-q[1]**2-q[2]**2+q[3]**2)
        q3 = att3 / np.linalg.norm(att3)
        yaw3 = -np.arctan2( -2*(q3[1]*q3[3]-q3[0]*q3[2]), q3[0]**2-q3[1]**2-q3[2]**2+q3[3]**2)
        p2 = np.array([pos2[0], -pos2[2], pos2[1]])
        p3 = np.array([pos3[0], -pos3[2], pos3[1]])
        rlxE = p3[0] - p2[0]
        rlyE = p3[1] - p2[1]
        rlxB = rlxE * np.cos(-yaw2) - rlyE * np.sin(-yaw2)
        rlyB = rlxE * np.sin(-yaw2) + rlyE * np.cos(-yaw2)
        print("relaX:%1.2f, relaY:%1.2f, relaYaw:%2.2f" % (rlxCF, rlyCF, rlyawCF))
        # print("relaX:%1.2f, relaY:%1.2f, relaYaw:%2.2f" % (rlxB, rlyB, yaw3-yaw2))
        # print("relaX01:%1.2f, relaY01:%1.2f, relaYaw01:%2.2f; ErrX:%1.2f, ErrY:%1.2f, ErrYaw:%2.2f" % (rlxCF, rlyCF, rlyawCF, rlxCF-rlxB, rlyCF-rlyB, rlyawCF-(yaw3-yaw2)))
        # yawErr = np.arctan2(np.sin(rlyawCF-(yaw3-yaw2)), np.cos(rlyawCF-(yaw3-yaw2)))
        # print("ErrX:%1.2f, ErrY:%1.2f, ErrYaw:%2.2f" % (rlxCF-rlxB, rlyCF-rlyB, yawErr)) # rlyawCF-(yaw3-yaw2)



