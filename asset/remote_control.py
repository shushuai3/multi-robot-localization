import logging
import random
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie

import sys

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class ParamExample:

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

        self._param_check_list = []
        self._param_groups = []

        random.seed()

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        self._cf.param.add_update_callback(group='relative_ctrl',
                                            name='keepFlying',
                                            cb=self._a_pitch_kd_callback)
        if int(sys.argv[1]):
            self._cf.param.set_value('relative_ctrl.keepFlying', '1')
            print('Start flights')
        else:
            self._cf.param.set_value('relative_ctrl.keepFlying', '0')
            print('stop flights')

        print('')

    def _a_pitch_kd_callback(self, name, value):
        """Callback for pid_attitude.pitch_kd"""
        print('Readback: {0}={1}'.format(name, value))

        # End the example by closing the link (will cause the app to quit)
        self._cf.close_link()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
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
    cflib.crtp.init_drivers(enable_debug_driver=False)
    pe = ParamExample('radio://0/50/2M/E7E7E7E7E5')
    while pe.is_connected:
        time.sleep(0.01)
    pe = ParamExample('radio://0/60/2M/E7E7E7E7E6')
    while pe.is_connected:
        time.sleep(0.01)
    pe = ParamExample('radio://0/70/2M/E7E7E7E7E7')
    while pe.is_connected:
        time.sleep(0.01)
    pe = ParamExample('radio://0/80/2M/E7E7E7E7E8')
    while pe.is_connected:
        time.sleep(0.01)
    pe = ParamExample('radio://0/90/2M/E7E7E7E7E9')
    while pe.is_connected:
        time.sleep(0.01)
    pe = ParamExample('radio://0/100/2M/E7E7E7E7EA')
    while pe.is_connected:
        time.sleep(0.01)
    pe = ParamExample('radio://0/110/2M/E7E7E7E7EB')
    while pe.is_connected:
        time.sleep(0.01)