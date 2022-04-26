import logging
import sys
import time
from threading import Event
import keyboard

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

uri = 'radio://0/80/2M/E7E7E7E701'
deck_attached_event = Event()

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

logging.basicConfig(level=logging.ERROR)

def log_position_callback(timestamp, data, logconf):
    x, y, z = data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']
    roll, pitch, yaw = data['stateEstimate.roll'], data['stateEstimate.pitch'], data['stateEstimate.yaw']
    print(x, y, z)
    print(roll, pitch, yaw)

def log_sensor_callback(timestamp, data, logconf):
    back, front, left, right = data['range.back'], data['range.front'], data['range.left'], data['range.right']
    #print(front)
    

def move_linear_motion(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:

        key = None
        
        while(key != "q"):
            key = keyboard.read_key()
            if key == "w":
                mc.start_left(0.2)
                print("w")
            elif key == "s":
                mc.start_right(0.2)
                print("s")
            elif key == "a":
                mc.start_forward(0.2)
                print("a")
            elif key == "d":
                mc.start_back(0.2)
                print("d")

            elif key == "h":
                mc.start_up(0.2)
                print("h")

            elif key == "b":
                mc.start_down(0.2)
                print("b")

            else:
                mc.stop()
                print("Stop")

            time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        logconf = LogConfig(name='Sensor', period_in_ms=100)
        logconf.add_variable('range.back', 'float')
        logconf.add_variable('range.front', 'float')
        logconf.add_variable('range.left', 'float')
        logconf.add_variable('range.right', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_sensor_callback)

        print("a")
        logconf2 = LogConfig(name='Position', period_in_ms=100)
        logconf2.add_variable('stateEstimate.x', 'float')
        logconf2.add_variable('stateEstimate.y', 'float')
        logconf2.add_variable('stateEstimate.z', 'float')
        logconf2.add_variable('stateEstimate.roll', 'float')
        logconf2.add_variable('stateEstimate.pitch', 'float')
        logconf2.add_variable('stateEstimate.yaw', 'float')
        scf.cf.log.add_config(logconf2)
        logconf2.data_received_cb.add_callback(log_position_callback)
        print("aa")
        logconf.start()
        print("ab")
        logconf2.start()
        print("b")
        
        move_linear_motion(scf)

        logconf.stop()
        logconf2.stop()

        
