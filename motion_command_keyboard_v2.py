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

x_vel = 0
y_vel = 0

logging.basicConfig(level=logging.ERROR)

def log_pos_callback(timestamp, data, logconf):
    #print(data)
    pass

def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_right(180)
        #mc.back(0.5)
        time.sleep(1)

def move_linear_motion(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:

##        x_vel = 0
##        y_vel = 0

        key = None
        
        while(key != "q"):
            key = keyboard.read_key()
            if key == "w":
##                x_vel = 0.2
                mc.start_left(0.2)
                print("w")
            elif key == "s":
##                x_vel = -0.2
                mc.start_right(0.2)
                print("s")
            elif key == "a":
##                y_vel = 0.2
                mc.start_forward(0.2)
                print("a")
            elif key == "d":
##                y_vel = -0.2
                mc.start_back(0.2)
                print("d")

            elif key == "h":
                mc.start_up(0.2)
                print("h")

            elif key == "b":
                mc.start_down(0.2)
                print("b")

            else:
##                x_vel = 0
##                y_vel = 0
                mc.stop()
                print("Stop")
            
            #mc.start_linear_motion(x_vel, y_vel, 0)

            time.sleep(0.05)

def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        # Dès que le motion commander est initialisé il fait un take off par défaut
        time.sleep(1)
        mc.up(0.3)
        time.sleep(1)
        mc.down(0.3)
        time.sleep(3)
        # Dès qu'il s'arrête, le drone atterit par défaut.
        mc.stop()

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached')
    else:
        print('Deck is NOT attached')

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf.start()
        
        move_linear_motion(scf)

        logconf.stop()
