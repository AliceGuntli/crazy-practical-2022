from pynput import keyboard

import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

uri = 'radio://0/80/2M/E7E7E7E701'
deck_attached_event = Event()

DEFAULT_HEIGHT = 0.5
body_x_cmd = 0
body_y_cmd = 0

logging.basicConfig(level=logging.ERROR)

def log_pos_callback(timestamp, data, logconf):
    pass

def on_press(key):
    if(key == keyboard.Key.up):
        print("up")
        body_y_cmd = 0.1
    elif(key == keyboard.Key.down):
        print("down")
        body_y_cmd = -0.1
    elif(key == keyboard.Key.left):
        print("left")
        body_x_cmd = 0.1
    elif(key == keyboard.Key.right):
        print("right")
        body_y_cmd = -0.1
    elif(key == keyboard.Key.enter):
        listener.stop()
        
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:  
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

        
def on_release(key):
    if(key == keyboard.Key.up or key == keyboard.Key.down):
        print("STOP")
        body_y_cmd = 0
    elif(key == keyboard.Key.left or key == keyboard.Key.right):
        print("STOP")
        body_x_cmd = 0

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:    
            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached')
    else:
        print('Deck is NOT attached')

print("0")

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    
    print("1")
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        print("1.2")

##        if not deck_attached_event.wait(timeout=5):
##            print('No flow deck detected!')
##            sys.exit(1)




        #print("2")
        #listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        print("3")
        logconf.start()
        print("4")
        #listener.start()

        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
            
        print("5")
        logconf.stop()
        print("C'est la fin :'(")

