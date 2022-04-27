import logging
import sys
import time
from weakref import KeyedRef

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

MAX_DISTANCE = 0.5

outside = False
measured_x = 0
measured_y = 0
measured_z = 0

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def log_pos_callback(timestamp, data, logconf):
    global measured_x 
    measured_x= data['stateEstimate.x']
    global measured_y 
    measured_y = data['stateEstimate.y']
    global measured_z 
    measured_z = data['stateEstimate.z']
    
def move_to_target(scf)  :
    with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
                keep_flying = True
                goal = False
                while (keep_flying and not outside):
                    VELOCITY = 0.5
                    velocity_x = 0.0
                    velocity_y = 0.0

                    if is_close(multiranger.front):
                        velocity_x = 0.0
                        velocity_y = 0.2
                    else :
                        velocity_x = 0.2
                        velocity_y = 0.0

                    if is_close(multiranger.up):
                        keep_flying = False

                    motion_commander.start_linear_motion(
                        velocity_x, velocity_y, 0)

                    time.sleep(0.1)

            print('Demo terminated!')
    

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_alt = LogConfig(name='Altitude', period_in_ms=10)
    lg_alt.add_variable('stateEstimate.x', 'float')
    lg_alt.add_variable('stateEstimate.y', 'float')
    lg_alt.add_variable('stateEstimate.z', 'float')

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
         # add callback to the altitude listener
        scf.cf.log.add_config(lg_alt)
        lg_alt.data_received_cb.add_callback(log_pos_callback)
        time.sleep(1)

        lg_alt.start()
        move_to_target(scf)
        lg_alt.stop()

        