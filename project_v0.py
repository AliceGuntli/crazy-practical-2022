import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

uri = "radio://0/80/2M/E7E7E7E701"

logging.basicConfig(level=logging.ERROR)

DEFAULT_HEIGHT = 0.5
START_POS = [0, 0, 0] # Starting position x, y, z
state = 0


def log_pos_callback(timestamp, data, logconf):
    measured_x = data['stateEstimate.x']
    measured_y = data['stateEstimate.y']
    measured_z = data['stateEstimate.z']

    print(isObstacle)

def const_alt(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        while(1):

#############################################################################
            
            if state == 0:
            # Automatic Take-off

            # state += 1


#############################################################################
                
            elif state == 1:
            # Fly to zone 2 & Avoid obstacle

            # Alice & Léanne

            # state += 1

#############################################################################
            
            elif state == 2:
            # Search the platform

            # David & Stephen

            # state += 1

#############################################################################
            
            elif state == 3:
            # Search center of the platform

            # David & Stephen

            # state += 1

#############################################################################

            elif state == 4:
            # Take off in the middle of the platform

            # David & Stephen

            # state += 1

#############################################################################

            else:
            # Oooooooops state not valid
                pass
            
            time.sleep(0.01)
        
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    logdef = LogConfig(name='Altitude', period_in_ms=10)
    logdef.add_variable('stateEstimate.x', 'float')
    logdef.add_variable('stateEstimate.y', 'float')
    logdef.add_variable('stateEstimate.z', 'float')
    

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # add callback to the altitude listener
        scf.cf.log.add_config(lg_alt)
        lg_alt.data_received_cb.add_callback(log_pos_callback)
        time.sleep(1)
    
        lg_alt.start()
        
        #time.sleep(5)
        const_alt(scf)
        
        lg_alt.stop()
