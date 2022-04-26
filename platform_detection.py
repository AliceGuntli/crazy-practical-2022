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
measured_x = 0
measured_y = 0
measured_z = 0 # Altitude mesurée
isObstacle = False # présence d'un obstacle
z2go = 0
threshold = 0.05


def log_pos_callback(timestamp, data, logconf):
    measured_x = data['stateEstimate.x']
    measured_y = data['stateEstimate.y']
    measured_z = data['stateEstimate.z']
    
    z2go = DEFAULT_HEIGHT - measured_z
    if(abs(z2go) > threshold):
        isObstacle = True
    else:
        isObstacle = False

    print(isObstacle)

def const_alt(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        while(1):
            mc.default_height = z2go
            if(measured_x != 0):
                mc.forward(-measured_x)

            if(measured_y != 0):
                mc.left(-measured_y)
                
            #time.sleep(0.01)
        
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_alt = LogConfig(name='Altitude', period_in_ms=10)
    lg_alt.add_variable('stateEstimate.x', 'float')
    lg_alt.add_variable('stateEstimate.y', 'float')
    lg_alt.add_variable('stateEstimate.z', 'float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # add callback to the altitude listener
        scf.cf.log.add_config(lg_alt)
        lg_alt.data_received_cb.add_callback(log_pos_callback)
        time.sleep(1)
    
        lg_alt.start()
        
        #time.sleep(5)
        const_alt(scf)
        
        lg_alt.stop()
