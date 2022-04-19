import logging
import time
from threading import Timer
import datetime as dt

# Used for scanning for Crazyflies instances
import cflib.crtp

# Class to connect/send/receive from Crazyflie
from cflib.crazyflie import Crazyflie

# Synchronize le bordel mais pas vraiment capté
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Classe permettant de se logger au crazyflie
from cflib.crazyflie.log import LogConfig
# Classe permettant d'accéder aux datas du crazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# Uniform Resource Identifier -> To distinguish one drone from another
uri = 'radio://0/80/2M/E7E7E7E701'
#uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Function to check connection
def simple_log(scf, logconf):
    
    with SyncLogger(scf, logconf) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))
            break
                                

def simple_connect():
    print("Connected :)")
    time.sleep(3)
    print("Goodbye :(")

if __name__ == '__main__':
    # Initialize all the drivers
    cflib.crtp.init_drivers()

    # Le nom du logConfig n'a aucune importance, c'est juste une étiquette pour nous
    # ce qui est important ce sont les variables qu'on y met
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stabilizer.roll', 'float')
    

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Tant qu'on reste dans le with, on est connecté au crazyflie.
        # Dès qu'on en sort, la connexion est perdue.
        for i in range(200):
            simple_log(scf, lg_stab)
            time.sleep(0.2)



# SYNCHRONOUS LOGGING -> ON LIT LES VALEURS LES UNES APRES LES AUTRES
# SUITE : TESTER LE CODE SUR LE DRONE PUIS FAIRE LE LOGGING ASYNCHRONOUS
# Lien : https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_connect_log_param/
        

        
    
