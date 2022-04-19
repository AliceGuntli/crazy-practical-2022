# Test de l'asynchronous logging
# TOC : List of the logging variables defined in the Crazyflie

##import logging
##import time
##
##import cflib.crtp
##from cflib.crazyflie import Crazyflie
##from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
##
##from cflib.crazyflie.log import LogConfig
##from cflib.crazyflie.syncLogger import SyncLogger
##
##uri = 'radio://0/80/2M/E7E7E7E701'
##
### Only output errors from the logging framework
##logging.basicConfig(level=logging.ERROR)
##
##def param_stab_est_callback(name, value):
##    print('The crazyflie has parameter ' + name + 'set at number: ' + value)
##
##def simple_param_async(scf, groupstr, namestr):
##    cf = scf.cf
##    full_name = groupstr + "." + namestr
##    cf.param.add_update_callback(group=groupstr, name=namestr, cb=param_stab_est_callback)
##    time.sleep(1)
##    # Modify a parameter : Par exemple la méthode de calcul de l'estimateur
##    cf.param.set_value(full_name, 1)
##
### Callback function
##def log_stab_callback(timestamp, data, logconf):
##    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
##
##def simple_log_async(scf, logconf):
##    cf = scf.cf
##    cf.log.add_config(logconf)
##    logconf.data_received_cb.add_callback(log_stab_callback)
##
##    # Démarre la lecture des paramètes
##    logconf.start()
##    time.sleep(20)
##    # Stop la lecture des paramètres
##    logconf.stop()
##
##if __name__ == '__main__':
##
##    # Initialize the low-level drivers
##    cflib.crtp.init_drivers()
##
##    lg_stab = LogConfig(name='Kalman_pred', period_in_ms=100)
##    lg_stab.add_variable('kalman_pred.measNX', 'float')
##    lg_stab.add_variable('kalman_pred.measNY', 'float')
##    lg_stab.add_variable('kalman_pred.predNY', 'float')
##
##    group = "stabilizer"
##    name = "estimator"
##
##    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
##        simple_param_async(scf, group, name)


import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E701'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def param_stab_est_callback(name, value):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)


def simple_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr + '.' + namestr

    cf.param.add_update_callback(group=groupstr, name=namestr,
                                 cb=param_stab_est_callback)
    time.sleep(1)
    cf.param.set_value(full_name, 2)
    time.sleep(1)
    cf.param.set_value(full_name, 1)
    time.sleep(1)


def log_stab_callback(timestamp, data, logconf):
    ...
def simple_log_async(scf, logconf):
    ...
def simple_log(scf, logconf):
    ...
def simple_connect():
    ...

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    group = 'stabilizer'
    name = 'estimator'

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        simple_param_async(scf, group, name)    
