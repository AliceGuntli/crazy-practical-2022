import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

logging.basicConfig(level=logging.ERROR)

######################################### CLASS ########################################

class Charles:
    def __init__(self):

        print("Bienvenue sur Charles Airline")
        
        self.uri = "radio://0/80/2M/E7E7E7E701"
        self.default_height = 0.5
        
        # Initial position in the global frame
        self.xyz0 = [0, 0, 0]

        # Position in the "take off platform" frame
        self.xyz = [0, 0, 0]
        self.rpy = [0, 0, 0]
        self.xyz_rate_cmd = [0, 0, 0]
        self.rpy_rate_cmd = [0, 0, 0]

        self.current_state = 0
        
        self.var_list = ['stateEstimate.x',
                         'stateEstimate.y',
                         'stateEstimate.z',
                         'stabilizer.roll',
                         'stabilizer.pitch',
                         'stabilizer.yaw',
                         'range.front',
                         'range.back',
                         'range.up',
                         'range.left',
                         'range.right',
                         'range.zrange']
                         
        self.Te_loop = 0.01 # Cadence la boucle principale
        self.Te_log = 0.01 # Cadence la réception des données

        print("Driver initialisation ..")
        cflib.crtp.init_drivers()

        print("Log Configuration ..")
        self.setLog()

#----------------------------------------------------------------------------------------#

    def setLog(self):

        self.log = LogConfig(name='Position', period_in_ms=self.Te_log)
        
        for var in self.var_list:
            self.log.add_variable(var, 'float')

#----------------------------------------------------------------------------------------#

    def log_callback(self, timestamp, data, logconf):
        self.xyz = [data[self.var_list[0]], data[self.var_list[1]], data[self.var_list[2]]]
        self.pry = [data[self.var_list[3]], data[self.var_list[4]], data[self.var_list[5]]]
        self.range = [data[self.var_list[6]],
                      data[self.var_list[7]],
                      data[self.var_list[8]],
                      data[self.var_list[9]],
                      data[self.var_list[10]],
                      data[self.var_list[11]]]

#----------------------------------------------------------------------------------------#

    def obstacle_avoidance(self):
        # self.range = [front, back, up, left, right, zrange]
        self.xyz_rate_cmd[0] += 0
        self.xyz_rate_cmd[1] += 0
        self.xyz_rate_cmd[2] += 0

#----------------------------------------------------------------------------------------#

    def stateMachine(self, scf):
        with MotionCommander(scf, default_height = self.default_height) as mc:
            while(1):
                if self.state == 0:

                    #---- Take off ----#

                    # default height has been reached -> Next state
                    if self.xyz[2] >= self.default_height:
                        self.state += 1
                        print("Next state : " + str(self.state))

                elif self.state == 1:

                    #---- Fly to zone 2 ----#

                    if True:
                        self.state += 1
                        print("Next state : " + str(self.state))

                elif self.state == 2:

                    #---- Search landing zone ----#

                    if True:
                        self.state += 1
                        print("Next state : " + str(self.state))

                elif self.state == 3:

                    #---- Search center of the landing zone ----#

                    if True:
                        self.state += 1
                        print("Next state : " + str(self.state))

                elif self.state == 4:

                    #---- Landing ----#

                    if True:
                        self.state = 0
                        print("Next state : " + str(self.state))

                else:
                    print("Woooooops invalid state")

                self.obstacle_avoidance()    
                mc.start_linear_motion(self.xyz_rate_cmd[0], self.xyz_rate_cmd[1], self.xyz_rate_cmd[2], self.rpy_rate_cmd[0])

                time.sleep(self.Te_loop)

#----------------------------------------------------------------------------------------#

    def run(self):
        print("Connection ..")

        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            print("Charles connecté, Charles content")

            print("Add config ..")
            scf.cf.log.add_config(self.log)
            print("Add Callback ..")
            self.log.data_received_cb.add_callback(self.log_callback)

            time.sleep(1)

            print("Start dataflow")
            self.log.start()

            print("Z'eeeeeeest parti")
            self.stateMachine(scf)

            print("Goodbye :'(")
            self.log.stop()
            
####################################### MAIN ##############################################

test = Charles()
test.run()
    
