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
        # self.range = [front, back, up, left, right, zrange]
        self.range = [0, 0, 0, 0, 0, 0]
        self.xyz_rate_cmd = [0, 0, 0]
        self.rpy_rate_cmd = [0, 0, 0]

        self.state = 0
        self.min_dist = 300 # Distance to stop flying 
        
        self.pos_var_list = ['stateEstimate.x',
                         'stateEstimate.y',
                         'stateEstimate.z',
                         'stabilizer.roll',
                         'stabilizer.pitch',
                         'stabilizer.yaw']

        self.multi_var_list = ['range.front',
                         'range.back',
                         'range.up',
                         'range.left',
                         'range.right',
                         'range.zrange']
                                    
                         
        self.Te_loop = 0.01 # Cadence la boucle principale EN SECONDES
        self.Te_log = 10 # Cadence la réception des données EN !!! MILLISECONDES !!!

        print("Driver initialisation ..")
        cflib.crtp.init_drivers()

        print("Log Configuration ..")
        self.setLog()
        
#----------------------------------------------------------------------------------------#

    def is_not_close(self):
        # False if an object is too close to the drone (up)
        return (self.range[2] > self.min_dist)

#----------------------------------------------------------------------------------------#

    def setLog(self):

        self.log_position = LogConfig(name='Position', period_in_ms=self.Te_log)
        self.log_multiranger = LogConfig(name='Multiranger', period_in_ms=self.Te_log)
        
        for var in self.pos_var_list:
            self.log_position.add_variable(var, 'float')

        for var in self.multi_var_list:
            self.log_multiranger.add_variable(var, 'float')

#----------------------------------------------------------------------------------------#

    def log_pos_callback(self, timestamp, data, logconf):
        # Get x,y,z and roll, pitch, yaw values and save it into self variables
        self.xyz = [data[self.pos_var_list[0]], data[self.pos_var_list[1]], data[self.pos_var_list[2]]]
        self.pry = [data[self.pos_var_list[3]], data[self.pos_var_list[4]], data[self.pos_var_list[5]]]

#----------------------------------------------------------------------------------------#

    def log_multi_callback(self, timestamp, data, logconf):
        # Get multiranger values and save it into self variables
        self.range = [data[self.multi_var_list[0]],
                      data[self.multi_var_list[1]],
                      data[self.multi_var_list[2]],
                      data[self.multi_var_list[3]],
                      data[self.multi_var_list[4]],
                      data[self.multi_var_list[5]]]
        
#----------------------------------------------------------------------------------------#

    def obstacle_avoidance(self):
        # self.range = [front, back, up, left, right, zrange]
        self.xyz_rate_cmd[0] += 0
        self.xyz_rate_cmd[1] += 0
        self.xyz_rate_cmd[2] += 0

#----------------------------------------------------------------------------------------#

    def stateMachine(self, scf):
        with MotionCommander(scf, default_height = self.default_height) as mc:
            while(self.is_not_close()):
                print(self.range[2])
                if self.state == 0:

                    #---- Take off ----#

                    # default height has been reached -> Next state
                    if self.xyz[2] >= self.default_height:
                        self.state += 1
                        #print("Next state : " + str(self.state))

                elif self.state == 1:

                    #---- Fly to zone 2 ----#

                    if True:
                        self.state += 1
                        #print("Next state : " + str(self.state))

                elif self.state == 2:

                    #---- Search landing zone ----#

                    if True:
                        self.state += 1
                        #print("Next state : " + str(self.state))

                elif self.state == 3:

                    #---- Search center of the landing zone ----#

                    if True:
                        self.state += 1
                        #print("Next state : " + str(self.state))

                elif self.state == 4:

                    #---- Landing ----#

                    if True:
                        self.state = 0
                        #print("Next state : " + str(self.state))

                else:
                    print("Woooooops invalid state")
   
                mc.start_linear_motion(self.xyz_rate_cmd[0], self.xyz_rate_cmd[1], self.xyz_rate_cmd[2], self.rpy_rate_cmd[0])

                #time.sleep(self.Te_loop)

#----------------------------------------------------------------------------------------#

    def run(self):
        print("Connection ..")

        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            print("Charles connecté, Charles content")

            print("Add config ..")
            scf.cf.log.add_config(self.log_position)
            scf.cf.log.add_config(self.log_multiranger)
            print("Add Callback ..")
            self.log_position.data_received_cb.add_callback(self.log_pos_callback)
            self.log_multiranger.data_received_cb.add_callback(self.log_multi_callback)

            time.sleep(1)

            print("Start dataflow")
            self.log_position.start()
            self.log_multiranger.start()

            print("Z'eeeeeeest parti")
            self.stateMachine(scf)

            print("Goodbye :'(")
            self.log_position.stop()
            self.log_multiranger.stop()
            
####################################### MAIN ##############################################

test = Charles()
test.run()
    
