import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
import numpy as np

logging.basicConfig(level=logging.ERROR)

######################################### INFO ###########################################

"""
ATTENTION : J'AI CHANGE LE SIGNE DE Y DANS LA CALLBACK ET A LA FIN DE LA STATE MACHINE QUAND
ON SET LA COMMANDE DE VITESSE POUR QUE LE SENS DES Y POSITIFS
CORRESPONDE AU SENS DU DESSIN DU PLAYGROUND -> SI CA POSE PROBLEME ON POURRA CHANGER
MAIS POUR LA PARTIE RECHERCHE DE PLATEFORME CA M'ARRANGAIT
"""

###################################### PLAYGROUND ########################################

class playground:
    def __init__(self):
        """        
                            W
        ##########################################
        #                   L               h    #
        # l   ------------------------------   l #
        #                                   |    #
        #                                   | H  #
        #                                   |    #
        #     ------------------------------     # H3
        #    |                                   #
        #    | H                                 #
        #    |            L                      #
        #  l  ------------------------------   l #
        #                                  h     #
        ##########################################
        #                                        #
        #                          xxxx          #
        #                          xxxx          #
        #                          xxxx          #
        #        xxxx                            # H2
        #        xxxx                            #
        #        xxxx                            #
        #                                        #
        ##########################################
        #                                        #
        #                                        #
        #                                        #
        #             ooooo                      #
   x0 > #             ooooo                      # H1
        #             ooooo                      #
        ^                                        #
        |                                        #
        O-->######################################
                        ^
                        y0
        """
        self.W = 1 # m
        self.H1 = 1 # m
        self.H2 = 1 # m
        self.H3 = 1.5 # m
        self.xyz0 = np.array([1, 0.4, 0.1]) # Inital position of the platform

###################################### CHARLES AIRLINES ########################################

class Charles:
    def __init__(self):

        print("Bienvenue sur Charles Airline")
        
        self.uri = "radio://0/80/2M/E7E7E7E701"
        self.default_height = 0.5

        self.playground = playground()
        
        # Initial position in the global frame
        self.xyz0 = self.playground.xyz0

        # Position in the "take off platform" frame
        self.xyz = np.array([0, 0, 0])
        self.rpy = np.array([0, 0, 0])
        
        # Position in the global frame
        self.xyz_global = self.xyz0
            
        # self.range = [front, back, up, left, right, zrange]
        self.range = np.array([0, 0, 0, 0, 0, 0])
        self.xyz_rate_cmd = np.array([0, 0, 0])
        self.rpy_rate_cmd = np.array([0, 0, 0])

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

        # Searching path variables
        self.waypoints = None
        # Constants : 
        self.l = 0.1 # marge de chaque côté en y
        self.L = self.playground.W - 2*self.l # Largeur des allers retours en y
        self.h = 0.1 # marge de chaque côté en x
        self.N = 5  # Nombre d'allers
        self.H = (self.playground.H3 - 2 * self.h)/(self.N - 1) # Ecart x entre chaque aller
                         
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

# ----------------------------------------------------------------------------------------#

    def is_close_obs(self,range): # if use of self.range, change scale to mm
        MIN_DISTANCE = 300  # mm

        if range is None:
            return False
        else:
            return range < MIN_DISTANCE

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
        self.xyz = np.array([data[self.pos_var_list[0]], -data[self.pos_var_list[1]], data[self.pos_var_list[2]]])
        self.xyz_global = self.xyz + self.xyz0 # Position in the global frame
        self.pry = np.array([data[self.pos_var_list[3]], data[self.pos_var_list[4]], data[self.pos_var_list[5]]])

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

    def move_to_landing_zone(self):
        keep_flying = True

        VELOCITY = 0.3
        MIN_Y = 0.5
        MAX_DISTANCE = 2
        velocity_x = 0.0
        velocity_y = 0.0

        if self.is_close_obs(self.range[0]):  # There is an obstacle in front
            #print("Front : ", self.range[0])
            if self.xyz[1] < MIN_Y:
                velocity_x = 0.0
                velocity_y = 2 * VELOCITY

            else:
                velocity_x = 0.0
                velocity_y = -2 * VELOCITY

        else:  # If no obstacle, go forward
            #print("Straight")
            velocity_x = VELOCITY
            velocity_y = 0.0

        if (self.xyz[0] > MAX_DISTANCE):
            keep_flying = False
            velocity_x = 0.0
            velocity_y = 0.0

            # if (measured_z < 0.25 and measured_x > 0.5):
            #    keep_flying = False
            # print(measured_z)

            # to be removed maybe...
        if self.is_close_obs(self.range[2]):
            keep_flying = False

        self.xyz_rate_cmd = [velocity_x, velocity_y, 0]

        return keep_flying

#----------------------------------------------------------------------------------------#

    def set_waypoints(self):
        """
        Create a list of waypoints in the GLOBAL FRAME to search the platform
                            W
        ##########################################
        #                   L               h    #
        # l   ------------------------------   l #
        #                                   |    #
        #                                   | H  #
        #                                   |    #
        #     ------------------------------     # H3
        #    |                                   #
        #    | H                                 #
        #    |            L                      #
        #  l  ------pi-----------------------  l #
        #                                  h     #
        ##########################################
        """
        # When we enter this function, drone is at pi

        self.waypoints = np.array([])

        # Start en bas à gauche : P(x0, l, 0.5) -> On assume que xyz_global[1] > W/2
        self.waypoints = np.append(self.waypoints, [self.xyz_global[0], self.l, self.default_height])

        for i in range(self.N-1):
            self.waypoints = np.append(self.waypoints, self.waypoints[6*i:6*i+3] + np.array([self.H, 0, 0]))
            self.waypoints = np.append(self.waypoints, self.waypoints[6*i+3:6*i+6] + np.array([0, self.L * (-1)**i, 0]))

        # Correct starting direction
        if self.xyz_global[1] < self.playground.W / 2:
            # Mirroir + décalage de 2*l + L
            for i in range(int(len(self.waypoints)/3)):
                self.waypoints[3*i+1] = -self.waypoints[3*i+1] + 2*self.l + self.L
                

#----------------------------------------------------------------------------------------#

    def follow_waypoints(self):
        # Min distance to consider point as reached
        epsilon = 0.05 # m
        modulus_error = np.sum((self.waypoints[0:3] - self.xyz_global)**2)
        print(modulus_error)
        
        # Check if the waypoint has been reached
        if modulus_error < epsilon**2:
            # If yes, check if it was the last waypoint in the list
            print("Next Waypoint")
            if len(self.waypoints) == 3:
                # If yes stop the search
                self.waypoints = None
                
                return False
  
            # If No, then remove the first waypoint from the list
            self.waypoints = self.waypoints[3:len(self.waypoints)]

        current_waypoint = self.waypoints[0:3]

        # Compute a command to reach this waypoint
        error = current_waypoint - self.xyz_global
        #print(error)
        
        kp = 1
        MAX_SPEED = 0.1

        # Compute Speed command of the drone
        self.xyz_rate_cmd = kp * error
        
        # Limit the speed of each component xyz
        for i in range(3):
            if self.xyz_rate_cmd[i] > MAX_SPEED:
                self.xyz_rate_cmd[i] = MAX_SPEED

            if self.xyz_rate_cmd[i] < -MAX_SPEED:
                self.xyz_rate_cmd[i] = -MAX_SPEED
        
        
        return True

#----------------------------------------------------------------------------------------#

    def stateMachine(self, scf):
        with MotionCommander(scf, default_height = self.default_height) as mc:
            while(self.is_not_close()):
                #print(self.range[2])
                if self.state == 0:

                    #---- Take off ----#

                    # default height has been reached -> Next state
                    if self.xyz[2] >= self.default_height:
                        self.state += 1
                        #print("Next state : " + str(self.state))

                elif self.state == 1:

                    #---- Fly to zone 2 ----#
                    
                    # self.range = [front, back, up, left, right, zrange]
                    #print("Front main : ", self.range[0])
                    #print("Left : ", self.range[3])
                    #print("Right : ", self.range[4])
                    #print("Up : ", self.range[2])

                    #keep_flying = self.move_to_landing_zone()
                    keep_flying = False
                    
                    if not keep_flying:
                        print('Safe arrival in Landing zone ! Let the scan begin')
                        keep_searching = True
                        self.state += 1
                        #print("Next state : " + str(self.state))

                elif self.state == 2:

                    #---- Search landing zone ----#
                    if self.waypoints is None and keep_searching == True:
                        # Wait to compute waypoints (searching path)
                        self.xyz_rate_cmd = np.array([0, 0, 0])
                        self.set_waypoints()
                        print("Setting waypoints")

                    # Return true if we reached last waypoint, false otherwise
                    keep_searching = self.follow_waypoints()

                    if not keep_searching:
                        self.state += 1
                        self.waypoints = None
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
                #print(self.xyz_rate_cmd[0])
                
                mc.start_linear_motion(self.xyz_rate_cmd[0], -self.xyz_rate_cmd[1], self.xyz_rate_cmd[2], self.rpy_rate_cmd[0])

                time.sleep(self.Te_loop)

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
    
