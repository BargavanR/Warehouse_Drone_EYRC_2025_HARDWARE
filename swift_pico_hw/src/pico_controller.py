#!/usr/bin/env python3

#antiwindup       //need to add
# keyboard interrupt  // added

'''                                                   
# Team ID:          1154
# Theme:            Warehouse Drone (WD)
# Author List:      R Bargavan.
# Filename:         pico_ws2/swift_pico_hw/src/pico_controller.py
    
# Functions:        main()
                    Class Swift_Pico -> __init__ , send_arm_request , send_disarm_request , whycon_callback ,
                                        altitude_set_pid , roll_set_pid , pitch_set_pid , pid , publish_filtered_data
# Global variables: MIN_ROLL , CMD , SUM_ERROR_THROTTLE_LIMIT , MAX_THROTTLE , BASE_THROTTLE , MIN_THROTTLE , SUM_ERROR_PITCH_LIMIT , MAX_PITCH,
                    BASE_PITCH , MIN_PITCH , SUM_ERROR_ROLL_LIMIT , MAX_ROLL , BASE_ROLL

                    


This python file runs a ROS 2-node of name pico_controller which holds the position of Swift Pico Drone at the height of 26 in Real World. - Arena has been prepared .
This node publishes and subsribes the following topics:

        PUBLICATIONS			SUBSCRIPTIONS       SERVICES                TIMER
        /drone/rc_command		/whycon/poses       /drone/cmd/arming       PID()
        /pid_error				/throttle_pid
                                /pitch_pid
                                /roll_pid
'''
    #-------------------------------------------------------------------------LIBRARIES-------------------------------------------------------------------------------------------
import time
# Importing the required libraries
import scipy.signal          #Used in Butterworth Filter. Design and apply digital and analog filters (e.g., low-pass, high-pass, band-pass).
import numpy as np           #not used this yet
from rc_msgs.msg import RCMessage        
#To send the RC(RealCraft) msgs from receiver to drone   
# (i.e) Sending throttle = 1500 rc values to drone
from rc_msgs.srv import CommandBool      
#This one is Used to Send the arming commands ,to arm the drone using false and true commands.
from geometry_msgs.msg import PoseArray   
# This one is used to provide a path to receive the Position From the whycon output. [x,y,z] 
# ^ Camera calibration matrix is used to determine the x,y,z co ordinates using whycon. 
#   and those poses are published by whycon/poses via POSEARRAY
from pid_msg.msg import PIDTune, PIDError  #PID error Publishes The error values to a topic '/pid_error' which is used to determine the drones 
#pid error and uses pid algorthim , PID basically produces an GUI in which we can change the Kp Ki Kd gains of roll,pitch ,and throttle .
#Publishes it to change the Kp,ki ,kd gains while running. 
import rclpy                 #this module contains all the build in functions to integrate ROS and Python
from rclpy.node import Node  #This is used to make this python file as a ROS NODE to get used by other ros functionalities/. 

    #-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #-----------------------------------Creating Global Constants so that we can easily use it in butterworth filter

MIN_ROLL = 1200
BASE_ROLL = 1500
MAX_ROLL = 1700
SUM_ERROR_ROLL_LIMIT = 5000

MIN_PITCH = 1200
BASE_PITCH = 1500
MAX_PITCH = 1700
SUM_ERROR_PITCH_LIMIT = 5000

MIN_THROTTLE = 1250
BASE_THROTTLE = 1500
MAX_THROTTLE = 2000
SUM_ERROR_THROTTLE_LIMIT = 5000

CMD = [[], [], []]

    #-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # initializing ros node with name pico_controller
        #------------------------------Drone_position and Set_Position ------------------------------------

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0, 0.0, 0.0]
        
        # [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = [0, 0, 26]  
        #   above value will be shown in whycon marker at the position where the drone needs to be in the scene.
        #   Make the whycon marker associated with position_to_hold renderable and make changes accordingly
        #---------------------------------------------------------------------------------------------------
        #---------------------------------------RC_commands-------------------------------------------------

        # Declaring a cmd of message type rc_msgs and initializing values
        #These values are general used for intitial rise of the drone . Anyways these values get Replaced by pid function and butterworth flter 
        # except YAW, everything changes with respect to the PID equation. 
        self.cmd = RCMessage()
        self.cmd.rc_roll = 1500     #initial rc_command for Roll 
        self.cmd.rc_pitch = 1500    #initial rc_command for Pitch
        self.cmd.rc_yaw = 1500      #initial rc_command for Throttle 
        self.cmd.rc_throttle = 1500 #initial rc_command for Yaw -> this one is constant as this wasn't used in PID equation.  
        #self.last_time = 0
        self.commandbool = CommandBool.Request()
        service_endpoint= "/drone/cmd/arming"
        self.arming_service_client = self.create_client(CommandBool,service_endpoint)

        #---------------------------------------------------------------------------------------------------
        #----------------------------------------PID Gains-------------------------------------------------
        
        #initial setting of Kp, Kd and ki for [roll, pitch, throttle]. 
        #after tuning and computing corresponding PID parameters, change the parameters
        #PID gains are used in the PID equation to get the rc_cmd values , this PID gains are tuned using PIDTUNE and gets accumulated constantly while flying
        #After Finalizing the Proper PID Gains using GUI we are hard core the gains here.
        self.Kp = [0.0, 0.0, 7.0]
        self.Ki = [0.00, 0.0000, 0.261]
        self.Kd = [0,0, 101.40]

        #-----------------------other required variables for pid  ----------------------------------------------
        
        self.error =[0,0,0]         #error is used in pid equation in which it gets calculated as Drone_current_pos - Drone_setPoint 
        self.sum_error =[0,0,0]     #Sum_error is Sum of all the error occuring in that itertion period.
        self.prev_error = [0,0,0]   #Prev_error stores the previous error to get used in diff_error.
        self.diff_error = [0,0,0]   #diff_error used as the diff in current error and prev_error in one iteration 

        #----------------------------------------------------------------------------------------------------------

        #This is the sample time in which PID runs.
        self.sample_time = 0.060  # in seconds
        
        # -------------------------------------Publishers------------------------------------------------------------

        self.command_pub = self.create_publisher(RCMessage, '/drone/rc_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        self.pid_error = PIDError()

        #-------------------------------------------------------------------------------------------------------------
    

        # ----------------------------------------Subscribers--------------------------------------------------------
         
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune,"/roll_pid",self.roll_set_pid,1)
        self.create_subscription(PIDTune, "/pitch_pid",self.pitch_set_pid,1)

        #------------------------------------------------------------------------------------------------------------
        
        #------------------------------------------Service call for Arming and Disarming-----------------------------
        #self.get_logger.info(f"{self.hehe}")
        #arm/disarm service client
        self.cli = self.create_client(CommandBool, "/drone/cmd/arming")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting again,,,,')
        self.req = CommandBool.Request()

        #------------------------------------------DISARM------------------------------------------------------------
        
        future = self.send_disarm_request() # DISARMING THE DRONE
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(response.data)
        
        #--------------------------------------------------------------------------------------------------------------

        #------------------------------------------ARM-----------------------------------------------------------------
    
        future = self.send_arm_request() # ARMING THE DRONE
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(response.data)

        #--------------------------------------------------------------------------------------------------------------
        
        #----------------------------------PID_TIMER ------------------------------------------------------------------   
 
        # Create a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)
        self.timer =self.create_timer(self.sample_time,self.pid)

        #---------------------------------------------------------------------------------------------------------------
        #END of __init__

        #--------------------------------- Drone_arm_and_disarm_req_functions--------------------------------------------     

    def send_arm_request(self):
        self.req.value = True
        return self.cli.call_async(self.req)

    def send_disarm_request(self):
        self.req.value =False
        return self.cli.call_async(self.req)
    
        #------------------------------------------------------------------------------------------------------------------
        #-------------------------------------------------- Whycon callback function---------------------------------------------

    # The function gets executed each time when /whycon node publishes /whycon/poses 
    def whycon_callback(self, msg):
        self.whycon_last_came_at = self.get_clock().now().seconds_nanoseconds()[0]
        self.drone_position[0] = msg.poses[0].position.x 
        self.drone_position[1] = msg.poses[0].position.y 
        self.drone_position[2] = msg.poses[0].position.z
    
        #-------------------------------------------------------------------------------------------------------------------------


        #--------------------------------------------------PID_gains_callback_function from PIDTUNE() to PID()-------------------    
 
    # Callback function for /throttle_pid
    # This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.01  # the ratio/fraction  is just for an example to multiple the consts
        self.Ki[2] = alt.ki * 0.001
        self.Kd[2] = alt.kd * 0.1
    def roll_set_pid(self, roll):
        self.Kp[0] =roll.kp * 0.01
        self.Ki[0] =roll.ki * 0.001
        self.Kd[0] =roll.kd * 0.1

    def pitch_set_pid(self,pitch):
        self.Kp[1] =pitch.kp * 0.01
        self.Ki[1] =pitch.ki * 0.001
        self.Kd[1] =pitch.kd * 0.1

        #-------------------------------------------------------------------------------------------------------------------------

        #-------------------------------------Publishes Filtered rc commands to drone(ButterworthFilter) -------------------------

    def publish_filtered_data(self, roll, pitch, throttle):
        try:
            self.cmd.rc_throttle = int(throttle)
            self.cmd.rc_roll = int(1470) #int(roll)
            self.cmd.rc_pitch = int(1470) #int(pitch)
            self.cmd.rc_yaw = int(1500)


            # BUTTERWORTH FILTER low pass filter
            span = 15
            for index, val in enumerate([roll, pitch, throttle]):
                CMD[index].append(val)
                if len(CMD[index]) == span:
                    CMD[index].pop(0)
                if len(CMD[index]) != span-1:
                    return
                order = 3 # determining order 
                fs = 30 # to keep in order same as hz topic runs
                fc = 4 
                nyq = 0.5 * fs
                wc = fc / nyq
                b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
                filtered_signal = scipy.signal.lfilter(b, a, CMD[index])
                if index == 0:
                    self.cmd.rc_roll = int(filtered_signal[-1])
                    self.get_logger().info(f"PID roll from filter  is {self.cmd.rc_roll}")
                elif index == 1:
                    self.cmd.rc_pitch = int(filtered_signal[-1])
                    self.get_logger().info(f"PID pitch from filter is {self.cmd.rc_pitch}")
                elif index == 2:
                    self.cmd.rc_throttle = int(filtered_signal[-1])
                    self.get_logger().info(f"PID throttle from filter is {self.cmd.rc_throttle}")
                if self.cmd.rc_roll > MAX_ROLL:     #checking range i.e. bet 1000 and 2000
                    self.cmd.rc_roll = MAX_ROLL
                elif self.cmd.rc_roll < MIN_ROLL:
                    self.cmd.rc_roll = MIN_ROLL

                # Similarly add bounds for pitch yaw and throttle 
                if self.cmd.rc_throttle > MAX_THROTTLE:
                    self.cmd.rc_throttle = MAX_THROTTLE
                elif self.cmd.rc_throttle < MIN_THROTTLE:
                    self.cmd.rc_throttle = MIN_THROTTLE

                if self.cmd.rc_pitch > MAX_PITCH:
                    self.cmd.rc_pitch = MAX_PITCH
                elif self.cmd.rc_pitch < MIN_PITCH:
                    self.cmd.rc_pitch = MIN_PITCH

            self.command_pub.publish(self.cmd)
        except Exception as e :
            self.get_logger().error(f"Error in filter : {str(e)}")
        #--------------------------------------------------------------------------------------------------------------------

        #-----------------------------------------------PID_algorithm-------------------------------------------------------
    
    def pid(self):
        try:  

            #self.seconds = time.time()
            #self.current_time = self.seconds - self.last_time
            #self.get_logger().info(f"Current_time :{self.current_time}")
            for i in range(3):	
                self.error[i]=self.drone_position[i]- self.setpoint[i]
                self.sum_error[i] = self.sum_error[i] + self.error[i]
                self.diff_error[i] =(self.error[i] - self.prev_error[i]) #/self.current_time
                self.prev_error[i] = self.error[i]
                #self.last_time = self.current_time

            #restricting Sum error for roll 
            if self.sum_error[0]> SUM_ERROR_ROLL_LIMIT:
                self.sum_error[0]=SUM_ERROR_ROLL_LIMIT
            if self.sum_error[0]< -SUM_ERROR_ROLL_LIMIT:
                self.sum_error[0]= SUM_ERROR_ROLL_LIMIT
            #restricting Sum error for pitch
            if self.sum_error[1]> SUM_ERROR_PITCH_LIMIT:
                self.sum_error[1]=SUM_ERROR_PITCH_LIMIT
            if self.sum_error[1]< -SUM_ERROR_PITCH_LIMIT:
                self.sum_error[1]= SUM_ERROR_PITCH_LIMIT
            #restricting Sum error for throttle
            if self.sum_error[2]> SUM_ERROR_THROTTLE_LIMIT:
                self.sum_error[2]=SUM_ERROR_THROTTLE_LIMIT
            if self.sum_error[2]< -SUM_ERROR_THROTTLE_LIMIT:
                self.sum_error[2]= SUM_ERROR_THROTTLE_LIMIT
                    

            self.pid_error.throttle_error =self.error[2]
            self.pid_error.roll_error =self.error[0]
            self.pid_error.pitch_error =self.error[1]
        
            self.throttle = ((self.Kp[2]* 0.5) * self.error[2]) + ((self.Ki[2]*0.1) * self.sum_error[2]) +((self.Kd[2]*0.5) * self.diff_error[2])
            self.roll = ((self.Kp[0]*0.5) * self.error[0]) + ((self.Ki[0]*0.1) * self.sum_error[0]) +((self.Kd[0]*0.5) * self.diff_error[0])
            self.pitch = ((self.Kp[1]*0.5) * self.error[1]) + ((self.Ki[1]*0.1) * self.sum_error[1]) +((self.Kd[1]*0.5) * self.diff_error[1])
            
        # self.get_logger().info(f"last_time :{self.last_time}")
        
            '''self.cmd.rc_roll = int(max(self.min[0], min(self.max[0], 1500 - self.roll)))
            self.cmd.rc_pitch = int(max(self.min[1], min(self.max[1], 1500 + self.pitch)))
            self.cmd.rc_throttle= int(max(self.min[2], min(self.max[2], 1500 + self.throttle)))
    '''
            self.throttle_out   = int(1470 + self.throttle)
            self.roll_out       = int(1470 - self.roll)
            self.pitch_out      = int(1470 + self.pitch)
            self.get_logger().info(f"PID OUT VALUES TO FILTER ROLL PITCH THROT = [{self.roll_out},{self.pitch_out},{self.throttle_out}]")
        #------------------------------------------------------------------------------------------------------------------------
          
            #Sending the roll, pitch,throttle to butterWorth Filter and publishing it after filtering. 
            self.publish_filtered_data(roll = self.roll_out,pitch = self.pitch_out,throttle = self.throttle_out)

            # calculate throttle error, pitch error and roll error, then publish it accordingly. This used for Viewing in Rqt Graph.
            self.pid_error_pub.publish(self.pid_error)
        except Exception as e:
            self.get_logger().error(f"Error in pid algoritm {str(e)}")
    #-----------------------------------------------
    def shutdown_hook(self):
        self.get_logger().info("Calling shutdown hook")
        self.disarm()
    def disarm(self):
        # Create the disarm function
        self.commandbool.value = False
        self.future = self.arming_service_client.call_async(self.commandbool)
        

def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    try:
        rclpy.spin(swift_pico)
    except Exception as e:
        swift_pico.get_logger().error(f"enna da achu , itha achu - > {str(e)}")
        swift_pico.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        #rclpy.spin_once(swift_pico)
        swift_pico.shutdown_hook()
        swift_pico.destroy_node()
        rclpy.shutdown()
        swift_pico.get_logger().info('Disarming the drone...')
        


if __name__ == '__main__':
    main()