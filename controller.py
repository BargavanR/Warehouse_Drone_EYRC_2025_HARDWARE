#!/usr/bin/env python3

"""
Controller for the drone
"""

# standard imports
import copy
import time

# third-party imports
import scipy.signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool



MIN_ROLL = 1250
BASE_ROLL = 1500
MAX_ROLL = 1600
SUM_ERROR_ROLL_LIMIT = 10000

IN_PITCH = 1250
BASE_PITCH = 1500
MAX_PITCH = 1600
SUM_ERROR_PITCH_LIMIT = 10000

MIN_THROTTLE = 1250
BASE_THROTTLE = 1500
MAX_THROTTLE = 1600
SUM_ERROR_THROTTLE_LIMIT = 10000


DRONE_WHYCON_POSE = [[], [], []]

# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch

class DroneController():
    def __init__(self,node):
        self.node= node
        
        self.rc_message = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = 0
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"

        self.arming_service_client = self.node.create_client(CommandBool,service_endpoint)
        self.set_points = [0, 0, 22]         # Setpoints for x, y, z respectively      
        self.out_roll = 1500
        self.out_pitch = 1500
        self.out_throttle = 1500
        self.last_time = 0.0
        self.loop_time = 0.032
        self.error = [0, 0, 0]         # Error for roll, pitch and throttle        
        self.error_x=0.0
        self.error_y=0.0
        self.error_z=0.0
        # Create variables for integral and differential error
        self.integral = [0, 0, 0]
        self.differential = [0, 0, 0]
        # Create variables for previous error and sum_error
        self.prev_error = [ 0, 0 ,0 ]
        self.sum_error = [ 0, 0, 0 ]

        self.Kp = [ 0 * 0.01  , 0 * 0.01  , 0 * 0.01  ]
        self.Ki = [ 0 * 0.01  , 0 * 0.01  , 0 * 0.01  ]
        self.Kd = [ 0 * 0.01  , 0 * 0.01  , 0 * 0.01  ]
        # Similarly create variables for Kd and Ki

        # Create subscriber for WhyCon 
        
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        
        # Similarly create subscribers for pid_tuning_altitude, pid_tuning_roll, pid_tuning_pitch and any other subscriber if required
       
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_throttle_callback,1)
        self.pid_roll = node.create_subscription(PidTune,"/pid_tuning_roll",self.pid_tune_roll_callback,1)
        self.pid_pitch = node.create_subscription(PidTune,"/pid_tuning_pitch",self.pid_tune_pitch_callback,1)
        # Create publisher for sending commands to drone 

        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)

        # Create publisher for publishing errors for plotting in plotjuggler 
        
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)        


    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.drone_whycon_pose_array = msg

        self.whycon_x = self.drone_whycon_pose_array.poses[0].position.x
        self.whycon_y = self.drone_whycon_pose_array.poses[0].position.y
        self.whycon_z = self.drone_whycon_pose_array.poses[0].position.z



        # self.node.get_logger().info("Received WHYCON poses at: {}".format(self.last_whycon_pose_received_at))
        # if self.drone_whycon_pose_array.poses:
        #     self.node.get_logger().info("Number of poses received: {}".format(len(self.drone_whycon_pose_array.poses)))
        #     self.node.get_logger().info("poses: {}".format(self.drone_whycon_pose_array.poses[0].position.x))

    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.kp * 0.01
        self.Ki[2] = msg.ki * 0.0001
        self.Kd[2] = msg.kd * 0.1

    def pid_tune_pitch_callback(self, msg):
        self.Kp[1] = msg.kp * 0.01
        self.Ki[1] = msg.ki * 0.0001
        self.Kd[1] = msg.kd * 0.1

    def pid_tune_roll_callback(self, msg):
        self.Kp[0] = msg.kp * 0.01
        self.Ki[0] = msg.ki * 0.0001
        self.Kd[0] = msg.kd * 0.1

    # Similarly add callbacks for other subscribers


    def roll(self): #x-axis
        try:
            # if self.drone_whycon_pose_array.poses:
            self.node.get_logger().info("hi")
            self.error_x = self.whycon_x- self.set_points[0]
            self.node.get_logger().info("error0: {}".format(self.error_x))
            self.integral[0] = (self.integral[0] + self.error_x)
            if self.integral[0] > SUM_ERROR_ROLL_LIMIT:
                self.integral[0] = SUM_ERROR_ROLL_LIMIT
            if self.integral[0] < -SUM_ERROR_ROLL_LIMIT:
                self.integral[0] = -SUM_ERROR_ROLL_LIMIT
            self.differential[0] = ((self.error_x-self.prev_error[0])/self.current_time)
            self.out_roll = (self.Kp[0]*0.005)*(self.error_x)+(self.Kd[0]*0.005)*(self.differential[0])+(self.Ki[0]*0.001)*(self.integral[0])
            self.out_roll += 1490

            self.prev_error[0] = self.error_x


            # else:
            #     self.node.get_logger().info("whycon poses is empty")


        except Exception as e:
            self.node.get_logger().error(f"error in calc pid :{str(e)}")


    def pitch(self): #y-axis
        try:
            #if self.drone_whycon_pose_array.poses:            

            self.error_y = self.whycon_y - self.set_points[1]
            self.node.get_logger().info("error1 : {}".format(self.error_y))
            self.integral[1] = (self.integral[1] + self.error_y)
            if self.integral[1] > SUM_ERROR_PITCH_LIMIT:
                self.integral[1] = SUM_ERROR_PITCH_LIMIT
            if self.integral[1] < -SUM_ERROR_PITCH_LIMIT:
                self.integral[1] = -SUM_ERROR_PITCH_LIMIT
            self.differential[1] = ((self.error_y-self.prev_error[1])/self.current_time)
            self.out_pitch = 1490 - ((self.Kp[1]*0.005)*(self.error_y)+(self.Kd[1]*0.005)*(self.differential[1])+(self.Ki[1]*0.001)*(self.integral[1]))
            # self.out_pitch += 1500

            self.prev_error[1] = self.error_y
            # else:
            #     self.node.get_logger().info("whycon poses is empty")


        except Exception as e:
            self.node.get_logger().error(f"error in calc pid :{str(e)}")

    def throt(self):
        try:
            #if self.drone_whycon_pose_array.poses:

            self.error_z = self.whycon_z - self.set_points[2]
            self.node.get_logger().info("error2 : {}".format(self.error_z))
            self.integral[2] = (self.integral[2] + self.error_z)
            if self.integral[2] > SUM_ERROR_THROTTLE_LIMIT:
                self.integral[2] = SUM_ERROR_THROTTLE_LIMIT
            if self.integral[2] < -SUM_ERROR_THROTTLE_LIMIT:
                self.integral[2] = -SUM_ERROR_THROTTLE_LIMIT
            self.differential[2] = ((self.error[2]-self.prev_error[2])/self.current_time)
            self.out_throttle = 1490 - ((self.Kp[2]*0.005)*(self.error_z)+(self.Kd[2])*(self.differential[2]*0.005)+ (self.Ki[2]*0.001)*(self.integral[2]))
            self.node.get_logger().info("out_throt :{}".format(self.out_throttle))
            # self.out_throttle += 1500
            self.node.get_logger().info("out throt: {}".format(type(self.out_throttle)))
            self.prev_error[2] = self.error_z
            # else:
            #     self.node.get_logger().info("whycon poses is empty")


        except Exception as e:
            self.node.get_logger().error(f"error in calc pid :{str(e)}")
    def calculate_pid(self):
        
        self.seconds = time.time()
        self.current_time = self.seconds - self.last_time
        self.node.get_logger().info("ct : {}".format(self.current_time))
        self.node.get_logger().info("lt : {}".format(self.loop_time))
        if self.current_time >= self.loop_time:
            self.roll()
            self.pitch()
            self.throt()

            self.last_time = self.seconds


    def pid(self):          # PID algorithm

        # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        try:
            if self.drone_whycon_pose_array.poses:
                self.node.get_logger().info("heyy")
                #return    
                self.calculate_pid()  
                self.node.get_logger().info("bye")      # Similarly calculate error for y and z axes 
        
        except Exception as e:
            self.node.get_logger().error(f"Error in pid function: {str(e)}")

        # Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)

        # self.integral[0] = (self.integral[0] + self.error[0])
        # if self.integral[0] > SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = SUM_ERROR_ROLL_LIMIT
        # if self.integral[0] < -SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = -SUM_ERROR_ROLL_LIMIT

        # Save current error in previous error

        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis

        # 2 : calculating Error, Derivative, Integral for Alt error : z axis


        # Write the PID equations and calculate the self.rc_message.rc_throttle, self.rc_message.rc_roll, self.rc_message.rc_pitch

        
    #------------------------------------------------------------------------------------------------------------------------

        self.publish_data_to_rpi( out_roll = 1490, out_pitch = 1490, out_throttle = 1490)

        #Replace the roll pitch and throttle values as calculated by PID 
        
        
        # Publish alt error, roll error, pitch error for plotjuggler debugging

        self.pid_error_pub.publish(
            PIDError(
                #roll_error=float(self.error_x),
                roll_error = 0.8,
                pitch_error=float(self.error_y),
                #pitch_error= 0.5,
                throttle_error=float(self.error_z),
                yaw_error=-0.8,
                zero_error=0.0,
            )
        )



    

    def publish_data_to_rpi(self, out_roll, out_pitch, out_throttle):

        self.rc_message.rc_throttle = max(1000, min(int(self.out_throttle), 2000))
        self.rc_message.rc_roll = max(1000, min(int(self.out_roll), 2000))
        self.rc_message.rc_pitch = max(1000,min(int(self.out_pitch),2000))

        # Send constant 1500 to rc_message.rc_yaw
        self.rc_message.rc_yaw = int(1450)

        #BUTTERWORTH FILTER
        span = 15
        for index, val in enumerate([self.out_roll, self.out_pitch, self.out_throttle]):
            DRONE_WHYCON_POSE[index].append(val)
            if len(DRONE_WHYCON_POSE[index]) == span:
                DRONE_WHYCON_POSE[index].pop(0)
            if len(DRONE_WHYCON_POSE[index]) != span-1:
                return
            order = 3
            fs = 60
            fc = 5
            nyq = 0.5 * fs
            wc = fc / nyq
            b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
            filtered_signal = scipy.signal.lfilter(b, a, DRONE_WHYCON_POSE[index])
            if index == 0:
                self.rc_message.rc_roll = int(filtered_signal[-1])
            elif index == 1:
                self.rc_message.rc_pitch = int(filtered_signal[-1])
            elif index == 2:
                self.rc_message.rc_throttle = int(filtered_signal[-1])

        if self.rc_message.rc_roll > 2000:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_roll = 2000
        elif self.rc_message.rc_roll < 1000:
            self.rc_message.rc_roll = 1000

        # Similarly add bounds for pitch yaw and throttle 
        if self.rc_message.rc_pitch > 2000:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_pitch = 2000
        elif self.rc_message.rc_pitch < 1000:
            self.rc_message.rc_pitch = 1000
        
        if self.rc_message.rc_throttle > 2000:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_throttle = 2000
        elif self.rc_message.rc_throttle < 1000:
            self.rc_message.rc_throttle = 1000
        # Similarly add bounds for pitch yaw and throttle 

        self.rc_pub.publish(self.rc_message)


    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C. 
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop 

    def shutdown_hook(self):
        self.node.get_logger().info("Calling shutdown hook")
        self.disarm()

    # Function to arm the drone 

    def arm(self):
        self.node.get_logger().info("Calling arm service")
        self.commandbool.value = True
        self.future = self.arming_service_client.call_async(self.commandbool)

    # Function to disarm the drone 

    def disarm(self):

        # Create the disarm function
        self.commandbool.value = False
        self.future = self.arming_service_client.call_async(self.commandbool)
        pass


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('controller')
    node.get_logger().info(f"Node Started")
    node.get_logger().info("Entering PID controller loop")

    controller = DroneController(node)
    controller.arm()
    node.get_logger().info("Armed")

    try:
        while rclpy.ok():
            controller.pid()
            if node.get_clock().now().to_msg().sec - controller.last_whycon_pose_received_at > 1:
                node.get_logger().error("Unable to detect WHYCON poses")

            #else:
                #node.get_logger().info("detected whycon")
            rclpy.spin_once(node) # Sleep for 1/30 secs
        

    except Exception as err:
        print(err)

    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()