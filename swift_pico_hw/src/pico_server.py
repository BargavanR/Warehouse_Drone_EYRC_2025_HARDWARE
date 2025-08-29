#!/usr/bin/env python3
'''
# Team ID:          1154
# Theme:            Warehouse Drone (WD)
# Author List:      R Bargavan,P Sabarishh,Prasanth kanaga Sabai S,Dhasrad Raghav A B.
# Filename:         swift_pico/src/pico_controller.py
# Functions:        disarm,arm,whycon_callback,altitude_set_pid,roll_set_pid,pitch_set_pid,pid,odometry_callback,execute_callback
# Global variables: None
'''

import time
import math
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

#from std_msgs.msg import Int32
#import the action done ig

#pico control specific libraries
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
from nav_msgs.msg import Odometry

class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.duration = 0


        self.drone_position = [0.0, 0.0, 0.0, 0.0]
        self.setpoint = [0, 0, 27, 0] 
        self.dtime = 0

        self.cmd = RCMessage()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
       
        
        #Kp, Ki and Kd values here
# Example adjusted gains
        self.Kp = [0,0,0,0]     # Small increase for throttle
        self.Ki = [0.0, 0.0, 0.0, 0.0]        # Slightly reduce Ki for throttle only
        self.Kd = [0,0,0,0] # Reduce throttle Kd slightly to reduce oscillations


        

        #variables for storing different kinds of errors done ig 
        self.error=[0,0,0,0]
        self.sum_error =[0,0,0,0]
        self.diff_error =[0,0,0,0]
        self.prev_error =[0,0,0,0]
        #self.filtered_error = [0, 0, 0, 0]  # Initialize for throttle, roll, pitch, yaw
        #self.alpha = 0.2 # Define your alpha for the filter (tuning needed)


        self.min=[1000,1000,1000,1000]
        self.max=[2000,2000,2000,2000]
        #self.current_time=[0,0,0] #Experiment using some current times for derivative

        self.pid_error = PIDError()

        self.sample_time = 0.060

        self.command_pub = self.create_publisher(RCMessage, '/drone/rc_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune,"/roll_pid",self.roll_set_pid,1)
        self.create_subscription(PIDTune, "/pitch_pid",self.pitch_set_pid,1)
        #self.create_subscription(PIDTune, "/yaw_pid",self.yaw_set_pid,1)
        #Add other sunscribers here done ig

        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10)
        
        self._action_server =ActionServer(self, NavToWaypoint ,'waypoint_navigation',self.execute_callback,callback_group=self.action_callback_group) 
        #create an action server for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials done ig
        #action name should 'waypoint_navigation'.
        #include the action_callback_group in the action server. Refer to executors in ROS 2 concepts
       # self.action_callback_group = MutuallyExclusiveCallbackGroup()
        '''self._action_server = ActionServer(
            self,
            WayPointServer,
            'WayPointServer',
            self.execute_callback,
            callback_group=self.action_callback_group 
            ) #idk if this is crct
            
        '''
        
        #arm/disarm service client
        self.cli = self.create_client(CommandBool, "/drone/cmd/arming")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting again,,,,')
        self.req = CommandBool.Request()



        future = self.send_request() # ARMING THE DRONE
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(response.data)

        self.arm()

        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.pid_callback_group)

    def send_request(self):
        self.req.value = True
        return self.cli.call_async(self.req)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)


    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)


    def whycon_callback(self, msg):
            self.drone_position[0] = msg.poses[0].position.x
            self.drone_position[1] = msg.poses[0].position.y 
            self.drone_position[2] = msg.poses[0].position.z
            #Set the remaining co-ordinates of the drone from msg done ig
            self.dtime = msg.header.stamp.sec

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.001
        self.Kd[2] = alt.kd * 6

    def roll_set_pid(self, roll):
        self.Kp[0] =roll.kp * 0.03
        self.Ki[0] =roll.ki * 0.001
        self.Kd[0] =roll.kd * 6

    def pitch_set_pid(self,pitch):
        self.Kp[1] =pitch.kp * 0.03
        self.Ki[1] =pitch.ki * 0.001
        self.Kd[1] =pitch.kd * 6

    '''def yaw_set_pid(self,yaw):
        self.Kp[3] =yaw.kp * 0.03
        self.Ki[3] =yaw.ki * 0.001
        self.Kd[3] =yaw.kd * 6
    '''

    #Define callback function like altitide_set_pid to tune pitch, roll done 


    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        self.yaw_deg = math.degrees(yaw)
        self.drone_position[3] = self.yaw_deg	

    def pid(self):
    #-----------------------------Write the PID algorithm here--------------------------------------------------------------

    # Steps:
    # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
    #	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
    #	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
    #	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
    #	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
    #	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
    #																														self.cmd.rcPitch = self.max_values[1]
    #	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
    #	8. Add error_sum
        
        for i in range(3):	
            self.error[i]=self.drone_position[i]- self.setpoint[i]
            self.sum_error[i] = self.sum_error[i] + self.error[i]
            self.diff_error[i] =self.error[i] - self.prev_error[i]
            self.prev_error[i] = self.error[i]

        self.pid_error.throttle_error =self.error[2]
        self.pid_error.roll_error =self.error[0]
        self.pid_error.pitch_error =self.error[1]

        self.throttle = self.Kp[2] * self.error[2] + self.Ki[2] * self.sum_error[2] +self.Kd[2] * self.diff_error[2]
        self.roll = self.Kp[0] * self.error[0] + self.Ki[0] * self.sum_error[0] +self.Kd[0] * self.diff_error[0]
        self.pitch = self.Kp[1] * self.error[1] + self.Ki[1] * self.sum_error[1] +self.Kd[1] * self.diff_error[1]


        self.cmd.rc_roll = int(max(self.min[0], min(self.max[0], 1500 - self.roll)))
        self.cmd.rc_pitch = int(max(self.min[1], min(self.max[1], 1500 + self.pitch)))
        self.cmd.rc_throttle= int(max(self.min[2], min(self.max[2], 1500 + self.throttle)))

        #------------------------------------------------------------------------------------------------------------------------
        self.command_pub.publish(self.cmd)
        # calculate throttle error, pitch error and roll error, then publish it accordingly
        self.pid_error_pub.publish(self.pid_error)



    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        self.setpoint[0] = goal_handle.request.waypoint.position.x
        self.setpoint[1] = goal_handle.request.waypoint.position.y
        self.setpoint[2] = goal_handle.request.waypoint.position.z
        self.get_logger().info(f'New Waypoint Set: {self.setpoint}')
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0
        self.duration = self.dtime

        feedback_msg =NavToWaypoint.Feedback()
        #feedback_msg.partial_sequence = [0, 1]

        #create a NavToWaypoint feedback object. Refer to Writing an action server and client (Python) in ROS 2 tutorials.
        #done ig
        #--------The script given below checks whether you are hovering at each of the waypoints(goals) for max of 3s---------#
        # This will help you to analyse the drone behaviour and help you to tune the PID better.
    
        while True:
            feedback_msg.current_waypoint.pose.position.x = self.drone_position[0]
            feedback_msg.current_waypoint.pose.position.y = self.drone_position[1]
            feedback_msg.current_waypoint.pose.position.z = self.drone_position[2]
            feedback_msg.current_waypoint.header.stamp.sec = self.max_time_inside_sphere

            goal_handle.publish_feedback(feedback_msg)

            drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle,0.8) #the value '0.4' is the error range in the whycon coordinates that will be used for grading. 
            #You can use greater values initially and then move towards the value '0.4'. This will help you to check whether your waypoint navigation is working properly. 

            if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        pass
            
            elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        self.point_in_sphere_start_time = self.dtime
                        #self.get_logger().info('Drone in sphere for 1st time')                        #you can choose to comment this out to get a better look at other logs

            elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
                        #self.get_logger().info('Drone in sphere')                                     #you can choose to comment this out to get a better look at other logs
                             
            elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        #self.get_logger().info('Drone out of sphere')                                 #you can choose to comment this out to get a better look at other logs
                        self.point_in_sphere_start_time = None
                        #self.time_inside_sphere = 0 #experiment for stability 


            if self.time_inside_sphere > self.max_time_inside_sphere:
                 self.max_time_inside_sphere = self.time_inside_sphere

            if self.max_time_inside_sphere >= 3:
                 break
                        

        goal_handle.succeed()
        self.error_sum = [0.0, 0.0, 0.0, 0.0]
        #done ig
        #create a NavToWaypoint result object. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        result =NavToWaypoint.Result()
        result.hov_time = self.dtime - self.duration #this is the total time taken by the drone in trying to stabilize at a point
        return result

    def is_drone_in_sphere(self, drone_pos, sphere_center, radius):
        return (
            (drone_pos[0] - sphere_center.request.waypoint.position.x) ** 2
            + (drone_pos[1] - sphere_center.request.waypoint.position.y) ** 2
            + (drone_pos[2] - sphere_center.request.waypoint.position.z) ** 2
        ) <= radius**2


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    
    try:
         executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
         waypoint_server.destroy_node()
         rclpy.shutdown()


if __name__ == '__main__':
    main()
