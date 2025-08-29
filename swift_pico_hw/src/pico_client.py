#!/usr/bin/env python3
'''
# Team ID:          1154
# Theme:            Warehouse Drone (WD)
# Author List:      R Bargavan,P Sabarishh,Prasanth kanaga Sabai S,Dhasrad Raghav A B.
# Filename:         swift_pico/src/pico_client.py
# Functions:        send_goal,get_result_callback,get_result_callback,feedback_callback,send_request,receive_goals
# Global variables: None
'''
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.action import ActionServer
from  waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints
from geometry_msgs.msg import Pose  
#import the action and service done ig



class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.goals = []
        self.goal_index = 0
        self.action_client =ActionClient(self, NavToWaypoint ,'waypoint_navigation') 
        #create an action client for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #action name should 'waypoint_navigation'. done 

        self.cli =self.create_client(GetWaypoints,'waypoints')
        #create a client for the service 'GetWaypoints'. Refer to Writing a simple service and client (Python) in ROS 2 tutorials
        #service name should be 'waypoints' done 
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = GetWaypoints.Request()
        #create a request object for GetWaypoints service.
        

    
    ###action client functions

    def send_goal(self, waypoint): 

        #create a NavToWaypoint goal object.
        goal_msg =NavToWaypoint.Goal()
        #goal_msg.waypoint = waypoint
        #self.get_logger().info('Waiting for action server to become available...')
       
        #self.get_logger().info('Action server is now available.')


        goal_msg.waypoint.position.x = waypoint[0]
        goal_msg.waypoint.position.y = waypoint[1]
        goal_msg.waypoint.position.z = waypoint[2]

        #create a method waits for the action server to be available. done ig
        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)    
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    '''  def send_goal(self, waypoint): 
            goal_msg = NavToWaypoint.Goal()

            # Create a Pose message and set the coordinates
            pose = Pose()
            pose.position.x = waypoint[0]
            pose.position.y = waypoint[1]
            pose.position.z = waypoint[2]

            # Assign the Pose object to goal_msg.waypoint
            goal_msg.waypoint = pose

            self.action_client.wait_for_server()
            self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)    
            self.send_goal_future.add_done_callback(self.goal_response_callback)'''
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected Sorryyyy')
            return

        self.get_logger().info('Hurrayyy! Goal accepted ')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        

        #complete the goal_response_callback. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #done ig
        

    def get_result_callback(self, future):

        #complete the missing line done ig
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.hov_time))

        self.goal_index += 1

        if self.goal_index < len(self.goals):
            self.send_goal(self.goals[self.goal_index])
        else:
            self.get_logger().info('All waypoints have been reached successfully')      

    def feedback_callback(self, feedback_msg):

        #complete the missing line  done 
        feedback = feedback_msg.feedback
        x = feedback.current_waypoint.pose.position.x
        y = feedback.current_waypoint.pose.position.y
        z = feedback.current_waypoint.pose.position.z
        t = feedback.current_waypoint.header.stamp.sec
        #self.get_logger().info(f'Received feedback! The current whycon position is: {x}, {y}, {z}')
        #self.get_logger().info(f'Max time inside sphere: {t}')


    #service client functions

    def send_request(self):
        request = GetWaypoints.Request()
        request.get_waypoints = True  
        future = self.cli.call_async(request)
        return future

        #done ig
        #  complete send_request method, which will send the request and return a future
    
    def receive_goals(self):
        future = self.send_request()
        #write a statement to execute the service until the future is complete done ig
        
        # Wait for the service to complete
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        self.get_logger().info('Waypoints received by the action client')

        for pose in response.waypoints.poses:
            waypoints = [pose.position.x, pose.position.y, pose.position.z]
            self.goals.append(waypoints)
            self.get_logger().info(f'Waypoints: {waypoints}')

        self.send_goal(self.goals[0])
    

def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WayPointClient()
    waypoint_client.receive_goals()

    try:
        rclpy.spin(waypoint_client)
    except KeyboardInterrupt:
        waypoint_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        rclpy.shutdown()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
