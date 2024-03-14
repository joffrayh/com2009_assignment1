#! /usr/bin/env python3
# search_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from my_tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow
import numpy as np

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        rospy.init_node("search_action_server")

        # Create a "simple action server" with a callback function, 
        # and start it [DONE]
        self.actionserver = actionlib.SimpleActionServer(
            "/toms_search", 
            SearchAction,
            self.action_server_launcher,
            auto_start=False
        )
        self.actionserver.start()

        # pull in some useful publisher/subscriber functions from
        # the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        rospy.sleep(2)

        ## Implement some checks on the "goal" input parameter(s) [DONE]
        success = True
        vel = goal.fwd_velocity # m/s
        dist = goal.approach_distance # m
        if vel > 0.26 or vel < 0:
            print("Invalid velocity!")
            success = False

        if dist < 0.2:
            success = False
            print("Invalid distance!")

        if not success:
            ## Abort the action server if an invalid goal
            # has been requested [DONE]
            self.result.total_distance_travelled = -1.0
            self.result.closest_object_angle = -1.0
            self.result.closest_object_distance = -1.0
            self.actionserver.set_aborted(self.result)
            return

        ## Print a message to indicate that the requested
        # goal is valid [DONE]
        print(f"Search goal received: fwd_vel = {vel:.2f} m/s, approach_distance = {dist:.2f} m.")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        self.yaw0 = self.tb3_odom.yaw
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.front_min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        self.left_arc_min = self.tb3_lidar.left_arc_min
        self.right_arc_min = self.tb3_lidar.right_arc_min

        ## Set the robot's forward velocity
        # (as specified in the "goal") [DONE]
        self.vel_controller.set_move_cmd(linear=vel, angular=0.0)

        ## Establish a conditional statement so that the  
        ## while loop continues as long as the distance to the closest object
        ## ahead of the robot is always greater than the "approach distance"
        ## (as specified in the "goal") [DONE]
        self.i = 1
        while True:
            # update LaserScan data:
            self.closest_object = self.tb3_lidar.front_min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            self.left_arc_min = self.tb3_lidar.left_arc_min
            self.right_arc_min = self.tb3_lidar.right_arc_min


            print('left arc min = ', self.left_arc_min)


            ## Publish a velocity command to make the robot
            ## start moving [DONE]
            self.vel_controller.publish()

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                ## Take appropriate action if the action is
                ## cancelled (pre-empted) [DONE]
                print("Pre-empt requested!")
                self.actionserver.set_preempted(self.result)
                self.vel_controller.stop()
                success = False
                # exit the loop:
                break

            # determine how far the robot has travelled so far:
            self.distance = sqrt(
                pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2)
            )


            if self.closest_object < 0.5:
                rospy.loginfo("Obstacle detected! Rotating right to avoid.")
                self.vel_controller.set_move_cmd(0.0, -1)
                # self.vel_controller.publish()
                # while self.closest_object < 0.5:
                #     self.closest_object = self.tb3_lidar.min_distance
                #     if self.actionserver.is_preempt_requested():
                #         ## Take appropriate action if the action is
                #         ## cancelled (pre-empted) [DONE]
                #         print("Pre-empt requested!")
                #         self.actionserver.set_preempted(self.result)
                #         self.vel_controller.stop()
                #         success = False
                #         # exit the loop:
                #         break
                
                # while self.left_wall
                # self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)
                # self.vel_controller.publish()

                # rospy.sleep(1.571)           
                # self.vel_controller.set_move_cmd(0.0, 0)
                # self.vel_controller.publish()    
            # if self.right_closest_object < 0.5:
            #     rospy.loginfo("Obstacle detected on right side! Rotating left to avoid.")
            #     self.vel_controller.set_move_cmd(0.0, 1)
            # elif self.left_closest_object < 0.5:
            #     rospy.loginfo("Obstacle detected on left side! Rotating right to avoid.")
            #     self.vel_controller.set_move_cmd(0.0, -1)
            elif self.left_arc_min < 0.3:
                print('turning right from small arc')
                self.vel_controller.set_move_cmd(0.0, -0.2)
            elif self.right_arc_min < 0.3:
                print('turning left from small arc')
                self.vel_controller.set_move_cmd(0, 0.2)
            else:
                self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.0)

            self.vel_controller.publish()


            # print('this is i = ', self.i)

            # if self.i== 350: #should be 300
            #     print('###############################################################################')
            #     print('###############################################################################')
            #     print('facing the center')
            #     self.vel_controller.set_move_cmd(0.0, 0)
            #     self.vel_controller.publish()
            #     rospy.sleep(2)
            #     dx = self.posx0 - self.tb3_odom.posx
            #     dy = self.posy0 - self.tb3_odom.posy
            #     angle_to_start = np.arctan2(dy, dx)
            #     relative_angle = angle_to_start - self.tb3_odom.yaw
            #     print('relative angle = ', relative_angle)

            #     self.vel_controller.set_move_cmd(0.0, 1)
            #     self.vel_controller.publish()
            #     rospy.sleep(abs(relative_angle))
            #     self.vel_controller.set_move_cmd(0.0, 0)
            #     self.vel_controller.publish()
                

            ## Update all feedback message values 
            ## and publish a feedback message [DONE]
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)

            ## Update all result parameters [DONE]
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_angle = self.closest_object_location
            self.result.closest_object_distance = self.closest_object

            rate.sleep()
            self.i += 1 

        if success:
            rospy.loginfo("approach completed successfully.")
            ## Set the action server to "succeeded" and stop the robot [DONE]
            self.vel_controller.stop()
            self.actionserver.set_succeeded(self.result)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    node = SearchActionServer()
    node.main()