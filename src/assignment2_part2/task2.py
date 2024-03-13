#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from my_tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

class Task2ActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.server_name = "task2_action_server"
        rospy.init_node(self.server_name)

        ## TODO: create a "simple action server" with a callback function, and start it...
        self.actionserver = actionlib.SimpleActionServer(self.server_name, 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()


        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        self.wall_max_distance = 0.5

        rospy.loginfo("The 'Search Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        ## TODO: Implement some checks on the "goal" input parameter(s)
        success = True
        if goal.fwd_velocity > 0.26 or goal.fwd_velocity <=0:
            print('Invalid velocity! Select a velocity between 0 and 0.26 m/s.')
            success = False

        if goal.approach_distance <=0 or goal.approach_distance > 3:
            print('Invalid approach distance! Select a distance between 0 and 3.')
            success = False

        if not success:
            ## TODO: abort the action server if an invalid goal has been requested...
            self.result.total_distance_travelled = 0
            self.result.closest_object_distance = 0
            self.result.closest_object_angle = 0
            self.actionserver.set_aborted(self.result)
            return

        ## TODO: Print a message to indicate that the requested goal was valid
        print(f"The requested goal was valid,")
        print(f"Travelling at a speed of {goal.fwd_velocity} m/s")
        print(f"With a distance from the walls of {goal.approach_distance} m ...")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance 
        #closest object referes to the object at in the front 40 degrees 
        self.closest_object_location = self.tb3_lidar.closest_object_position

        self.left_wall_closest = self.tb3_lidar.left_wall
        self.back_left = self.tb3_lidar.back_left_wall
        self.front_left = self.tb3_lidar.front_left_wall

        ## TODO: set the robot's forward velocity (as specified in the "goal")...
        self.vel_controller.set_move_cmd(goal.fwd_velocity, 0)

        ## TODO: establish a conditional statement so that the
        ## while loop continues as long as the distance to the closest object
        ## ahead of the robot is always greater than the "approach distance"
        ## (as specified in the "goal")...
        while self.closest_object > goal.approach_distance:
            # update LaserScan data:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            self.left_wall_closest = self.tb3_lidar.left_wall
            self.back_left = self.tb3_lidar.back_left_wall
            self.front_left = self.tb3_lidar.front_left_wall

            self.vel_controller.set_move_cmd(goal.fwd_velocity, 0)

            ## TODO: publish a velocity command to make the robot start moving 
            self.vel_controller.publish()

            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

            if abs(self.back_left - self.front_left) > 0.01:
                print('back - front')
                if self.back_left > self.front_left:
                    self.vel_controller.set_move_cmd(goal.fwd_velocity, -0.2)            
                else:
                    self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.2)            
                self.vel_controller.publish()

            if self.left_wall_closest < 0.2:
                print('too close')
                self.vel_controller.set_move_cmd(goal.fwd_velocity, -0.2)
            elif self.left_wall_closest >0.5:
                self.vel_controller.set_move_cmd(goal.fwd_velocity, 0.2)
            self.vel_controller.publish()

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                ## TODO: take appropriate action if the action is cancelled (pre-empted)...
                rospy.loginfo("Cancelling the serach server.")

                self.result.total_distance_travelled = self.distance
                self.result.closest_object_distance = self.closest_object
                self.result.closest_object_angle = self.closest_object_location               
                self.actionserver.set_preempted(self.result)
                # stop the robot:
                self.vel_controller.stop()
                success = False
                # exit the loop:
                break

            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

            ## TODO: update all feedback message values and publish a feedback message:
            self.feedback.current_distance_travelled =  self.distance
            self.actionserver.publish_feedback(self.feedback)



            ## TODO: update all result parameters:
            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.closest_object
            self.result.closest_object_angle = self.closest_object_location 

            rate.sleep()

        if success:
            rospy.loginfo("approach completed successfully.")
            ## TODO: Set the action server to "succeeded" and stop the robot...
            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop()

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    node = Task2ActionServer()
    node.main()