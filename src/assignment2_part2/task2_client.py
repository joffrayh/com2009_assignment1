#! /usr/bin/env python3

import rospy
import actionlib

from msg import SearchAction, SearchFeedback, SearchGoal

class Task2SearchActionClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):
        ## TODO: get the current distance travelled, from the feedback message
        ## and assign this to a class variable...
        self.distance = feedback_data.current_distance_travelled
        print(f'Current distance travelled: {self.distance} m.')



    def __init__(self):
        self.distance = 0.0
        self.action_complete = False

        node_name = 'task2_action_client'
        action_server_name = 'task2_action_server'

        rospy.init_node(node_name)
        self.rate = rospy.Rate(1)

        ## TODO: setup a "simple action client" with a callback function
        ## and wait for the server to be available...
        self.client = actionlib.SimpleActionClient(action_server_name, 
            SearchAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")

        ## TODO: Print the result here...
        # get the result:  
        rospy.sleep(1) # wait for the result to come in
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * {self.client.get_result}")




    def main(self):
        ## TODO: assign values to all goal parameters
        ## and send the goal to the action server...
        self.goal.fwd_velocity = 0.1
        self.goal.approach_distance = 0.5
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        while self.client.get_state() < 2:
            ## TODO: Construct an if statement and cancel the goal if the 
            ## distance travelled exceeds 2 meters...
            if self.distance > 2:
                self.client.cancel_goal() 
                # break out of the while loop to stop the node:
                break

            self.rate.sleep()

        self.action_complete = True

if __name__ == '__main__':
    ## TODO: Instantiate the node and call the main() method from it...
    actionClient = Task2SearchActionClient()
    actionClient.main()