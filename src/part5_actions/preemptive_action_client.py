#!/usr/bin/env python3

import rospy
import actionlib

from tuos_msgs.msg import CameraSweepAction, CameraSweepGoal, CameraSweepFeedback

class PreemptiveActionClient(): 
    goal = CameraSweepGoal() 

    def feedback_callback(self, feedback_data: CameraSweepFeedback): 
        self.captured_images = feedback_data.current_image
        print(f"FEEDBACK: Current yaw: {feedback_data.current_angle:.1f} degrees. "
              f"Image(s) captured so far: {self.captured_images}...")

    def __init__(self): 
        self.captured_images = 0
        self.action_complete = False

        node_name = "preemptive_camera_sweep_action_client"
        action_server_name = "/camera_sweep_action_server"

        rospy.init_node(node_name)

        self.rate = rospy.Rate(3)

        self.client = actionlib.SimpleActionClient(action_server_name, 
                    CameraSweepAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self): 
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")

        # get the result:  
        rospy.sleep(1) # wait for the result to come in
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * {self.captured_images} image(s) saved to {self.client.get_result()}")

    def send_goal(self, images, angle): 
        self.goal.sweep_angle = angle
        self.goal.image_count = images

        # send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)        

    def main(self):
        self.send_goal(images = 10, angle = 90) 
        i = 1 
        print("While we're waiting, let's do our seven-times tables...")
        while self.client.get_state() < 2:
            if self.captured_images == 5:
                self.client.cancel_goal()   
                break
            print(f"STATE: Current state code is {self.client.get_state()}")
            print(f"TIMES TABLES: {i} times 7 is {i*7}")
            i += 1
            self.rate.sleep()
        self.action_complete = True 

if __name__ == '__main__':
    actionClient = PreemptiveActionClient()
    actionClient.main()