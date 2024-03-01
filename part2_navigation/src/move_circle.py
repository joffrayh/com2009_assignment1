#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist

class Circle():

    def __init__(self):
        self.node_name = "move_circle"

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.vel_cmd = Twist()

        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s

        self.pub.publish(self.vel_cmd)

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self):
        

        self.ctrl_c = True

    def main(self):
        self.vel_cmd.linear.x = 0.2 # m/s
        self.vel_cmd.angular.z = 0.5 # rad/s
        while not self.ctrl_c:
            self.pub.publish(self.vel_cmd)
            


if __name__ == '__main__':
    node = Circle()
    node.main()