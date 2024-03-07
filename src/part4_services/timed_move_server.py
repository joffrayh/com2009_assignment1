#!/usr/bin/env python3 

import rospy
from geometry_msgs.msg import Twist 
from tuos_msgs.srv import Approach 
from sensor_msgs.msg import LaserScan
import numpy as np

class moveService():

    def __init__(self):
        service_name = "timed_move_service"
        rospy.init_node(f"{service_name}_server") 

        self.service = rospy.Service(service_name, Approach, self.srv_callback) 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)

        self.min_distance =  float('inf')

        rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 


    def callback(self, scan_data: LaserScan):
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()

        # Optional Extra:
        # arc_angles = np.arange(-20, 21)
        # self.object_angle = arc_angles[np.argmin(front_arc)]

    def srv_callback(self, request_from_client): 
        vel = Twist()

        print(f"Server received a request, stopping {request_from_client.approach_distance}m away from block, at a velocity of {request_from_client.approach_velocity}") 

        vel.linear.x = request_from_client.approach_velocity
    
        self.pub.publish(vel) 

        while self.min_distance > request_from_client.approach_distance: 
            continue
        
        vel.linear.x = 0.0
        self.pub.publish(vel) 


        rospy.loginfo(f'Stopped {request_from_client.approach_distance}m away from wall')
        
        return 'finished...'

    def main(self):
        rospy.spin() 

if __name__ == '__main__':
    server = moveService()
    server.main()   