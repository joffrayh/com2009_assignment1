#!/usr/bin/env python3 

import rospy
from geometry_msgs.msg import Twist 
from tuos_msgs.srv import TimedMovement 

class moveService():

    def __init__(self):
        service_name = "detect_stop_service"
        rospy.init_node(f"{service_name}_server") 

        self.service = rospy.Service(service_name, TimedMovement, self.srv_callback) 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 

        rospy.loginfo(f"the '{service_name}' Server is ready to be called...") 

    def srv_callback(self, request_from_client): 
        vel = Twist()

        print(f"Server received a request, moving {request_from_client.movement_request}, for a duration of {request_from_client.duration}") 

        if request_from_client.movement_request == 'fwd':
            vel.linear.x = 1
        elif request_from_client.movement_request == 'back':
            vel.linear.x = -1
        elif request_from_client.movement_request == 'left':
            vel.angular.z = 0.1
        elif request_from_client.movement_request == 'right':
            vel.angular.z = -0.1
    
        StartTime = rospy.get_rostime() 
    
        self.pub.publish(vel) 

        rospy.loginfo('Published the velocity command to /cmd_vel')
        while (rospy.get_rostime().secs - StartTime.secs) < request_from_client.duration: 
            continue

        rospy.loginfo(f'{request_from_client.duration} seconds have elapsed, stopping the robot...')

        vel.linear.x = 0.0
        vel.angular.z  = 0.0
        self.pub.publish(vel) 
        
        return True

    def main(self):
        rospy.spin() 

if __name__ == '__main__':
    server = moveService()
    server.main()   