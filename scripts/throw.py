#!/usr/bin/env python

import rospy
from mavbase.MAV import MAV
from geometry_msgs.msg import AccelWithCovarianceStamped


class ThrowFromBaloon():
    def __init__(self,MAV):
            # ROS setup
            self.rate = rospy.Rate(60)
            self.accel = AccelWithCovarianceStamped()
            self.drone = MAV
            

            # Subscribers
            self.accel_sub = rospy.Subscriber("/mavros/local_position/accel", AccelWithCovarianceStamped, self.accel_callback)
            
            # Attributes
    
    def accel_callback(self,data):
        self.lx = data.linear.x
        self.ly = data.linear.y
        self.lz = data.linear.z
        self.ax = data.angular.x
        self.ay = data.angular.y
        self.az = data.angular.z
    
    def run (self):
        self.drone.arm()
        self.drone.takeoff(2)
            



if __name__ == '__main__':
    rospy.init_node('throw')
    drone = MAV("felipe")
    c = ThrowFromBaloon(drone)
    c.run()