#!/usr/bin/env python

import rospy
from mavbase.MAV import MAV
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State


class ThrowFromBaloon():
    vlz = 0
    
    def __init__(self,MAV):
            # ROS setup
            self.rate = rospy.Rate(60)
            self.accel = AccelWithCovarianceStamped()
            self.vel = TwistStamped()
            self.drone = MAV
            self.droneState = State()

            # Subscribers
            self.accel_sub = rospy.Subscriber("/mavros/local_position/accel", AccelWithCovarianceStamped, self.accel_callback)
            self.vel_sub = rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, self.vel_callback)
            self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)
            
            # Attributes
    def state_callback(self,state_data):
        self.droneState = state_data

    def accel_callback(self,accel_data):
        self.lx = accel_data.linear.x
        self.ly = accel_data.linear.y
        self.lz = accel_data.linear.z
        self.ax = accel_data.angular.x
        self.ay = accel_data.angular.y
        self.az = accel_data.angular.z

    def vel_callback(self,vel_data):
        #self.vlx = vel_data.linear.x
        #self.vly = vel_data.linear.y
        self.vlz = vel_data.twist.linear.z
        #self.vax = vel_data.angular.x
        #self.vay = vel_data.angular.y
        #self.vaz = vel_data.angular.z

    
    def run (self):
        self.drone.arm(True)
        rospy.loginfo("ARMING")
        while(not rospy.is_shutdown()):
            if not self.droneState.armed:
                self.drone.arm(True)
                rospy.loginfo("ARMING")
            rospy.loginfo("Vel(Z) = " + str(self.vlz))
            #print("Vel: " + self.vlz )
            if self.vlz < -3:
                rospy.loginfo('FALL DETECTED!!!')
                self.drone.takeoff(4)
                self.drone.hold(10)
            self.drone.hold(0.1)
            #self.setStabilizeMode()
            


if __name__ == '__main__':
    rospy.init_node('throw')
    drone = MAV("felipe")
    c = ThrowFromBaloon(drone)
    c.run()