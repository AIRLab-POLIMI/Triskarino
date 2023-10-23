#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

PUBLISHER_QUEUE_SIZE = 100
RATE = 100

class SpeedManagerNode():
    NODE_NAME = "speed_manager"
    def __init__(self):
        rospy.init_node("speed_manager")
        self.joy_subscriber = rospy.Subscriber("cmd_vel_out", Twist, self.change_speed)
        self.twist_publisher = rospy.Publisher("cmd_vel_out_filled",Twist,queue_size=PUBLISHER_QUEUE_SIZE)
        self.last_twist_msg = None
        self.rate = rospy.Rate(RATE)

    def change_speed(self,twist_data):
        self.last_twist_msg = twist_data
    
    def spin(self):
        while not rospy.is_shutdown():
            if self.last_twist_msg != None:
                rospy.loginfo( "Publishing " + str(self.last_twist_msg) )
                self.twist_publisher.publish(self.last_twist_msg)
            self.rate.sleep()
        

if __name__ == '__main__':
    rospy.loginfo("AO")
    node = SpeedManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    node.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
