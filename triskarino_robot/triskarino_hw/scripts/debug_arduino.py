#!/usr/bin/env python3
import rospy
from std_msgs import String


PUBLISHER_QUEUE_SIZE = 100

class DebugArduinoNode():
    NODE_NAME = "debugArduino"
    def __init__(self):
        rospy.init_node("debugArduino")
        self.debug_sub = rospy.Subscriber('debug', String, self.log_debug)
     
    def log_debug(self,debug_msg):
       rospy.loginfo("Received " + debug_msg.data)

if __name__ == '__main__':
    print("AO")
    node = DebugArduinoNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
