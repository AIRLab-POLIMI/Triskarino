#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time



PUBLISHER_QUEUE_SIZE = 100
MAX_SPEED = 80  #cm/s
MAX_ANGULAR = 6.28 #rad/s

class DebugTeleopNode():
    NODE_NAME = "debugTeleop"
    def __init__(self):
        rospy.init_node("debugTeleop")
        self.debug_sub = rospy.Subscriber('cmd_vel', Twist, self.log_twist)
        self.speedX = []
        self.speedY = []
        self.speedTh = []
        self.time = []
        plt.figure()
        self.figure, self.axis = plt.subplots(3,1)
        plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.9,
                    wspace=0.4,
                    hspace=0.8)
        self.start_time = time.time_ns() / 1000
    
    def log_twist(self,twist_msg):
        rospy.loginfo("Received " + str(twist_msg))
        self.time.append((time.time_ns() / 1000)-self.start_time)
        self.speedX.append(twist_msg.linear.x * MAX_SPEED)
        self.speedY.append(twist_msg.linear.y * MAX_SPEED)
        self.speedTh.append(twist_msg.angular.z * MAX_ANGULAR)
        self.plot_vel()
        
    def plot_vel(self):
        self.axis[0].clear()
        self.axis[0].plot(self.time,self.speedX)
        self.axis[1].clear()
        self.axis[1].plot(self.time,self.speedY)
        self.axis[2].clear()
        self.axis[2].plot(self.time,self.speedTh)

    
   
if __name__ == '__main__':
    print("AO")
    node = DebugTeleopNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    node.figure.savefig("../requested_speeds.jpg")
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
