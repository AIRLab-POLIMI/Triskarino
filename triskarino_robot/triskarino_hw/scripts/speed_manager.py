#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

PUBLISHER_QUEUE_SIZE = 100
RATE = 30

class SpeedManagerNode():
    """
    A ROS node that keeps the last command to send to the robot and sends it at a fixed amount of Hz until it changes.
    This is done in order to make the PID more responsive to changes of the setpoint from the joystick.
    the speed of a robot by subscribing to a topic for incoming velocity commands
    and publishing them to another topic.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        joy_subscriber (rospy.Subscriber): Subscriber to the "cmd_vel_out" topic for receiving Twist messages.
        twist_publisher (rospy.Publisher): Publisher to the "cmd_vel_out_filled" topic for sending Twist messages.
        last_twist_msg (Twist): The last received Twist message.
        rate (rospy.Rate): The rate at which to run the main loop.
    Methods:
        __init__(): Initializes the ROS node, subscribers, and publishers.
        change_speed(twist_data): Callback function to update the last received Twist message.
        spin(): Main loop that publishes the last received Twist message at a specified rate.
    """

    NODE_NAME = "speed_manager"
    def __init__(self):
        rospy.init_node("speed_manager")
        self.joy_subscriber = rospy.Subscriber("cmd_vel_out", Twist, self.change_speed)
        self.twist_publisher = rospy.Publisher("cmd_vel_out_filled",Twist,queue_size=PUBLISHER_QUEUE_SIZE,latch = True)
        self.last_twist_msg = None
        self.rate = rospy.Rate(RATE)

    def change_speed(self,twist_data):
        self.last_twist_msg = twist_data
    
    def spin(self):
        while not rospy.is_shutdown():
            if self.last_twist_msg != None:
               # rospy.loginfo( "Publishing " + str(self.last_twist_msg) )
                self.twist_publisher.publish(self.last_twist_msg)
            self.rate.sleep()
        

if __name__ == '__main__':
    rospy.loginfo("AO")
    node = SpeedManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    node.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
