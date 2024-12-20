#!/usr/bin/env python3

import py_trees
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

WAIT_FOR_MESSAGE = 20

class MoveLinOdom(py_trees.behaviour.Behaviour):

    def __init__(self, name, distance=0.3, max_speed=1.0,topic="cmd_vel_social_lin"):
        super(MoveLinOdom, self).__init__(name=name)
        self.distance = distance   
        self.max_speed = max_speed
        self.vel_publisher = rospy.Publisher(topic,Twist,queue_size=10)
        self.navigation_started = False
        
    def setup(self, timeout):
        return True
    
    
    def get_distance(self, current_position, starting_position):
        return math.sqrt((current_position.pose.pose.position.x - starting_position.pose.pose.position.x) ** 2 + (current_position.pose.pose.position.y - starting_position.pose.pose.position.y) ** 2)
    
    def update(self):
        # Your code here
        try:
            if not self.navigation_started:
                rospy.loginfo("Started Moving Linearly")
                self.starting_position = rospy.wait_for_message("/odom", Odometry, timeout=WAIT_FOR_MESSAGE)
                self.navigation_started = True
            current_position = rospy.wait_for_message("/odom", Odometry, timeout=WAIT_FOR_MESSAGE)
        except:
            rospy.logerr("Error while waiting for odom message")
            return py_trees.common.Status.RUNNING
        
        distance_elapsed = self.get_distance(current_position,self.starting_position)
        twist_msg = Twist()
        if distance_elapsed >= self.distance:
            twist_msg.linear.x = 0  
            self.vel_publisher.publish(twist_msg)
            rospy.loginfo(" Finished moving linearly ")
            self.navigation_started = False
            return py_trees.common.Status.SUCCESS
        else:
            twist_msg.linear.x = self.max_speed 
            self.vel_publisher.publish(twist_msg)
            return py_trees.common.Status.RUNNING


    def terminate(self,new_status):
        """
        When terminating, if the goal is still in the move base, cancel the goal
        """
        pass
