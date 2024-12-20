#!/usr/bin/env python3

import py_trees
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

WAIT_FOR_MESSAGE = 20

class RotateRobotOdom(py_trees.behaviour.Behaviour):
    '''
    A behavior class to rotate a robot to a specified angle using odometry data.
    Attributes:
        angle_of_rotation (float): The angle by which the robot should rotate, in degrees.
        max_speed_of_rotation (float): The maximum speed of rotation for the robot.
        vel_publisher (rospy.Publisher): ROS publisher to publish velocity commands.
        navigation_started (bool): Flag to indicate if the rotation has started.
    Methods:
        __init__(name, angle_of_rotation=10, max_speed_of_rotation=0.5, topic="cmd_vel_social_rot"):
            Initializes the RotateRobotOdom behavior with the given parameters.
        calculate_goal_rotation(starting_quaternion):
            Calculates the goal rotation angle based on the starting orientation.
        get_rotation_percentage(starting_quaternion, goal_angle):
            Calculates the percentage of rotation completed towards the goal angle.
        convert_angle_to_degree(angle):
            Converts an angle from radians to degrees and normalizes it between 0 and 360 degrees.
        setup(timeout):
            Sets up the behavior. Returns True if setup is successful.
        did_robot_go_over_goal(current_orientation, goal_rotation, speed):
            Checks if the robot has rotated past the goal angle.
        update():
            Updates the behavior, publishing velocity commands to rotate the robot and checking if the goal is reached.
        terminate(new_status):
            Terminates the behavior, performing any necessary cleanup.
    '''

    def __init__(self, name, angle_of_rotation=10, max_speed_of_rotation=0.5,topic="cmd_vel_social_rot"):
        super(RotateRobotOdom, self).__init__(name=name)
        self.angle_of_rotation = angle_of_rotation
        self.max_speed_of_rotation = max_speed_of_rotation
        self.vel_publisher = rospy.Publisher(topic,Twist,queue_size=10)
        self.navigation_started =  False
    
    def calculate_goal_rotation(self, starting_quaternion):
        euler_angles = list(euler_from_quaternion([starting_quaternion.x, starting_quaternion.y, starting_quaternion.z, starting_quaternion.w]))
        deg_euler_angle = self.convert_angle_to_degree(euler_angles[2])
        return (deg_euler_angle + self.angle_of_rotation) % 360

    def get_rotation_percentage(self, starting_quaternion, goal_angle):
        euler_starting = euler_from_quaternion([starting_quaternion.x, starting_quaternion.y, starting_quaternion.z, starting_quaternion.w])
        deg_euler_angle = self.convert_angle_to_degree(euler_starting[2])
        return 1 - (abs(goal_angle - deg_euler_angle)/360)
    
    def convert_angle_to_degree(self, angle):
        #Convert to degree
        angle = angle % (2 * math.pi)
        deg_angle = angle * 180 / math.pi
        #Rescale between 0 and 360
        return deg_angle % 360
    
    
    def setup(self, timeout):
        return True

    def did_robot_go_over_goal(self, current_orientation, goal_rotation, speed):
        current_euler = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        curr_angle = self.convert_angle_to_degree(current_euler[2])
        #If speed > 0 and current orientation > goal rotation return 1
        if speed > 0 and curr_angle >= goal_rotation:
            return True
        elif speed < 0 and curr_angle <= goal_rotation:
            return True
        else:
            return False

    
    def update(self):
        try:
            if not self.navigation_started:
                rospy.loginfo("Starting to Rotate")
                self.starting_position = rospy.wait_for_message("/odom", Odometry, timeout=WAIT_FOR_MESSAGE)
                starting_quaternion = self.starting_position.pose.pose.orientation
                self.goal_rotation = self.calculate_goal_rotation(starting_quaternion)
                self.navigation_started = True
        
            current_position = rospy.wait_for_message("/odom", Odometry, timeout=WAIT_FOR_MESSAGE)
        except:
            rospy.loginfo("No odometry message received")
            return py_trees.common.Status.RUNNING
        goal_percentage = self.get_rotation_percentage(current_position.pose.pose.orientation, self.goal_rotation)
        twist_msg = Twist()
        #Going for an error of approximately 5 degrees acceptable
        if self.did_robot_go_over_goal(current_position.pose.pose.orientation, self.goal_rotation, self.max_speed_of_rotation) or goal_percentage >= 0.985:
            twist_msg.angular.z = 0
            self.vel_publisher.publish(twist_msg)
            self.navigation_started = False
            rospy.loginfo("Finished Rotation ")
            return py_trees.common.Status.SUCCESS
        else:
            twist_msg.angular.z = self.max_speed_of_rotation 
            self.vel_publisher.publish(twist_msg)
            return py_trees.common.Status.RUNNING


    def terminate(self,new_status):
        """
        When terminating, if the goal is still in the move base, cancel the goal
        """
        pass
