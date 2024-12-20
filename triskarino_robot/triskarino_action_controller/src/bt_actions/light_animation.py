#!/usr/bin/env python3

import py_trees
import rospy
from triskarino_msgs.msg import Light

class LightAnimation(py_trees.behaviour.Behaviour):
    """
    A behavior class for controlling light animations in a ROS-based robot.
    Attributes:
        topic_name (str): The name of the ROS topic to publish light messages to.
        light_color (tuple): A tuple representing the RGB color of the light.
        light_action_type (str): The type of light action to perform.
        light_delay (int): The delay between light actions.
        brightness (int): The brightness level of the light.
    Methods:
        setup(timeout):
            Initializes the ROS publisher for the light topic.
        update():
            Executes the light action by publishing a light message to the ROS topic.
        terminate(new_status):
            Cleans up any resources when the behavior terminates.
    """

    def __init__(self, name, topic_name="/light", light_color=(255,0,0),light_action_type="A",light_delay=20,brightness=50):
        super(LightAnimation, self).__init__(name=name)
        self.topic_name = topic_name
        self.light_color = light_color
        self.light_action_type = light_action_type
        self.light_delay = light_delay
        self.brightness = brightness


    def setup(self,timeout):
        self.light_pub = rospy.Publisher('light',Light,queue_size=10)
        return True
    
    def update(self):
        #Executing action
        light_msg = Light()
        light_msg.action = self.light_action_type
        light_msg.delay = self.light_delay
        light_msg.color[0] = self.light_color[0]
        light_msg.color[1] = self.light_color[1]
        light_msg.color[2] = self.light_color[2]
        light_msg.brightness = self.brightness
        self.light_pub.publish(light_msg)
        self.feedback_message = "Light action of type " + str(self.light_action_type) + " and color " + str(self.light_color) + " was sent"
        return py_trees.common.Status.SUCCESS

    def terminate(self,new_status):
        pass