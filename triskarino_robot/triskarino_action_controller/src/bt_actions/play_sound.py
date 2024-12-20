#!/usr/bin/env python3

import py_trees
import rospy
from triskarino_msgs.msg import Sound

#TODO: Use rospack to get the path
SOUND_FOLDER = "~/catkin_ws/src/triskarino_robot/triskarino_hw/resources/"

class PlaySound(py_trees.behaviour.Behaviour):
    """
    A behavior class to play a sound using ROS (Robot Operating System).
    Attributes:
        topic_name (str): The name of the ROS topic to publish the sound message.
        sound_volume (float): The volume of the sound to be played.
        sound_filepath (str): The file path of the sound file to be played.
    Methods:
        setup(timeout):
            Initializes the ROS publisher for the sound topic.
        update():
            Publishes the sound message to the ROS topic and returns the status of the behavior.
        terminate(new_status):
            Cleans up any resources or state when the behavior terminates.
    """

    def __init__(self, name, topic_name="/sound", sound_volume=0.5, sound_filepath="happy.wav"):
        super(PlaySound, self).__init__(name=name)
        self.topic_name = topic_name
        self.sound_volume = sound_volume
        self.sound_filepath = sound_filepath


    def setup(self,timeout):
        self.sound_pub = rospy.Publisher(self.topic_name,Sound,queue_size=10)
        return True
    
    def update(self):
        #Executing action
        sound_msg = Sound()
        sound_msg.volume = self.sound_volume
        sound_msg.filepath = SOUND_FOLDER + self.sound_filepath
        self.feedback_message = "Sound played is " + str(self.sound_filepath) + " with volume " + str(self.sound_volume)
        return py_trees.common.Status.SUCCESS

    def terminate(self,new_status):
        pass