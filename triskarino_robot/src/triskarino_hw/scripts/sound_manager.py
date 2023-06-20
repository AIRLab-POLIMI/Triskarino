#!/usr/bin/env python3
import rospy
from triskarino_msgs.msg import Sound
from subprocess import run 


class SoundManagerNode():
    NODE_NAME = "sound_manager"
    def __init__(self):
        rospy.init_node("sound_manager")
        self.sound_subscriber = rospy.Subscriber("sound", Sound, self.play_sound)
    
    def play_sound(self,sound_data):
        rospy.loginfo(run("pwd",shell = False, capture_output= True))
        rospy.loginfo(run("ls",shell = False, capture_output= True))

        #-t alsa is there only to stop a warning, volume has to be a real number (1 is actual default volume)
        command_string = "play -v " + str(sound_data.volume) + " " + str(sound_data.filepath) + " -t alsa"
        rospy.loginfo("Executing Command " + command_string)
        run(command_string,shell = True)
        
        

if __name__ == '__main__':
    rospy.loginfo("AO")
    node = SoundManagerNode()
    rospy.loginfo("Executing from " + str(run("pwd",capture_output=True)))
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
