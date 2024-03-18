#!/usr/bin/env python3
import rospy
from triskarino_msgs.msg import Sound
from subprocess import run

#INSTRUCTIONS
#This class is responsible for playing sounds
#It subscribes to the sound topic and plays the sound file specified in the message
#The sound is played using the play command using ALSA (Advanced Linux Sound Architecture) driver
#The sound file is played using the -t flag that selects the driver to use (ALSA in this case)
#The volume is set using the -v flag (1 is the default volume)
#The sound file is specified using the filepath field


# Node initialization
class SoundManagerNode():
    NODE_NAME = "sound_manager"
    def __init__(self):
        rospy.init_node("sound_manager")                                            # initialize the node
        self.sound_subscriber = rospy.Subscriber("sound", Sound, self.play_sound)   # subscribe to the sound topic

    # Play the sound
    def play_sound(self,sound_data):
        rospy.loginfo(run("pwd",shell = False, capture_output= True))  # log the current working directory
        rospy.loginfo(run("ls",shell = False, capture_output= True))   # log the contents of the current working directory

        #-t alsa is there only to stop a warning, volume has to be a real number (1 is actual default volume)
        command_string = "play -v " + str(sound_data.volume) + " " + str(sound_data.filepath) + " -t alsa"  # create the command string to play the sound
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