import rospy
from triskarino_msgs.msg import Sound
from subprocess import run 


class SoundManagerNode():
    NODE_NAME = "sound_manager"
    def __init__(self):
        rospy.init_node("sound_manager")
        self.sound_subscriber = rospy.Subscriber("sound", Sound, self.play_sound)
    
    def play_sound(self,sound_data):
        command_string = "lxterminal -e omxplayer --vol " + str(sound_data.volume) + " " + str(sound_data.filepath) + " &"
        run(command_string,shell = True)
       

if __name__ == '__main__':
    rospy.loginfo("AO")
    node = SoundManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
