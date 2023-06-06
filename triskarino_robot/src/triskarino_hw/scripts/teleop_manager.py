import rospy
from geometry_msgs.msg import Twist
from evdev import InputDevice

INPUT_PATH = "/dev/input/event3"
RATE=10
ANALOG_MAX_VALUE = 32768
BUTTON_MAX_VALUE = 255

class TeleopManagerNode():
    NODE_NAME = "teleop_manager"
    def __init__(self):
        rospy.init_node("teleop_manager")
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.gamepad = InputDevice(INPUT_PATH)
        rospy.loginfo(self.gamepad)

    
    def get_teleop_input(self):
        rospy.loginfo("In Teleop Function")
        twist_msg = Twist()
        for event in self.gamepad.read_loop():
            #RIGHT ANALOG: Event Code 4 equals to up/down, Event Code 3 equals to right/left values are between 32768 and -32768
            #LT: Event Code 2 value goes from 0 to 255
            #RT: Event Code 5 value goes from 0 to 255
            #Idea Parse UntiL Event.Code is equal to 0 (which is what the input uses to separate input from different buttons) and then send it to twist
            if event.code == 0:
                print(twist_msg)
                self.twist_pub.publish(twist_msg)
            elif event.code == 4:
                twist_msg.linear.y = event.value / ANALOG_MAX_VALUE
            elif event.code == 3: 
                #Get the value and put it into twist linear x
                twist_msg.linear.x = event.value / ANALOG_MAX_VALUE
            elif event.code == 2:
                #Get the value and put it as negative rotation on z axis
                twist_msg.angular.z = - event.value / BUTTON_MAX_VALUE
            elif event.code == 5: 
                #Get the value and put it as positive rotation on z axis
                twist_msg.angular.z = event.value / BUTTON_MAX_VALUE
                
            else:
                continue
        
        rospy.loginfo("Exiting Teleop Function")




if __name__ == '__main__':
    rospy.loginfo("AO")
    node = TeleopManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    node.get_teleop_input()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
