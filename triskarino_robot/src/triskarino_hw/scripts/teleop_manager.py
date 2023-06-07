import rospy
from geometry_msgs.msg import Twist
from triskarino_msgs.msg import Light
from evdev import InputDevice

INPUT_PATH = "/dev/input/event3"
RATE=10
ANALOG_MAX_VALUE = 32768
BUTTON_MAX_VALUE = 255
PUBLISHER_QUEUE_SIZE = 10
#CUTTING MAXIMUM ANGULAR AND LINEAR VELOCITIES TO HAVE BETTER CONTROL WITH JOYSTICK
MAX_ANGULAR = 0.3
MAX_LINEAR = 0.3
ROUND_DIGITS = 2


class TeleopManagerNode():
    NODE_NAME = "teleop_manager"
    def __init__(self):
        rospy.init_node("teleop_manager")
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=PUBLISHER_QUEUE_SIZE)
        self.light_pub = rospy.Publisher('light',Light,PUBLISHER_QUEUE_SIZE)
        self.gamepad = InputDevice(INPUT_PATH)
        rospy.loginfo(self.gamepad)

    
    def get_teleop_input(self):
        rospy.loginfo("In Teleop Function")
        twist_msg = Twist()
        light_msg = Light()
        publish_twist = False
        publish_light = False
        for event in self.gamepad.read_loop():
            #RIGHT ANALOG: Event Code 4 equals to up/down, Event Code 3 equals to right/left values are between 32768 and -32768
            #LT: Event Code 2 value goes from 0 to 255
            #RT: Event Code 5 value goes from 0 to 255
            #Idea Parse UntiL Event.Code is equal to 0 (which is what the input uses to separate input from different buttons) and then send it to twist
            #RIGHT ANALOG BUTTON: Event Code 318, values 1 or 0
            print("Event Code is " + str(event.code) + " Event Value is " + str(event.value))
            if event.code == 0:
                if publish_twist:
                    self.twist_pub.publish(twist_msg)
                    publish_twist = False
                if publish_light:
                    self.light_pub.publish(light_msg)
                    publish_light = False
            elif event.code == 4:
                twist_msg.linear.y = round((event.value / ANALOG_MAX_VALUE) * MAX_LINEAR,ROUND_DIGITS)
                publish_twist = True
            elif event.code == 3: 
                #Get the value and put it into twist linear x
                twist_msg.linear.x = round((event.value / ANALOG_MAX_VALUE) * MAX_LINEAR,ROUND_DIGITS)
                publish_twist = True
            elif event.code == 2:
                #Get the value and put it as negative rotation on z axis
                twist_msg.angular.z = - round((event.value / BUTTON_MAX_VALUE) * MAX_ANGULAR,ROUND_DIGITS)
                publish_twist = True
            elif event.code == 5: 
                #Get the value and put it as positive rotation on z axis
                twist_msg.angular.z = round((event.value / BUTTON_MAX_VALUE) * MAX_ANGULAR,ROUND_DIGITS)
                publish_twist = True
            elif event.code == 318:
                #Using this as a stop button that resets the twist message to all zeros
                twist_msg.linear.x = 0
                twist_msg.linear.y = 0
                twist_msg.angular.z = 0
                publish_twist = True
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
