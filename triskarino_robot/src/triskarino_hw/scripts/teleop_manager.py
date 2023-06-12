import rospy
from geometry_msgs.msg import Twist
from triskarino_msgs.msg import Light, Sound
from evdev import InputDevice


INPUT_PATH = "/dev/input/by-id/usb-Logitech_Wireless_Gamepad_F710_653595B3-event-joystick"
ANALOG_MAX_VALUE = 32768
BUTTON_MAX_VALUE = 255
PUBLISHER_QUEUE_SIZE = 10
#CUTTING MAXIMUM ANGULAR AND LINEAR VELOCITIES TO HAVE BETTER CONTROL WITH JOYSTICK
MAX_ANGULAR = 0.2
MAX_LINEAR = 0.5
ROUND_DIGITS_LINEAR = 1
ROUND_DIGITS_ANGULAR = 2
#LIGHT CONSTANTS
WAIT = 10
COLOR = [255,0,0]
#SOUND CONSTANTS
VOLUME=600

class TeleopManagerNode():
    NODE_NAME = "teleop_manager"
    def __init__(self):
        rospy.init_node("teleop_manager")
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=PUBLISHER_QUEUE_SIZE)
        self.light_pub = rospy.Publisher('light',Light,queue_size=PUBLISHER_QUEUE_SIZE)
        self.sound_pub = rospy.Publisher('sound',Sound,queue_size=PUBLISHER_QUEUE_SIZE)
        self.gamepad = InputDevice(INPUT_PATH)
        rospy.loginfo(self.gamepad)

    
    def get_teleop_input(self):
        rospy.loginfo("In Teleop Function")
        twist_msg = Twist()
        light_msg = Light()
        sound_msg = Sound()
        publish_twist = False
        publish_light = False
        publish_sound = False
        for event in self.gamepad.read_loop():
            #RIGHT ANALOG: Event Code 4 equals to up/down -> Putting it on x and negating to align forward with robot's face
            # Event Code 3 equals to right/left values -> Putting it to y and negating to align with robot's left/right are between 32768 and -32768
            #LT: Event Code 2 value goes from 0 to 255
            #RT: Event Code 5 value goes from 0 to 255
            #Idea Parse UntiL Event.Code is equal to 0 (which is what the input uses to separate input from different buttons) and then send it to twist
            #RIGHT ANALOG BUTTON: Event Code 318, values 1 or 0
            #Button A is 304, values 1 or 0
            #Button B is 305, values 1 or 0 
            #Button X is 307 values 1 or 0
            #Button Y is 308 values 1 or 0
            #print("Event Code is " + str(event.code) + " Event Value is " + str(event.value))
            if event.code == 0:
                #EVENT CODE 0 is sent after each input, (analogic with both x and y components counts as one input but it is read with two iteration cycles)
                if publish_twist:
                    rospy.loginfo(str(twist_msg))
                    self.twist_pub.publish(twist_msg)
                    publish_twist = False
                if publish_light:
                    print(light_msg)
                    self.light_pub.publish(light_msg)
                    publish_light = False
                if publish_sound:
                    print(sound_msg)
                    self.sound_pub.publish(sound_msg)
                    publish_sound = False
            elif event.code == 4:
                #UP/DOWN analogic, controls robot forward and backwards movement
                twist_msg.linear.x = - round((event.value / ANALOG_MAX_VALUE) * MAX_LINEAR,ROUND_DIGITS_LINEAR)
                publish_twist = True
            elif event.code == 3: 
                #RIGHT/LEFT analogic, controls robot forward and backwards movement
                twist_msg.linear.y = - round((event.value / ANALOG_MAX_VALUE) * MAX_LINEAR,ROUND_DIGITS_LINEAR)
                publish_twist = True
            elif event.code == 2:
                #LT controls left rotation
                twist_msg.angular.z = - round((event.value / BUTTON_MAX_VALUE) * MAX_ANGULAR,ROUND_DIGITS_ANGULAR)
                publish_twist = True
            elif event.code == 5: 
                #RT controls right rotation
                twist_msg.angular.z = round((event.value / BUTTON_MAX_VALUE) * MAX_ANGULAR,ROUND_DIGITS_ANGULAR)
                publish_twist = True
            elif event.code == 318:
                #PRESSING RIGHT ANALOG, used as a safe measure to stop the robot
                twist_msg.linear.x = 0
                twist_msg.linear.y = 0
                twist_msg.angular.z = 0
                publish_twist = True
            elif event.code == 304 and event.value == 1:
                #BUTTON A, activates colorwipe
                light_msg.color = COLOR
                light_msg.delay = WAIT
                light_msg.action = "colorWipe"
                publish_light = True
            elif event.code == 307 and event.value == 1:
                #BUTTON X, activates rainbow
                light_msg.color = COLOR
                light_msg.delay = WAIT
                light_msg.action = "rainbow"
                publish_light = True
            elif event.code == 308 and event.value == 1:
                #BUTTON Y, activates rainbowane
                light_msg.color = COLOR
                light_msg.delay = WAIT
                light_msg.action = "rainbowane"
                publish_light = True
            elif event.code == 305 and event.value == 1:
                #BUTTON B, turns lights off
                light_msg.color = COLOR
                light_msg.delay = WAIT
                light_msg.action = "off"
                publish_light = True
            elif event.code == 311 and event.value == 1:
                #RB plays acknowledged sound from r2d2
                sound_msg.filepath = "Triskarino/triskarino_robot/src/triskarino_hw/resources/acknowledged.wav"
                sound_msg.volume = VOLUME
                publish_sound = True
                

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
