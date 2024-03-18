#!/usr/bin/env python3
import rospy

from dynamic_reconfigure.server import Server
from triskarino_hw.cfg import MyParamsConfig

from geometry_msgs.msg import Twist
from triskarino_msgs.msg import Light, Sound
from std_msgs.msg import String
from evdev import InputDevice


INPUT_PATH = "/dev/input/by-id/usb-Logitech_Wireless_Gamepad_F710_653595B3-event-joystick"
ANALOG_MAX_VALUE = 32768
BUTTON_MAX_VALUE = 255
PUBLISHER_QUEUE_SIZE = 10

#CUTTING MAXIMUM ANGULAR AND LINEAR VELOCITIES TO HAVE BETTER CONTROL WITH JOYSTICK
MAX_ANGULAR = 2.0
MAX_LINEAR = 0.5
ROUND_DIGITS_LINEAR = 2
ROUND_DIGITS_ANGULAR = 2

#LIGHT CONSTANTS
BRIGHTNESS = 50      # Brightness of the light (0-255)
MAX_BRIGHTNESS = 255

#SOUND CONSTANTS
SOUND_PATH = "/home/jetson/catkin_ws/src/triskarino_robot/triskarino_hw/resources/"
VOLUME = 1.0





class TeleopManagerNode():
    NODE_NAME = "teleop_manager_node"
    BRIGHTNESS = 50

    def __init__(self):
        self.twist_pub = rospy.Publisher('cmd_joy', Twist, queue_size=PUBLISHER_QUEUE_SIZE)
        self.light_pub = rospy.Publisher('light',Light,queue_size=PUBLISHER_QUEUE_SIZE)
        self.sound_pub = rospy.Publisher('sound',Sound,queue_size=PUBLISHER_QUEUE_SIZE)
        self.cmd_pub = rospy.Publisher('cmd_chase',String,queue_size=PUBLISHER_QUEUE_SIZE,latch=True)
        self.gamepad = InputDevice(INPUT_PATH)
        self.chase_toy = True
        rospy.loginfo(self.gamepad)

        self.server = Server(MyParamsConfig, self.reconfigure_callback)
        self.int_prova = 1
    
    def reconfigure_callback(self,config,level):
        self.delay_A = config.delay_a
        self.delay_B = config.delay_b
        self.delay_X = config.delay_x
        self.delay_Y = config.delay_y
        rospy.loginfo("You have updated the delay parameters!")
        
        return config

    def get_teleop_input(self):

        global BRIGHTNESS # Declare as a global variable

        rospy.loginfo("In Teleop Function")
        twist_msg = Twist()
        light_msg = Light()
        sound_msg = Sound()
        cmd_msg = String()
        publish_twist = False
        publish_light = False
        publish_sound = False
        publish_cmd = False
        for event in self.gamepad.read_loop():
            #RIGHT ANALOG: #Event Code 318, values 1 or 0
            # Event Code 4 equals to up/down -> Putting it on x and negating to align forward with robot's face
            # Event Code 3 equals to right/left values -> Putting it to y and negating to align with robot's left/right are between 32768 and -32768

            #RIGHT ANALOG BUTTON:
            #Button A is 304, values 1 or 0
            #Button B is 305, values 1 or 0
            #Button X is 307 values 1 or 0
            #Button Y is 308 values 1 or 0

            #LEFT ANALOG: #Event Code 317, values 1 or 0
            # Event Code 1 equals to up/down
            # Event Code 0 equals to right/left

            #LEFT ANALOG BUTTON:
            #UP/DOWN ARROWS are code 17 and respectively values -1 and 1
            #RIGHT/LEFT ARROWS are code 16 and respectively values 1 and -1
            
            # Back Buttons:
            #LT: Event Code 2 value goes from 0 to 255
            #RT: Event Code 5 value goes from 0 to 255
            #LB: Event Code 310 value goes from 0 to 1
            #RB: Event Code 311 value goes from 0 to 1
            #Idea Parse UntiL Event.Code is equal to 0 (which is what the input uses to separate input from different buttons) and then send it to twist

            # EXTRA BUTTONS:
            #START: Event Code 315 value goes from 0 to 1
            #BACK: Event Code 314 value goes from 0 to 1

            #print("Event Code is " + str(event.code) + " Event Value is " + str(event.value))

            if event.code == 0:
                #EVENT CODE 0 is sent after each input, (analogic with both x and y components counts as one input but it is read with two iteration cycles)
                if publish_twist:
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
                if publish_cmd:
                    print(cmd_msg)
                    self.cmd_pub.publish(cmd_msg)
                    publish_cmd = False

                ##################### Movement #####################

            elif event.code == 4:
                #UP/DOWN analogic, controls robot forward and backwards movement
                twist_msg.linear.x = -round((event.value / ANALOG_MAX_VALUE) * MAX_LINEAR,ROUND_DIGITS_LINEAR)
                publish_twist = True
            elif event.code == 3:
                #RIGHT/LEFT analogic, controls robot right and left
                twist_msg.linear.y = round((event.value / ANALOG_MAX_VALUE) * MAX_LINEAR,ROUND_DIGITS_LINEAR)
                publish_twist = True
            elif event.code == 2:
                #LT controls left rotation
                twist_msg.angular.z = round((event.value / BUTTON_MAX_VALUE) * MAX_ANGULAR,ROUND_DIGITS_ANGULAR)
                publish_twist = True
            elif event.code == 5:
                #RT controls right rotation
                twist_msg.angular.z = - round((event.value / BUTTON_MAX_VALUE) * MAX_ANGULAR,ROUND_DIGITS_ANGULAR)
                publish_twist = True

            elif event.code == 318:
                #PRESSING RIGHT ANALOG, used as a safe measure to stop the robot
                twist_msg.linear.x = 0
                twist_msg.linear.y = 0
                twist_msg.angular.z = 0
                publish_twist = True
            
            ##################### Lights #####################
                            
            elif event.code == 315 and event.value == 1:
                #START BUTTON
                light_msg.action = "Start"
                light_msg.delay = 10                 # Delay between each wipe
                publish_light = True
            elif event.code == 304 and event.value == 1:
                #BUTTON A: green light
                light_msg.action = "A"               # colorWipe: wipe color across display
                light_msg.color = [0, 255, 0]        # Green
                light_msg.delay = self.delay_A * 100 # Delay between each wipe
                light_msg.brightness = BRIGHTNESS    # Brightness of the light
                publish_light = True                 # Publish the light message
            elif event.code == 307 and event.value == 1:
                #BUTTON X: blue light
                light_msg.action = "X"               # colorWipe: wipe color across display
                light_msg.color = [0, 0, 255]        # Blue
                light_msg.delay = self.delay_X * 100 # Delay between each wipe
                light_msg.brightness = BRIGHTNESS    # Brightness of the light
                publish_light = True                 # Publish the light message
            elif event.code == 308 and event.value == 1:
                #BUTTON Y: wihte light
                light_msg.action = "Y"               # colorWipe: wipe color across display
                light_msg.color = [255, 255, 255]    # White
                light_msg.delay = self.delay_Y * 100 # Delay between each wipe
                light_msg.brightness = BRIGHTNESS    # Brightness of the light
                publish_light = True                 # Publish the light message
            elif event.code == 305 and event.value == 1:
                #BUTTON B: red light
                light_msg.action = "B"               # colorWipe: wipe color across display
                light_msg.color = [255, 0, 0]        # Red
                light_msg.delay = self.delay_B * 100 # Delay between each wipe
                light_msg.brightness = BRIGHTNESS    # Brightness of the light   
                publish_light = True                 # Publish the light message
            elif event.code == 311 and event.value == 1:
                #RB: black light
                light_msg.action = "RB"              # colorWipe: wipe color across display
                light_msg.color = [0, 0, 0]          # Black
                light_msg.delay = 100                # Delay between each wipe
                light_msg.brightness = 50            # Brightness of the light
                publish_light = True                 # Publish the light message
           
            elif event.code == 1 and event.value > 30000:
                BRIGHTNESS = max(0, BRIGHTNESS - 1)
                print("Brightness:" + str(BRIGHTNESS))
            elif event.code == 1 and event.value < -30000:
                BRIGHTNESS = min(255, BRIGHTNESS + 1)
                print("Brightness:" + str(BRIGHTNESS))
            elif event.code == 317:
                BRIGHTNESS = 50
                print("Brightness:" + str(BRIGHTNESS))
                print(str(self.int_prova))


            ##################### Sound #####################
            elif event.code == 17 and event.value == -1:
                #UP ARROW
                sound_msg.filepath = SOUND_PATH + "fearful.wav"  # Path to the sound file
                sound_msg.volume = 10                   # Volume of the sound
                publish_sound = True                        # Publish the sound message
            elif event.code == 17 and event.value == 1:
                #DOWN ARROW
                sound_msg.filepath = SOUND_PATH + "disgust.wav"  # Path to the sound file
                sound_msg.volume = 10                   # Volume of the sound
                publish_sound = True                        # Publish the sound message
            elif event.code == 16 and event.value == 1:
                #RIGHT ARROW
                sound_msg.filepath = SOUND_PATH + "angry.wav"  # Path to the sound file
                sound_msg.volume = 10                   # Volume of the sound
                publish_sound = True                        # Publish the sound message
            elif event.code == 16 and event.value == -1:
                #LEFT ARROW
                sound_msg.filepath = SOUND_PATH + "sad.wav"  # Path to the sound file
                sound_msg.volume = 10                   # Volume of the sound
                publish_sound = True                        # Publish the sound message
            elif event.code == 314 and event.value == 1:
                # BACK BUTTON
                sound_msg.filepath = SOUND_PATH + "happy.wav" # Path to the sound file
                sound_msg.volume = 10                   # Volume of the sound
                publish_sound = True                        # Publish the sound message     

            ##################### Commands #####################
            elif event.code == 310 and event.value == 1:
                #LB: toy chasing command
                cmd_msg.data = "chase_toy" if self.chase_toy else "stop_chasing_toy"
                self.chase_toy = not self.chase_toy
                publish_cmd = True                 # Publish the light message

           

            ##########################################################
            else:
                continue


        rospy.loginfo("Exiting Teleop Function")

if __name__ == '__main__':
    rospy.loginfo("AO")
    rospy.init_node("teleop_manager_node")
    node = TeleopManagerNode()
    rospy.sleep(1)
    rospy.loginfo( node.NODE_NAME + " running..." )
    node.get_teleop_input()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)