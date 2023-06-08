import rospy
from triskarino_msgs.msg import Light
import json 
import serial 


class LightsManagerNode():
    NODE_NAME = "lights_manager"
    def __init__(self,port='/dev/ttyUSB0', baud_rate=115200, timeout=1):
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        self.ser.flush()
        rospy.init_node("lights_manager")
        self.light_subscriber = rospy.Subscriber("light", Light, self.activate_lights)
    
    def activate_lights(self,lights_data):
        light_msg = {
            "color": [lights_data.color[0], lights_data.color[1], lights_data.color[2]],
            "action": lights_data.action,
            "wait": lights_data.delay
        }
        serialized_light_msg = json.dumps(light_msg)
        rospy.loginfo("Received Light message is: " + str(light_msg))
        self.ser.write(bytes((serialized_light_msg+'\n'), encoding='utf-8'))
       

if __name__ == '__main__':
    rospy.loginfo("AO")
    node = LightsManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
