import rospy
from nav_msgs.msg import Odometry
from triskarino_msgs.msg import Sonar 
import tf 
import json 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import serial 

PUBLISHER_QUEUE_SIZE = 100

class ArduinoWheelsManagerNode():
    NODE_NAME = "arduino_wheels_manager"
    def __init__(self,port='/dev/ttyACM0', baud_rate=115200, timeout=1):
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        self.ser.flush()
        rospy.init_node("arduino_wheels_manager")
        self.listen_to_twist = rospy.Subscriber("cmd_vel", Twist, self.move_robot)
    
    def move_robot(self,twist_data):
        #twist_msg = ":"+str(twist_data.linear.x)+","+str(twist_data.linear.y)+","+str(twist_data.angular.z)
        #rospy.loginfo("Received Twist message is: " + str(twist_msg))
        #self.ser.write(bytes((twist_msg+'\n'), encoding='utf-8'))
        twist_msg = {"twist": [twist_data.linear.x, twist_data.linear.y, twist_data.angular.z]}
        serialized_twist_msg = json.dumps(twist_msg)
        rospy.loginfo("Received Twist message is: " + str(twist_msg))
        self.ser.write(bytes((serialized_twist_msg+'\n'), encoding='utf-8'))

    

if __name__ == '__main__':
    print("AO")
    rospy.loginfo("AO")
    node = ArduinoWheelsManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
