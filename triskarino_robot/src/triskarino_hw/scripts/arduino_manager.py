import rospy
from nav_msgs.msg import Odometry
from triskarino_msgs.msg import Sonar 
import tf 
import json 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import serial 
import time 

TIMESTAMPS = []

PUBLISHER_QUEUE_SIZE = 100

class ArduinoManagerNode():
    NODE_NAME = "arduino_manager"
    def __init__(self,port='/dev/ttyACM0', baud_rate=115200, timeout=1):
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        self.ser.flush()
        rospy.init_node("arduino_manager")
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=PUBLISHER_QUEUE_SIZE)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.sonar_pub = rospy.Publisher('sonar',Sonar,queue_size=PUBLISHER_QUEUE_SIZE)
        self.listen_to_twist = rospy.Subscriber("cmd_vel", Twist, self.move_robot)
    
    def move_robot(self,twist_data):
        TIMESTAMPS.append({"function":"move_start","time":time.time()})
        #twist_msg = ":"+str(twist_data.linear.x)+","+str(twist_data.linear.y)+","+str(twist_data.angular.z)
        #rospy.loginfo("Received Twist message is: " + str(twist_msg))
        #self.ser.write(bytes((twist_msg+'\n'), encoding='utf-8'))
        twist_msg = {"twist": [twist_data.linear.x, twist_data.linear.y, twist_data.angular.z]}
        serialized_twist_msg = json.dumps(twist_msg)
        rospy.loginfo("Received Twist message is: " + str(twist_msg))
        self.ser.write(bytes((serialized_twist_msg+'\n'), encoding='utf-8'))
        TIMESTAMPS.append({"function":"move_written","time":time.time()})
        self.read_and_publish_sensors_data()
        TIMESTAMPS.append({"function":"move_read_publish","time":time.time()})
        rospy.loginfo("move_robot finished " + str(TIMESTAMPS))

    def read_and_publish_sensors_data(self):
        while not rospy.is_shutdown():
            line = self.ser.readline()
            if line == "":
                continue
            try:
                line = line.decode("utf-8-sig")
                stripped_decoded_line = line.strip('\n')
                msg = json.loads(stripped_decoded_line)
                self.publish_odometry(msg)
                self.publish_sonar(msg)
                break
            except Exception as e:
                rospy.loginfo("Couldn't deserialize Sensors Message was broken: " + str(stripped_decoded_line) + " Error message was " + str(e))
                break
    
    def publish_sonar(self,msg):
        sonarData = Sonar()
        for i in range(4):
            sonarData.distances[i] = msg["sonarData"][i]
        self.sonar_pub.publish(sonarData)
    
    def publish_odometry(self,msg):
        current_time = rospy.Time.now()
        x = msg["odometryPos"][0]
        y = msg["odometryPos"][1]
        th = msg["odometryPos"][2]
        speedX = msg["odometryVel"][0]
        speedY = msg["odometryVel"][1]
        speedTh = msg["odometryVel"][2]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, th)
        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (x, y, 0.),
            quaternion,
            current_time,
            "base_link",
            "odom"
        )
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*quaternion))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(speedX, speedY, 0), Vector3(0, 0, speedTh))

        # publish the message
        self.odom_pub.publish(odom)
    

if __name__ == '__main__':
    print("AO")
    rospy.loginfo("AO")
    node = ArduinoManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
