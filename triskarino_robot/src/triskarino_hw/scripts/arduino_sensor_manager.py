import rospy
from nav_msgs.msg import Odometry
from triskarino_msgs.msg import Sonar 
import tf 
import json 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import serial 


PUBLISHER_QUEUE_SIZE = 100

class ArduinoSensorManagerNode():
    NODE_NAME = "arduino_sensor_manager"
    def __init__(self,port='/dev/ttyACM0', baud_rate=115200, timeout=1):
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        self.ser.flush()
        rospy.init_node("arduino_sensor_manager")
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=PUBLISHER_QUEUE_SIZE)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.sonar_pub = rospy.Publisher('sonar',Sonar,queue_size=PUBLISHER_QUEUE_SIZE)
        self.listen_to_twist = rospy.Subscriber("cmd_vel", Twist, self.read_and_publish_sensors_data)
    

    def read_and_publish_sensors_data(self,twist_data):
        while not rospy.is_shutdown():
            line = self.ser.readline()
            line = line.decode("utf-8-sig")
            if line == "" or line == "\n":
                continue
            try:
                stripped_decoded_line = line.strip('\n')
                rospy.loginfo(stripped_decoded_line)
                #msg = json.loads(stripped_decoded_line)
                #self.publish_odometry(msg)
                #self.publish_sonar(msg)
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
    node = ArduinoSensorManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
