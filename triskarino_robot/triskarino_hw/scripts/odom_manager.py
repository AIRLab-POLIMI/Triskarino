#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from triskarino_msgs.msg import RawOdometry
import tf 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


PUBLISHER_QUEUE_SIZE = 100
#X,Y,TH 
twist_covariance = [0.05, 0.05, 0.05]
pose_covariance = [0.007, 0.007, 0.08]
#Variance per measurements that are always zero to get ekf to work
low_variance = 1.0e-13

class OdometryManagerNode():
    """
    OdometryManagerNode is responsible for managing and publishing odometry data for the robit as coming from arduino.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        odom_pub (rospy.Publisher): Publisher for the 'odom' topic.
        odom_broadcaster (tf.TransformBroadcaster): Broadcaster for the transform.
        listen_to_raw_odom (rospy.Subscriber): Subscriber for the 'rawOdometry' topic.
    Methods:
        __init__(): Initializes the ROS node, publisher, broadcaster, and subscriber.
        publish_odometry(odom_msg): Callback function to process raw odometry data and publish it.
    """

    NODE_NAME = "odometry_manager"
    def __init__(self):
        
        rospy.init_node("odometry_manager")
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=PUBLISHER_QUEUE_SIZE)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.listen_to_raw_odom = rospy.Subscriber("rawOdometry", RawOdometry, self.publish_odometry)
    
    def publish_odometry(self,odom_msg):
        #All values coming from arduino are in cms
        current_time = rospy.Time.now()
        x = odom_msg.odometryPos[0]/100
        y = odom_msg.odometryPos[1]/100
        th = odom_msg.odometryPos[2]
        speedX = odom_msg.odometryVel[0]/100
        speedY = odom_msg.odometryVel[1]/100
        speedTh = odom_msg.odometryVel[2]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, th)
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*quaternion))
        odom.pose.covariance[0] = pose_covariance[0]
        odom.pose.covariance[7] = pose_covariance[1]
        odom.pose.covariance[14] = low_variance
        odom.pose.covariance[21] = low_variance
        odom.pose.covariance[28] = low_variance
        odom.pose.covariance[35] = pose_covariance[2]
        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(speedX, speedY, 0), Vector3(0, 0, speedTh))
        odom.twist.covariance[0] = twist_covariance[0]
        odom.twist.covariance[7] = twist_covariance[1]
        odom.twist.covariance[14] = low_variance
        odom.twist.covariance[21] = low_variance
        odom.twist.covariance[28] = low_variance
        odom.twist.covariance[35] = twist_covariance[2]
        # publish the message
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    print("AO")
    rospy.loginfo("AO")
    node = OdometryManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
