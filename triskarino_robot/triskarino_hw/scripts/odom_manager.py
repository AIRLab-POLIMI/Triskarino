#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from triskarino_msgs.msg import RawOdometry
import tf 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


PUBLISHER_QUEUE_SIZE = 100

class OdometryManagerNode():
    NODE_NAME = "odometry_manager"
    def __init__(self):
        
        rospy.init_node("odometry_manager")
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=PUBLISHER_QUEUE_SIZE)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.listen_to_raw_odom = rospy.Subscriber("rawOdometry", RawOdometry, self.publish_odometry)
    
    def publish_odometry(self,odom_msg):
        current_time = rospy.Time.now()
        x = odom_msg.odometryPos[0]
        y = odom_msg.odometryPos[1]
        th = odom_msg.odometryPos[2]
        speedX = odom_msg.odometryVel[0]
        speedY = odom_msg.odometryVel[1]
        speedTh = odom_msg.odometryVel[2]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, th)
        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (x, y, 0.),
            quaternion,
            current_time,
            "origin_link",
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
        rospy.loginfo("Publsihing odometry: " + str(odom))

if __name__ == '__main__':
    print("AO")
    rospy.loginfo("AO")
    node = OdometryManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
