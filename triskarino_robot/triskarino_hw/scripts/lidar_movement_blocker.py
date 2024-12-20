#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import numpy as np


ANGLE_MIN = -20
ANGLE_MAX = 20

class LidarMovementBlocker():
    """
    A ROS node that processes LIDAR scan data to determine the distance to obstacles within a specified angular range.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        lidar_sub (rospy.Subscriber): Subscriber to the 'scan_filtered' topic for receiving LIDAR scan data.
        blocked_pub (rospy.Publisher): Publisher to the 'lidar_obstacle_distance' topic for publishing the distance to the nearest obstacle.
    Methods:
        __init__(): Initializes the ROS node, subscriber, and publisher.
        publish_obstacle_distance(scan_msg): Callback function for processing LIDAR scan data and publishing the obstacle distance.
        get_distance_from_scan(angle_min, angle_max, last_scan): Calculates the median distance to obstacles within the specified angular range.
        get_range_index(angle, last_scan): Converts an angle to the corresponding index in the LIDAR scan ranges list.
    """

    NODE_NAME = "lidarMovementBlocker"
    def __init__(self):
        rospy.init_node("lidarMovementBlocker")
        self.lidar_sub = rospy.Subscriber('scan_filtered', LaserScan, self.publish_obstacle_distance)
        self.blocked_pub = rospy.Publisher('lidar_obstacle_distance', Float32, queue_size=10)
    
    def publish_obstacle_distance(self,scan_msg):
        rad_angle_min = ANGLE_MIN * math.pi / 180
        rad_angle_max = ANGLE_MAX * math.pi / 180
        distance = self.get_distance_from_scan(rad_angle_min,rad_angle_max,scan_msg)
        obstacle_distance_msg = Float32()
        obstacle_distance_msg.data = distance
        self.blocked_pub.publish(obstacle_distance_msg)
        

    
    def get_distance_from_scan(self, angle_min, angle_max, last_scan):
        #Getting the range indexes for angle min and angle max
        min_range_idx = self.get_range_index(angle_min, last_scan)
        max_range_idx = self.get_range_index(angle_max,last_scan)
        #Gets all the scans in the range index
        selected_scans = last_scan.ranges[min_range_idx:max_range_idx:1]
        #We eliminate the nan measures from the list
        selected_scans_no_nan = list(filter(lambda x: not math.isnan(x), selected_scans))
        if len(selected_scans_no_nan) <= 10:
            return math.inf
        #Otherwise we return the median value. The median value shoud be resistant to outliers
        median_distance = np.median(np.asarray(selected_scans_no_nan))
        return median_distance
    
    #Gets the index of the angle in the last_scan ranges list
    def get_range_index(self, angle, last_scan):
        #The ranges indexes should start from 
        if angle > last_scan.angle_max:
            rospy.logwarn("Detected angle is over the lidar angleMax")
        #Range indexes start as 0 for anlge min and go on by angle increments in the LaserScan message
        if angle < 0:
            angle_difference = (abs(angle) - abs(last_scan.angle_min))
        else:
            angle_difference = (angle + abs(last_scan.angle_min))
        range_index = abs(angle_difference) / last_scan.angle_increment
        return int(range_index)
    
if __name__ == '__main__':
    node = LidarMovementBlocker()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
