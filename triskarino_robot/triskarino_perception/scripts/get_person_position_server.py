#!/usr/bin/env python3
import rospy
from triskarino_perception.srv import getPersonPosition
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import numpy as np
from jenkspy import JenksNaturalBreaks
import tf2_geometry_msgs
import tf2_ros

#FRAMES
FIXED_FRAME = "map"
ORIGIN_FRAME = "base_link"
PERSON_FRAME = "person"
#CAMERA PARAMETERS
H_FOV=62.2
IMG_W=640
IMG_H=640
SCAN_TOPIC="/scan_filtered"
#MARKER PARAMETERS
MARKER_LINE_LENGTH = 4
MARKER_X_OFFSET = 0.165
MARKER_Y_OFFSET = 0
LINE_SCALE = 0.03
MARKER_DURATION = 0.2
#ANGLE CORRECTION TO TAKE INTO ACCOUNT THE DIFFERENCE BETWEEN LIDAR AND CAMERA (TO SEE IF IT REMAINS THE SAME WHEN DOING CAMERA CALIBRATION)
ANGLE_CORRECTION = -5
#PERCENTAGE OF ANGLE TO CROP FROM THE BBOX BOUNDARIES TO MAKE IT MORE ACCURATE
ANGLE_CROP_PERRCENTAGE = 0.2
#N_CLASSES FOR JENKS
JENKS_CLASSES = 3
class GetPersonPositionServer():
    """
    A ROS service server that provides the position of a person detected in an image. Works using the person's bounding box and processing LIDAR scan data with Jenks Natural Breaks.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        tf_buffer (tf2_ros.Buffer): Buffer to store transformations.
        tf_listener (tf2_ros.TransformListener): Listener to receive transformations.
    Methods:
        __init__(): Initializes the ROS node, service, and transformation listener.
        handle_get_person_position(srv_request): Handles the service request to get the person's position.
        get_jensky_groups(scans): Groups scan data using Jenks Natural Breaks.
        get_distance_from_scan(angle_min, angle_max, last_scan): Gets the distance from the scan data within the specified angle range.
        get_gvf(groups, array): Calculates the Goodness of Variance Fit (GVF) for the given groups.
        get_range_index(angle, last_scan): Gets the index of the angle in the scan ranges list.
        convert_angle(angle): Converts angles from Hokuyo to ROS frame of reference.
        get_marker_message(angle_min, angle_max, time): Prepares a marker message for visualization in RViz.
    """

    NODE_NAME = "get_person_position_server"
    def __init__(self):
        rospy.init_node("get_person_position_server")
        s = rospy.Service('get_person_position',getPersonPosition, self.handle_get_person_position)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def handle_get_person_position(self,srv_request):
        person = srv_request.person
        #Convert the bounding box edges to angles and shift it by the H_FOV 
        angle_min = ((person.bbox[0] / IMG_W) * H_FOV) - (H_FOV / 2) + ANGLE_CORRECTION
        angle_max = ((person.bbox[2] / IMG_W) * H_FOV) - (H_FOV / 2) + ANGLE_CORRECTION
        angle_to_crop = (abs(angle_max) - abs(angle_min)) * ANGLE_CROP_PERRCENTAGE
        angle_min = angle_min + angle_to_crop
        angle_max = angle_max - angle_to_crop
        #Convert the angles to radians
        rad_angle_min = angle_min * math.pi / 180
        rad_angle_max = angle_max * math.pi / 180
        rad_angle_mean = (rad_angle_max + rad_angle_min) / 2 
        #Idea: get robot position at the same kind of time as when I get the scans
        #Getting the last_scan message published and the robot transform
        last_scan = rospy.wait_for_message(SCAN_TOPIC, LaserScan, timeout=5)
        time = rospy.Time.now()
        #Looking for the distance that corresponds to the median of the angles in the scan
        distance = self.get_distance_from_scan(rad_angle_min,rad_angle_max,last_scan)
        point = PointStamped()
        #Forward offset w.r.t the robot's lidar (x is the forward axis, y is the strafing axis), z is defaulted to 0
        #Accounting for difference in reference systems between Hokuyo and ros
        rad_angle_mean = self.convert_angle(rad_angle_mean)
        point.header.stamp = time
        point.header.frame_id = ORIGIN_FRAME
        point.point.x = distance * math.sin(rad_angle_mean) 
        point.point.y = distance * math.cos(rad_angle_mean)
        point.point.z = 0 
        marker = self.get_marker_message(rad_angle_min,rad_angle_max, time)

        return [point, marker]
    
    def get_jensky_groups(self, scans):
        jnb = JenksNaturalBreaks(JENKS_CLASSES)
        jnb.fit(scans)
        return jnb.groups_
   
        
    def get_distance_from_scan(self, angle_min, angle_max, last_scan):
        #Getting the range indexes for angle min and angle max
        min_range_idx = self.get_range_index(angle_min, last_scan)
        max_range_idx = self.get_range_index(angle_max,last_scan)
        #Gets all the scans in the range index
        selected_scans = last_scan.ranges[min_range_idx:max_range_idx:1]
        #We eliminate the nan measures from the list
        selected_scans_no_nan = list(filter(lambda x: not math.isnan(x), selected_scans))
        #If the list was all NaN measures we return NaN
        if len(selected_scans_no_nan) <= JENKS_CLASSES:
            return float('NaN')
        #Otherwise we return the median value. The median value shoud be resistant to outliers
        groups = self.get_jensky_groups(np.asarray(selected_scans_no_nan))
        gvf, sdcm_arr = self.get_gvf(groups, np.asarray(selected_scans_no_nan))
        #The first group is usually the person
        return np.mean(groups[0]).item()
    

    def get_gvf(self,groups, array):
        sdam =  np.sum((array - array.mean()) ** 2)
        sdcm_all = [np.sum((group - group.mean()) ** 2) for group in groups]
        sdcm = sum(sdcm_all)
        gvf = (sdam - sdcm) / sdam
        return gvf, sdcm_all

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
    
    #Convert angles from hokuyo to ros frame of reference 
    # See https://robotics.stackexchange.com/questions/64410/need-explanation-on-sensor-msgs-laserscan-msg for hokuyo reference
    def convert_angle(self, angle):
        return angle + 0.5 * math.pi - ANGLE_CORRECTION * math.pi / 180
    
    #Takes the angle min and max and prepares a marker message of line lenght specified in MARKER_LINE_LENGTH
    def get_marker_message(self, angle_min, angle_max, time):
        #Accounting for the difference in reference system between the axis in the hokuyo laser reference and in the rviz reference
        angle_min = self.convert_angle(angle_min)
        angle_max = self.convert_angle(angle_max)
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = time
        starting_point = Point()
        starting_point.x = MARKER_X_OFFSET
        starting_point.y = MARKER_Y_OFFSET
        starting_point.z = 0
        left_bbox_bound = Point()
        left_bbox_bound.x = math.sin(angle_min) * MARKER_LINE_LENGTH
        left_bbox_bound.y = math.cos(angle_min) * MARKER_LINE_LENGTH
        left_bbox_bound.z = 0 
        right_bbox_bound = Point()
        right_bbox_bound.x = math.sin(angle_max) * MARKER_LINE_LENGTH
        right_bbox_bound.y = math.cos(angle_max) * MARKER_LINE_LENGTH
        right_bbox_bound.z = 0 
        points = [starting_point, left_bbox_bound, right_bbox_bound, starting_point]
        marker.points = points
        marker.color.r = 1
        marker.color.a = 1 
        marker.scale.x = LINE_SCALE
        marker.lifetime = rospy.Duration.from_sec(MARKER_DURATION)
        marker.type = marker.LINE_STRIP
        return marker

        
if __name__ == '__main__':
    rospy.loginfo("AO")
    node = GetPersonPositionServer()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
