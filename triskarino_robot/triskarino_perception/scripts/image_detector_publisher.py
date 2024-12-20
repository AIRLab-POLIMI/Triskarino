#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from triskarino_perception.srv import getGroupDetection, getPersonPosition
from geometry_msgs.msg import Point, PoseStamped, PointStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from triskarino_msgs.msg import Group
from cv_bridge import CvBridge
import tf
import math
import tf2_ros
import tf2_geometry_msgs 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import traceback
import imutils
import time
import pandas as pd

VISUALIZATION=True
BROADCAST_TF=True
#Filter to tune to zoom out all people outside walls of the map (boxes) in the lab
MAX_DISTANCE=2 
PERSON_FRAME_ID ="person"
ORIGIN_FRAME_ID = "base_link"
FIXED_FRAME_ID = "map"
MAX_ROBOT_ROTATION = 0.05
#PERSON MARKER PARAMETERS
Z_POSITION = 0.5
MARKER_TYPE = Marker.CYLINDER
X_SCALE=0.15
Y_SCALE=0.15
Z_SCALE=1
MARKER_DURATION = 1
#Duration of transform listener
BUFFER_CACHE_TIME = 10
WAIT_DURATION = 0.5
#Image parameters
img_width = 640
#Node Parameters
RATE = 5
#TODO: Check to see if this works with object detection messages now
class ImageDetectorPublisherNode():
    """
    A ROS node that detects and publishes information about people in images. Works if SLAM, person detection, and person position are enabled.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        image_detection_publisher (rospy.Publisher): Publisher for the detected image with bounding boxes.
        group_publisher (rospy.Publisher): Publisher for the detected group of people.
        person_position_publisher (rospy.Publisher): Publisher for the position of detected persons.
        angle_vis_publisher (rospy.Publisher): Publisher for the angle visualization markers.
        person_marker_publisher (rospy.Publisher): Publisher for the person markers.
        tracked_person_publisher (rospy.Publisher): Publisher for the tracked person ID.
        get_group_detection_service (rospy.ServiceProxy): Service proxy for group detection.
        get_person_position_service (rospy.ServiceProxy): Service proxy for person position detection.
        bridge (CvBridge): Bridge to convert ROS images to OpenCV images.
        transform_broadcaster (tf2_ros.TransformBroadcaster): Broadcaster for TF transforms.
        tf_buffer (tf2_ros.Buffer): Buffer for TF transforms.
        tf_listener (tf2_ros.TransformListener): Listener for TF transforms.
        tracked_id (String): ID of the tracked person.
        published_pose (PoseStamped): Pose of the published person.
        starting_time (float): Start time of the node.
        time_row (list): List to keep track of time for different stages.
    Methods:
        spin(): Main loop of the node.
        append_time(label): Appends the current time with a label to the time_row.
        getImageDetection(camera_image): Processes the image from the camera, detects people, and publishes relevant information.
        uncompress_image(compressed_image): Uncompresses a compressed image.
        compress_image(cv_image): Compresses an OpenCV image.
        resize_and_add_padding(cv_image): Resizes and adds padding to an image.
        getClosestPerson(people): Gets the closest person from a list of people.
        calculate_person_position_correction(robot_pose_before, robot_pose_after): Calculates the correction to the person position based on robot movement.
        apply_pose_correction(position_point, correction): Applies correction for robot movement to the pose of the person.
        publish_image_with_bboxes(cv_image): Publishes the image with bounding boxes added.
        publish_person_marker(position_point): Publishes a marker for the detected person.
        broadcast_tf_publish_pose(position_point, person_id): Broadcasts TF and publishes the pose of the detected person.
        prepare_transform(pose_msg, person_id): Prepares a transform message for broadcasting.
        addBboxToImage(cv_image, person, position_point): Adds bounding boxes to the image.
        getPersonPoseStamped(position_point): Gets a PoseStamped message for the detected person.
        getPersonMarker(): Gets a Marker message for the detected person.
    """

    NODE_NAME = "image_detector_publisher_node"
    def __init__(self):
        rospy.init_node("image_detector_publisher_node")
        #self.image_subscriber = rospy.Subscriber('/rpi_camera/image_raw/compressed',CompressedImage, self.getImageDetection, queue_size=1)
        self.image_detection_publisher = rospy.Publisher('/person_detection_image/image_raw/compressed',CompressedImage,queue_size=10)
        self.group_publisher = rospy.Publisher('/detected_group',Group, queue_size=10)
        self.person_position_publisher = rospy.Publisher('/person_position',PoseStamped,queue_size=10)
        self.angle_vis_publisher = rospy.Publisher('/person_angle_detection',Marker,queue_size=10)
        self.person_marker_publisher = rospy.Publisher('/person_marker',Marker,queue_size=10)
        self.tracked_person_publisher = rospy.Publisher('/tracked_person',String,queue_size=10)
        self.get_group_detection_service = rospy.ServiceProxy('get_group_detection', getGroupDetection)
        self.get_person_position_service = rospy.ServiceProxy('get_person_position', getPersonPosition)
        self.bridge = CvBridge()
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration.from_sec(BUFFER_CACHE_TIME))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tracked_id = String()
        self.published_pose = PoseStamped()
        self.starting_time = time.time()
        self.time_row = []
        

    def spin(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            try:
                compressed_image = rospy.wait_for_message('/rpi_camera/image_raw/compressed',CompressedImage,timeout=rospy.Duration.from_sec(WAIT_DURATION))
                self.getImageDetection(compressed_image)
                rate.sleep()
            except Exception as e:
                rospy.logerr("Exception caught while getting image from camera " + str(e))
                rospy.logerr(traceback.print_exc())
                continue
   
    def append_time(self,label):
        row = [label,time.time()-self.starting_time]
        self.time_row.append(row)    
    
    #Method that gets the image from camera, calls the group detection service, keeps track of tracked and closest person, broadcasts tf, and 
    #publishes pose and marker for the person, and image with added bboxes
    def getImageDetection(self,camera_image):
        self.append_time("P0")
        cv_image = self.uncompress_image(camera_image)
        robot_pose_before = rospy.wait_for_message('amcl_pose',PoseWithCovarianceStamped,timeout=rospy.Duration.from_sec(WAIT_DURATION))
        padded_image = self.resize_and_add_padding(cv_image)
        self.append_time("P1")
        group = self.get_group_detection_service(self.compress_image(padded_image)).detectedPeople.group
        self.group_publisher.publish(group)
        people = {"ids": [], "positions": []}
        self.append_time("P2")
        #During the loop, adding bounding box to the image, publishing all the angle markers for each person
        for person in group:
            try:
                self.append_time("P2.1")
                person_detection_response = self.get_person_position_service(person)
                self.append_time("P2.2")
                robot_pose_after = rospy.wait_for_message('amcl_pose',PoseWithCovarianceStamped,timeout=rospy.Duration.from_sec(WAIT_DURATION))
                pose_correction = self.calculate_person_position_correction(robot_pose_before,robot_pose_after)
                #We choose to not publish the person position if the robot is rotating, since it yields to wrong estimations
                if abs(pose_correction[3]) > MAX_ROBOT_ROTATION:
                    rospy.logwarn("Yaw movement too big, not publishing pose")
                    continue
                position_point = self.apply_pose_correction(person_detection_response.position,pose_correction)
                self.append_time("P3")
                if BROADCAST_TF:
                    self.broadcast_tf_publish_pose(position_point,person.person_id)
                self.append_time("P4")
                if VISUALIZATION:
                    #Adding Bounding box to the image
                    padded_image = self.addBboxToImage(padded_image,person,position_point.point)
                    #Publishing all angle markers
                    self.angle_vis_publisher.publish(person_detection_response.angle_visualizer)
                    self.publish_person_marker(position_point)
                self.append_time("P5")
                people["ids"].append(person.person_id)
                people["positions"].append(position_point)
            except Exception as e:
                rospy.logerr("Exception caught while looping on the detected people " + str(e))
                rospy.logerr(traceback.print_exc())
        self.append_time("P5.1")
        if VISUALIZATION:
            self.publish_image_with_bboxes(padded_image)
        self.append_time("P5.2")
        if self.tracked_id.data == "" or self.tracked_id.data not in people["ids"]:
            self.tracked_id = self.getClosestPerson(people)
        self.append_time("P5.3")
        self.tracked_person_publisher.publish(self.tracked_id)
        self.append_time("P6")

    def uncompress_image(self, compressed_image):
        np_arr = np.fromstring(compressed_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        return image_np

    def compress_image(self, cv_image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
        return msg
    
    def resize_and_add_padding(self, cv_image):
        padding_y = (cv_image.shape[1] - cv_image.shape[0])/2
        padded_img = cv2.copyMakeBorder(cv_image,int(padding_y),int(padding_y),0,0,cv2.BORDER_CONSTANT)
        return padded_img
    
    def getClosestPerson(self, people):
        closest_person_id = String()
        closest_person_distance = MAX_DISTANCE
        for i in range(len(people["ids"])):
            distance = math.sqrt(people["positions"][i].point.x ** 2 + people["positions"][i].point.y)
            if distance < closest_person_distance:
                closest_person_distance = distance
                closest_person_id.data = str(people["ids"][i])
        return closest_person_id
    
     #Calculates the correction to the person position by using the changes in robot pose before and after (After - Before)
    def calculate_person_position_correction(self, robot_pose_before, robot_pose_after):
        x_movement = robot_pose_after.pose.pose.position.x - robot_pose_before.pose.pose.position.x
        y_movement = robot_pose_after.pose.pose.position.y - robot_pose_before.pose.pose.position.y
        z_movement = robot_pose_after.pose.pose.position.z - robot_pose_before.pose.pose.position.z  
        euler_before =  euler_from_quaternion([robot_pose_before.pose.pose.orientation.x,robot_pose_before.pose.pose.orientation.y,robot_pose_before.pose.pose.orientation.z,robot_pose_before.pose.pose.orientation.w])
        euler_after = euler_from_quaternion([robot_pose_after.pose.pose.orientation.x,robot_pose_after.pose.pose.orientation.y,robot_pose_after.pose.pose.orientation.z,robot_pose_after.pose.pose.orientation.w])
        yaw_movement = euler_after[2] - euler_before[2]
        rospy.loginfo("euler before: " + str(euler_before) + " euler after: " + str(euler_after) + " yaw movement: " + str(yaw_movement))
        return [x_movement,y_movement,z_movement,yaw_movement]

    #Applies correction for robot movement to the pose of the person (to check if this is correct) 
    def apply_pose_correction(self, position_point, correction):
        distance = math.sqrt(math.pow(position_point.point.x,2) + math.pow(position_point.point.y,2))
        angle = math.atan2(position_point.point.y,position_point.point.x)
        rospy.loginfo("Distance: " + str(distance) + " Angle: " + str(angle) + " Correction: " + str(correction))
        rospy.loginfo("Point before correction was: " + str(position_point))
        #Now we get the new points by correcting for the robot rotation and translation
        position_point.point.x = distance * math.cos(angle-correction[3]) - correction[0]
        position_point.point.y = distance * math.sin(angle-correction[3]) - correction[1]
        position_point.point.z -= correction[2]
        rospy.loginfo("Point after correction is: " + str(position_point))
        return position_point
    
    #Publishes the image with the bboxes added
    def publish_image_with_bboxes(self, cv_image):
        compressed_image = self.compress_image(cv_image)
        self.image_detection_publisher.publish(compressed_image)

    #Publishes person marker
    def publish_person_marker(self,position_point):
        person_marker = self.getPersonMarker()
        self.person_marker_publisher.publish(person_marker)

    #Broadcsats tf and publish pose of the position point
    def broadcast_tf_publish_pose(self, position_point, person_id):
        pose_msg = self.getPersonPoseStamped(position_point)
        pose_msg_map_frame = self.tf_buffer.transform(pose_msg,FIXED_FRAME_ID,rospy.Duration.from_sec(WAIT_DURATION))
        #Check if this is alright changing the time to now before publishing to avoid publishing a past pose
        pose_msg_map_frame.header.stamp = position_point.header.stamp
        t = self.prepare_transform(pose_msg_map_frame,person_id)
        self.transform_broadcaster.sendTransform(t)
        #Publishing pose in the frame of the person
        pose_msg_person_frame = PoseStamped()
        pose_msg_person_frame.header.frame_id = PERSON_FRAME_ID + "_" + str(person_id)
        self.person_position_publisher.publish(pose_msg_person_frame)
        self.published_pose = pose_msg_person_frame
    
    def prepare_transform(self, pose_msg, person_id):
        t = tf2_ros.TransformStamped()
        t.header.stamp = pose_msg.header.stamp
        t.header.frame_id = FIXED_FRAME_ID
        t.child_frame_id = PERSON_FRAME_ID + "_" + str(person_id)
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        t.transform.rotation.x = pose_msg.pose.orientation.x
        t.transform.rotation.y = pose_msg.pose.orientation.y
        t.transform.rotation.z = pose_msg.pose.orientation.z
        t.transform.rotation.w = pose_msg.pose.orientation.w
        return t
    
                
    #Adds bbox of the person to the image and returns it 
    def addBboxToImage(self, cv_image, person,position_point):
        cv_image = cv2.rectangle(cv_image, (int(person.bbox[0]), int(person.bbox[1])), (int(person.bbox[2]), int(person.bbox[3])), (36,255,12), 1)
        cv_image = cv2.putText(cv_image, person.class_id + " " + str(person.confidence),[int(person.bbox[0]),int(person.bbox[3])],cv2.FONT_HERSHEY_SIMPLEX,0.9,(35,255,12),2)
        cv_image = cv2.putText(cv_image,"("+str(round(position_point.x,2)) +","+str(round(position_point.y,2))+")",[int(person.bbox[0]),int(person.bbox[1])],cv2.FONT_HERSHEY_SIMPLEX,0.9,(255,0,0),2)
        return cv_image


    def getPersonPoseStamped(self, position_point):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = position_point.point.x
        pose_msg.pose.position.y = position_point.point.y
        quaternion = quaternion_from_euler(0,0,math.pi)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        pose_msg.header.frame_id = ORIGIN_FRAME_ID
        pose_msg.header.stamp = position_point.header.stamp
        return pose_msg

    def getPersonMarker(self):
        marker = Marker()
        marker.header.frame_id = self.published_pose.header.frame_id
        marker.pose.position.x = self.published_pose.pose.position.x
        marker.pose.position.y = self.published_pose.pose.position.y
        marker.pose.position.z = Z_POSITION
        marker.type = MARKER_TYPE
        marker.color.r = 1
        marker.color.a = 1 
        marker.scale.x = X_SCALE
        marker.scale.y = Y_SCALE
        marker.scale.z = Z_SCALE
        marker.lifetime = rospy.Duration.from_sec(MARKER_DURATION)
        return marker        

if __name__ == '__main__':
    rospy.loginfo("AO")
    node = ImageDetectorPublisherNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    node.spin()
    df = pd.DataFrame(node.time_row, columns =['label', 'time'])
    df.to_csv('~/image_detector_publisher_node.csv')
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
