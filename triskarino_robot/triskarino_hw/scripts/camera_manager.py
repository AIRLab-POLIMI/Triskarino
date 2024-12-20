#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np 
from std_msgs.msg import String

"""
camera_manager.py
This script defines a ROS node for managing a camera using OpenCV and publishing the captured images as ROS messages.
Classes:
    CameraManagerNode: A class to manage the camera, capture images, and publish them as ROS messages.
Constants:
    flip (int): Flip method for the camera.
    dispW (int): Width of the published image.
    dispH (int): Height of the published image.
    RATE (int): Rate of publishing images in Hz.
    CAM_SET (str): Gstreamer pipeline string for camera settings.
Functions:
    CameraManagerNode.__init__(self, cam_set=CAM_SET): Initializes the CameraManagerNode, sets up the camera, and starts capturing images.
    CameraManagerNode.capture(self): Captures images from the camera and publishes them as ROS messages.
Usage:
    Run this script or use rosrun to start the camera manager node which captures images and publishes them to the specified ROS topics.
"""

#dispW is the width of the published image, dispH is the height of the published image, rate is the rate of publishing in hz
flip=2
dispW=640
dispH=360
RATE=40
#Gstreaming camera settings, do not change!
CAM_SET = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=59/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
class CameraManagerNode():
    NODE_NAME = "camera_manager"
    def __init__(self,cam_set=CAM_SET):
        rospy.init_node("camera_manager")
        self.cam= cv2.VideoCapture(cam_set)
        if not self.cam.isOpened():
            rospy.logwarn("CAM NOT OPEN")
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher('/rpi_camera/image_raw/compressed',CompressedImage, queue_size=10)
        self.counter = 0
        self.name = String()
        self.name_publisher = rospy.Publisher('/rpi_camera/image_name',String, queue_size=10)
        self.capture()
        
    def capture(self):
        rospy.loginfo("Capturing images...")
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            ret, frame = self.cam.read()
            if ret == True:
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = str(self.counter)
                msg.format = "jpeg"
                msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
                self.name.data = str(self.counter)
                self.image_publisher.publish(msg)
                self.name_publisher.publish(self.name)
                self.counter += 1
            else:
                rospy.logwarn("Image not published, RET was false")
            rate.sleep()
        self.cam.release()
        cv2.destroyAllWindows()
   
if __name__ == '__main__':
    rospy.loginfo("AO")
    node = CameraManagerNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
