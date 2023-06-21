#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

#dispW is the width of the published image, dispH is the height of the published image, rate is the rate of publishing in hz
flip=2
dispW=320
dispH=240
RATE=5
#Gstreaming camera settings, do not change!
CAM_SET = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
class CameraManagerNode():
    NODE_NAME = "camera_manager"
    def __init__(self,cam_set=CAM_SET):
        rospy.init_node("camera_manager")
        self.cam= cv2.VideoCapture(cam_set)
        if not self.cam.isOpened():
            rospy.logwarn("CAM NOT OPEN")
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher('/rpi_camera',Image, queue_size=10)
        self.capture()
   
    def capture(self):
        rospy.loginfo("Capturing images...")
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            ret, frame = self.cam.read()
            if ret == True:
                msg= self.bridge.cv2_to_imgmsg(frame,"bgr8")
                self.image_publisher.publish(msg)
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
