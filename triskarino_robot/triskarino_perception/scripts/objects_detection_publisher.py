#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from triskarino_perception.srv import getObjectsDetection
from triskarino_msgs.msg import Objects

#Image parameters
img_width = 640
#Node Parameters
RATE = 15
WAIT_DURATION = 0.5


class ObjectsDetectionPublisherNode():
    """
    A ROS node for detecting objects (currently toy, stick and person) in images and publishing the results.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        objects_publisher (rospy.Publisher): Publisher for detected objects.
        get_objects_detection_service (rospy.ServiceProxy): Service proxy for object detection.
        objects_detection_publisher (rospy.Publisher): Publisher for images with bounding boxes.
    Methods:
        __init__(): Initializes the ROS node and sets up publishers and service proxies.
        spin(): Main loop that waits for images, processes them, and publishes detected objects.
        publishObjects(camera_image): Processes the camera image to detect objects and publishes the results.
        uncompress_image(compressed_image): Uncompresses a compressed image message to an OpenCV image.
        convert_bbox_flat(bbox_flat): Converts a flat bounding box array to a 2D array.
        addBboxToImage(cv_image, objects): Adds bounding boxes to the image and returns the modified image.
        publish_image_with_bboxes(cv_image): Publishes the image with bounding boxes added.
        compress_image(cv_image): Compresses an OpenCV image to a ROS compressed image message.
    """

    NODE_NAME = "objects_detection_publisher_node"
    def __init__(self):
        rospy.init_node("objects_detection_publisher_node")
        self.objects_publisher = rospy.Publisher('/detected_objects',Objects, queue_size=10)
        self.get_objects_detection_service = rospy.ServiceProxy('get_objects_detection', getObjectsDetection)
        self.objects_detection_publisher = rospy.Publisher('/objects_detection_image/compressed',CompressedImage, queue_size=1)
    

    def spin(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            try:
                compressed_image = rospy.wait_for_message('/rpi_camera/image_raw/compressed',CompressedImage,timeout=rospy.Duration.from_sec(WAIT_DURATION))
                self.publishObjects(compressed_image)
                rate.sleep()
            except Exception as e:
                rospy.logerr("Exception caught while getting image from camera " + str(e))
                continue
   
    def publishObjects(self,camera_image):
        try:
            cv_image = self.uncompress_image(camera_image)
            objects = self.get_objects_detection_service(self.compress_image(cv_image)).detections
            self.objects_publisher.publish(objects)
            self.publish_image_with_bboxes(self.addBboxToImage(cv_image,objects.objects))
        except Exception as e:
            rospy.logerr("Exception caught while trying to use objects detection service, service not ready just wait " + str(e))
      
        
    def uncompress_image(self, compressed_image):
        np_arr = np.fromstring(compressed_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        return image_np

    def convert_bbox_flat(self, bbox_flat):
        return np.array([[int(bbox_flat[i]), int(bbox_flat[i+1])] for i in range(0, len(bbox_flat), 2)]).reshape((-1, 1, 2))
    
    #Adds bbox of the person to the image and returns it 
    def addBboxToImage(self, cv_image,objects):
        for object in objects:
            rospy.loginfo("BBOX AFTER CONVERSION IS " + str([self.convert_bbox_flat(object.bbox)]))
            cv2.drawContours(cv_image,[self.convert_bbox_flat(object.bbox)],0,(36,255,12),1)
            #cv_image = cv2.rectangle(cv_image, (int(object.bbox[0]), int(object.bbox[1])), (int(object.bbox[2]), int(object.bbox[3])), (36,255,12), 1)
            cv_image = cv2.putText(cv_image, object.class_id + " " + str(object.confidence),[int(object.bbox[0]),int(object.bbox[3])],cv2.FONT_HERSHEY_SIMPLEX,0.9,(35,255,12),2)
        return cv_image
    
    #Publishes the image with the bboxes added
    def publish_image_with_bboxes(self, cv_image):
        compressed_image = self.compress_image(cv_image)
        self.objects_detection_publisher.publish(compressed_image)

    def compress_image(self, cv_image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
        return msg
  
if __name__ == '__main__':
    rospy.loginfo("AO")
    node = ObjectsDetectionPublisherNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    node.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
