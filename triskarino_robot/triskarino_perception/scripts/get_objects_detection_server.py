#!/usr/bin/env python3
import rospy
from triskarino_perception.srv import getObjectsDetection
from ultralytics import YOLO
from triskarino_msgs.msg import Object, Objects
import rospkg 
import torch 
from cv_bridge import CvBridge
import numpy as np
import cv2



MODEL_NAME = "yolo_objects_detection_v2.pt"
rospack = rospkg.RosPack()
CONFIDENCE_CUTOFF_TOY = 0.45
CONFIDENCE_CUTOFF_PERSON = 0.3
# Stick color is    rgb(198,240,155)
h,s,v = 90, 35, 94

hue_range = 140
sat_range = 30
val_range = 130

STICK_COLOR_LOWER = np.array([h-hue_range, s-sat_range, v-val_range])
STICK_COLOR_UPPER = np.array([h+hue_range, s+sat_range, v+val_range])

class ObjectsDetectionServer():
    """
    A ROS service server for detecting objects using a YOLO model. The model currently detects
    the tennis ball at the end of the stick (toy), the stick, and people as seen from the robot camera.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        device (str): The device to run the model on ('cuda' or 'cpu').
        model (YOLO): The YOLO model for object detection.
        bridge (CvBridge): The bridge to convert ROS images to OpenCV images.
    Methods:
        __init__(model_name=MODEL_NAME):
            Initializes the ROS node, loads the YOLO model, and sets up the service.
        filter_boxes(boxes):
            Filters the detected boxes to keep only those with confidence above a threshold.
        enlarge_bbox(bbox, factor=0.0, max_width=640, max_height=640):
            Enlarges a bounding box by a given factor.
        find_longest_contour(contours):
            Finds the longest contour from a list of contours.
        fix_stick_box(box, cv_image):
            Adjusts the bounding box for a stick object by processing the image.
        filter_boxes_based_on_type(boxes):
            Filters the detected boxes based on their type (toy, stick, person).
        convert_xyxy_to_bbox(box):
            Converts a bounding box from xyxy format to a list of coordinates.
        handle_get_objects_detection(srv_request):
            Handles the ROS service request for object detection.
        uncompress_image(compressed_image):
            Uncompresses a ROS compressed image to an OpenCV image.
    """

    NODE_NAME = "objects_detection_server"
    def __init__(self,model_name=MODEL_NAME):
        rospy.init_node("get_objects_detection_server")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        rospy.loginfo("Using device " + str(self.device))
        self.model = YOLO(str(rospack.get_path('triskarino_perception'))+"/resources/"+model_name)

        s = rospy.Service('get_objects_detection',getObjectsDetection, self.handle_get_objects_detection)
        self.bridge = CvBridge()
    
    #Idea: Keep only boxes with confidence > least_confidence, if there are multiple boxes keep the one with maximum confidence
    def filter_boxes(self,boxes):
        if len(boxes) != 0:
            max_confidence_box = max(boxes, key=lambda x: x.conf)
        else:
            return None
        if max_confidence_box.conf > CONFIDENCE_CUTOFF_TOY:
            return [max_confidence_box]
        else:
            return None

    def enlarge_bbox(self, bbox, factor = 0.0, max_width = 640, max_height=640):
        x1, y1, x2, y2 = bbox.xyxy[0]
        width = x2 - x1
        height = y2 - y1
        x1 = max(x1 - width * factor, 0)
        x2 = max(x2 + width * factor, max_width)
        y1 = max(y1 - height * factor, 0)
        y2 = max(y2 + height * factor, max_height)
        return (x1, y1, x2, y2)
    
    def find_longest_contour(self, contours):
        max_length = -1
        # Loop through the contours and find the longest
        for contour in contours:
            length = cv2.arcLength(contour, True)  # True indicates the contour is closed
            if length > max_length:
                max_length = length
                longest_contour = contour
        return longest_contour
    
    #Idea: Make every stick box oblique and further processing the cv_image 
    def fix_stick_box(self, box, cv_image):
        enlarged_bbox = self.enlarge_bbox(box)
        cropped_image = cv_image[int(enlarged_bbox[1]):int(enlarged_bbox[3]), int(enlarged_bbox[0]):int(enlarged_bbox[2])]
        #Get the part of the image of the bounding box
        hsv_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
        # Define the range of yellowish colors in HSV
        mask = cv2.inRange(hsv_image, STICK_COLOR_LOWER, STICK_COLOR_UPPER)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # Find contours in the mask
        contours, _ = cv2.findContours(~mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        longest_contour = self.find_longest_contour(contours)
        rect = cv2.minAreaRect(longest_contour)
        new_box = cv2.boxPoints(rect)
        new_box = np.int0(new_box)
        rospy.loginfo("New Box is " + str(new_box))
        rospy.loginfo("Old bbox was " + str(box.xyxy[0]))
        converted_new_box = torch.Tensor([[new_box[x][0] + int(enlarged_bbox[0]), new_box[x][1] + int(enlarged_bbox[1])] for x in range(4)])
        rospy.loginfo("Converted New Box is " + str(converted_new_box))
        bbox_flat = [coordinate for pair in converted_new_box for coordinate in pair]
        return bbox_flat
            
    
    #Only keep one box if the type is toy or stick, but keep all the boxes for people
    def filter_boxes_based_on_type(self,boxes):
        if len(boxes) != 0:
            #Get all the boxes that have class toy and keep the one with Maximum Confidence
            toy_boxes = self.filter_boxes([box for box in boxes if self.model.names[int(box.cls)] == "toy"])
            #Get all the boxes that have class stick and keep the one with Maximum Confidence
            stick_boxes = self.filter_boxes([box for box in boxes if self.model.names[int(box.cls)] == "stick"])
            #Get all the boxes that have class person and keep the ones that have confidence above the cutoff
            person_boxes = [box for box in boxes if (self.model.names[int(box.cls)] == "person") and (box.conf >= CONFIDENCE_CUTOFF_PERSON)]
            filtered_boxes = []
            if toy_boxes != None:
                filtered_boxes = filtered_boxes + toy_boxes
            if stick_boxes != None:
                filtered_boxes = filtered_boxes + stick_boxes
            if person_boxes != None:
                filtered_boxes = filtered_boxes + person_boxes
            return filtered_boxes
        else: 
            return []


    def convert_xyxy_to_bbox(self, box):
        x1, y1, x2, y2 = box.xyxy[0]
        return [x1, y1, x2, y1, x2, y2, x1, y2]

    def handle_get_objects_detection(self,srv_request):
        cv_image = self.uncompress_image(srv_request.cameraImage)
        results = self.model.track(cv_image)
        print("Model Classes are " + str(self.model.names))

        objects = Objects()
        #Extracting detections for Person Class
        for result in results:
            boxes = result.boxes.cpu()
            filtered_boxes = self.filter_boxes_based_on_type(boxes)
            if len(filtered_boxes) == 0:
                return objects
            else:
                for box in filtered_boxes:
                    prediction_class = str(self.model.names[int(box.cls)])
                    if prediction_class == "stick":
                        bbox = self.fix_stick_box(box, cv_image)
                    else:
                        bbox = self.convert_xyxy_to_bbox(box)
                    confidence = round(float(box.conf),3)
                    object = Object()
                    object.bbox = bbox             
                    object.class_id = prediction_class
                    object.confidence = confidence
                    objects.objects.append(object)
        return objects
    
    def uncompress_image(self, compressed_image):
        np_arr = np.fromstring(compressed_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        return image_np
        

if __name__ == '__main__':
    rospy.loginfo("AO")
    node = ObjectsDetectionServer()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
