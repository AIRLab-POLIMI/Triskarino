# Triskarino Perception (triskarino_perception)
This folder contains the services and nodes that do object detection (for now for the stick toy and person) and touch classification 

## Object Detection
The get_objects_detection_server uses a custom trained YOLO model to detect the stick toy object and people. The server is called by the objects_detection_publisher node which then publishes all the detection on the detected_objects topic

## Touch Classification
The touch classification works with two different servers static_classifier (used when the robot is not moving) and dynamic classifier (used when the robot is moving). The static classifier classifies all types of touch while the dynamic classifier only classifies if the robot was touched or not. 
These two services are called by the touchclassifier_manager.py which then publishes the result on the 
touchclassification topic

## Legacy Services
In this folder there are also other services like the one that identifies the position of the person combining LIDAR and camera and the one that given this position decides a navigation goal for the robot. These are currently unused and work only if SLAM is on, but they could be reworked into something useful. 



