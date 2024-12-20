cmake_minimum_required(VERSION 3.0.2)
project(triskarino_perception)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
    rospy 
    triskarino_msgs
    triskarino_hw
    cv_bridge
    sensor_msgs
    message_generation
    geometry_msgs
    std_msgs
    visualization_msgs
)

add_service_files(
	FILES
    getPersonPosition.srv
    getPersonNavGoal.srv
    getObjectsDetection.srv
)

generate_messages( DEPENDENCIES triskarino_msgs sensor_msgs geometry_msgs std_msgs visualization_msgs)
catkin_package()

catkin_install_python(PROGRAMS ./scripts/touchclassifier_manager.py ./scripts/dynamic_detector_server.py ./scripts/static_classifier_server.py ./scripts/objects_detection_publisher.py ./scripts/get_objects_detection_server.py ./scripts/get_person_position_server.py ./scripts/get_person_nav_goal_server.py ./scripts/image_detector_publisher.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




include_directories(
  ${catkin_INCLUDE_DIRS}
)
# EDIT: list all executable python and shell scripts for installation
install(PROGRAMS
   scripts/*
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


