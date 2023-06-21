cmake_minimum_required(VERSION 3.0.2)
project(triskarino_hw)

catkin_python_setup()
catkin_package()

find_package(catkin REQUIRED COMPONENTS
    rospy 
    geometry_msgs
    nav_msgs
    tf 
    triskarino_msgs
    cv_bridge
    sensor_msgs
)


catkin_install_python(PROGRAMS ./scripts/odom_manager.py ./scripts/camera_manager.py ./scripts/teleop_manager.py ./scripts/lights_manager.py ./scripts/sound_manager.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
# EDIT: list all executable python and shell scripts for installation
install(PROGRAMS
   src/*
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


