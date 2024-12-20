cmake_minimum_required(VERSION 3.0.2)
project(triskarino_hw)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
    rospy 
    geometry_msgs
    nav_msgs
    sensor_msgs
    message_generation
    tf 
    triskarino_msgs
    cv_bridge
    sensor_msgs
    rosserial_python
    laser_filters
    robot_pose_ekf
)

generate_dynamic_reconfigure_options(
  cfg/MyParams.cfg
)


catkin_install_python(PROGRAMS ./scripts/lidar_movement_blocker.py ./scripts/image_saver.py ./scripts/debug_arduino.py ./scripts/speed_manager.py ./scripts/odom_manager.py ./scripts/camera_manager.py ./scripts/teleop_manager.py ./scripts/lights_manager.py ./scripts/sound_manager.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# EDIT: list all executable python and shell scripts for installation
install(PROGRAMS
   scripts/*
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
