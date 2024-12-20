cmake_minimum_required(VERSION 3.0.2)
project(triskarino_description)


catkin_package()

find_package(catkin REQUIRED COMPONENTS
  rviz
  urdf
  gazebo
)

catkin_install_python(PROGRAMS 
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
