cmake_minimum_required(VERSION 3.0.2)
project(triskarino_action_controller)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
    rospy 
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf 
    triskarino_msgs
    triskarino_hw
    triskarino_perception
    py_trees
    py_trees_ros
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/social_movement_reconfigure.cfg
  cfg/chase_reconfigure.cfg
)

catkin_package()


catkin_install_python(PROGRAMS ./src/bt_actions/play_sound.py ./src/bt_actions/sonar_to_blackboard.py ./src/bt_actions/obstacle_distance_to_blackboard.py ./src/bt_actions/objects_to_blackboard.py ./src/bt_actions/behavior_master.py ./scripts/autonomous_behavior_tree.py ./src/utility/person_chase_predictor.py ./scripts/chase_person.py ./src/bt_actions/move_linearly_odom.py ./src/bt_actions/rotate_robot_odom.py ./src/bt_actions/move_linearly_odom.py ./scripts/social_movement_tree.py ./scripts/chase_toy.py ./src/utility/toy_chase_predictor.py ./src/utility/chase_utilities.py ./scripts/find_and_approach_person.py ./src/bt_actions/move_base_status_to_blackboard.py ./src/bt_actions/robot_state_to_blackboard.py ./src/bt_actions/approach_person.py ./src/bt_actions/group_to_blackboard.py ./src/bt_actions/light_animation.py ./src/bt_actions/person_pose_to_blackboard.py ./src/bt_actions/rotate_robot.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




include_directories(
  ${catkin_INCLUDE_DIRS}
)
# EDIT: list all executable python and shell scripts for installation
install(PROGRAMS
   scripts/*
   src/bt_actions/*
   src/utility/*
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


