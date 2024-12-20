#!/usr/bin/env python3
##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
from bt_actions.cmd_to_blackboard import CmdToBlackboard
from bt_actions.move_linearly_odom import MoveLinOdom
from bt_actions.rotate_robot_odom import RotateRobotOdom
from dynamic_reconfigure.server import Server
from triskarino_action_controller.cfg import social_movement_reconfigureConfig

##############################################################################
# Behaviours
##############################################################################


param_choice = {"person_rot_angle": 10,
               "random_rot_angle": 60, 
               "mov_dist": 0.5, 
               "lin_mov_max_speed": 0.3, 
               "person_rot_mov_max_speed": 1.0, 
               "random_rot_mov_max_speed": 0.5 }

active_tree = None

def create_root(param_dict):
    cmd_to_blackboard = CmdToBlackboard(name="CmdToBlackboard", topic_name="cmd_chase")
    social_movement_is_on = py_trees.blackboard.CheckBlackboardVariable(
        name="cmd_to_blackboard",
        variable_name='on',
        expected_value=True
    )
    root = py_trees.composites.Parallel(name="SocialMovementParallel")
    sequence = py_trees.composites.Sequence(name="SocialMovementSequence")
    rotate_random = RotateRobotOdom(name="RotateRandom", angle_of_rotation=-param_dict["random_rot_angle"], max_speed_of_rotation=-param_dict["random_rot_mov_max_speed"])
    move_linearly = MoveLinOdom(name="MoveLinearly", distance=param_dict["mov_dist"], max_speed=param_dict["lin_mov_max_speed"])
    rotate_person_0 = RotateRobotOdom(name="RotatePerson_0", angle_of_rotation=param_dict["person_rot_angle"], max_speed_of_rotation=param_dict["person_rot_mov_max_speed"])
    rotate_person_1 = RotateRobotOdom(name="RotatePerson_1", angle_of_rotation=-param_dict["person_rot_angle"], max_speed_of_rotation=-param_dict["person_rot_mov_max_speed"])
    rotate_person_2 = RotateRobotOdom(name="RotatePerson_2", angle_of_rotation=param_dict["person_rot_angle"], max_speed_of_rotation=param_dict["person_rot_mov_max_speed"])
    root.add_children([cmd_to_blackboard, sequence])
    sequence.add_children([social_movement_is_on, rotate_random, move_linearly,rotate_person_0, rotate_person_1, rotate_person_2])
    return root

##############################################################################
# Main
##############################################################################

def shutdown(behaviour_tree):  
    if behaviour_tree is not None:
        behaviour_tree.interrupt()

def create_and_start_tree(param_dict,tick=200):
    root = create_root(param_dict)
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.loginfo("Before Setting up tree")
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        rospy.logerr("Failed to setup behavior tree")
        sys.exit(1)
    rospy.loginfo("Tree setup correctly")
    behaviour_tree.tick_tock(tick)
    return behaviour_tree

def restart_tree_with_new_param(config,level):
    global active_tree
    global param_choice
    if active_tree is not None:
        active_tree.interrupt()
    #param_choice['person_rot_angle'] = config.person_rot_angle
    #param_choice['random_rot_angle'] = config.random_rot_angle
    #param_choice['mov_dist'] = config.mov_dist
    #param_choice['lin_mov_max_speed'] = config.lin_mov_max_speed
    #param_choice['person_rot_mov_max_speed'] = config.person_rot_mov_max_speed
    #param_choice['random_rot_mov_max_speed'] = config.random_rot_mov_max_speed
    rospy.loginfo("New parameters: " + str(param_choice))
    active_tree = create_and_start_tree(param_choice, 200)
    return config

if __name__ == "__main__":
    """
    Entry point for the demo script.
    """
    rospy.init_node("tree_manager")
    srv = Server(social_movement_reconfigureConfig, restart_tree_with_new_param)
    rospy.on_shutdown(functools.partial(shutdown, active_tree))

    