#!/usr/bin/env python3
'''
This script defines and runs an autonomous behavior tree for the Triskarino robot using the py_trees library.
With this behavior tree the robot will first randomly move in the environment, if a person is seen it will approach the person.
If the toy is seen it will chase the toy, if the toy is lost it will look for the toy, if an obstacle is detected it will avoid the obstacle.
Functions:
    create_leaf_nodes(): Creates and returns a dictionary of all the leaf nodes used in the behavior tree.
    create_condition_checking_sequence(leaf_nodes): Creates a sequence that checks various conditions by inverting blackboard subscriptions.
    create_fallback_with_guard(guard, sequence_nodes, name): Creates a fallback structure with a guard condition and a sequence of nodes.
    create_root(): Creates and returns the root of the behavior tree, combining all sequences and fallbacks.
    shutdown(behaviour_tree): Shuts down the behavior tree by interrupting it.
Main:
    Initializes the ROS node, creates the behavior tree, sets up the tree, and starts the tick-tock mechanism to run the tree.
'''
##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
from bt_actions.objects_to_blackboard import ObjectsToBlackboard
from bt_actions.light_animation import LightAnimation
from bt_actions.rotate_robot_odom import RotateRobotOdom
from bt_actions.move_linearly_odom import MoveLinOdom
from bt_actions.behavior_master import BehaviorMaster
from bt_actions.cmd_to_blackboard import CmdToBlackboard
from bt_actions.obstacle_distance_to_blackboard import ObstacleDistanceToBlackboard
from bt_actions.sonar_to_blackboard import SonarToBlackboard
from bt_actions.play_sound import PlaySound
##############################################################################
# Behaviours
##############################################################################

#Creating all the leaf nodes for the tree
def create_leaf_nodes():
    #TOBLACKBOARD SUBS
    objects_to_bb = ObjectsToBlackboard(name="objects_to_bb")
    cmd_to_bb = CmdToBlackboard(name="cmd_to_bb")
    obstacle_distance_to_bb = ObstacleDistanceToBlackboard(name="obstacle_distance_to_bb")
    sonar_to_bb = SonarToBlackboard(name="sonar_to_bb")
    #BEHAVIOR MASTER
    behavior_master = BehaviorMaster(name="behavior_master",toy_is_lost_time=1, toy_not_found_time=5, time_toy_seen=1,time_chasing_person=3)
    #ACTION NODES -> Social Walk
    social_walk_light = LightAnimation(name="SocialWalkLight",light_action_type="X",light_color=(0,0,255),light_delay=500,brightness=50)
    rotate_random_social = RotateRobotOdom(name="RotateRandom", angle_of_rotation=-60, max_speed_of_rotation=-0.4)
    move_linearly_social = MoveLinOdom(name="MoveLinearly", distance=0.6, max_speed=0.3)
    rotate_person_social_0 = RotateRobotOdom(name="RotatePerson_0", angle_of_rotation=10, max_speed_of_rotation=1.0)
    rotate_person_social_1 = RotateRobotOdom(name="RotatePerson_1", angle_of_rotation=-10, max_speed_of_rotation=-1.0)
    rotate_person_social_2 = RotateRobotOdom(name="RotatePerson_2", angle_of_rotation=10, max_speed_of_rotation=1.0)
    #ACTION NODES -> Looking for toy
    sad_sound = PlaySound(name="SadSound",sound_volume=1.0,sound_filepath="sad.wav")
    looking_for_toy_light = LightAnimation(name="LookingForToyLight",light_action_type="X",light_color=(255,0,0),light_delay=500,brightness=50)
    rotate_looking_for_toy_0 = RotateRobotOdom(name="RotateLookingForToy_0", angle_of_rotation=6, max_speed_of_rotation=1.2)
    rotate_looking_for_toy_1 = RotateRobotOdom(name="RotateLookingForToy_1", angle_of_rotation=-6, max_speed_of_rotation=-1.2)
    rotate_looking_for_toy_2 = RotateRobotOdom(name="RotateLookingForToy_2", angle_of_rotation=8, max_speed_of_rotation=1.2)
    move_linearly_looking_for_toy = MoveLinOdom(name="MoveLinearlyLookingForToy", distance=0.3, max_speed=0.3)
    #ACTION NODES -> Chasing Toy
    chasing_toy_light = LightAnimation(name="ChasingToyLight",light_action_type="X",light_color=(0,255,0),light_delay=500,brightness=50)
    #ACTION NODES -> Chasing Person
    chasing_person_light = LightAnimation(name="ChasingPersonLight",light_action_type="X",light_color=(0,255,255),light_delay=500,brightness=50)
    #ACTION NODES -> Obstacle Detected
    obstacle_detected_light = LightAnimation(name="ObstacleDetectedLight",light_action_type="X",light_color=(255,255,0),light_delay=500,brightness=50)
    move_linearly_obstacle = MoveLinOdom(name="MoveLinearlyLookingForToy", distance=0.2, max_speed=-0.3)
    rotate_obstacle_detected = RotateRobotOdom(name="RotateObstacleDetected", angle_of_rotation=-180, max_speed_of_rotation=-0.7)
    #ACTION NODES -> Back Obstacle Detected while chasing toy
    is_back_obstacle_detected_toy_light = LightAnimation(name="BackObstacleDetectedToyLight",light_action_type="X",light_color=(255,182,193),light_delay=500,brightness=50)
    #GUARD SOCIAL WALK 
    is_robot_looking_for_person = py_trees.blackboard.CheckBlackboardVariable(
        name="is_robot_looking_for_person",
        variable_name='robot_state',
        expected_value="looking_for_person"
    )
    #GUARD CHASING TOY
    is_robot_chasing_toy = py_trees.blackboard.CheckBlackboardVariable(
        name="is_robot_chasing_toy",
        variable_name='robot_state',
        expected_value="chasing_toy"
    )
    #GUARD LOOKING FOR TOY
    is_robot_looking_for_toy = py_trees.blackboard.CheckBlackboardVariable(
        name="is_robot_looking_for_toy",
        variable_name='robot_state',
        expected_value="looking_for_toy"
    )

    #GUARD CHASING PERSON
    is_robot_chasing_person = py_trees.blackboard.CheckBlackboardVariable(
        name="is_robot_chasing_person",
        variable_name='robot_state',
        expected_value="chasing_person"
    )

    #GUARD OBSTACLE DETECTED
    is_obstacle_detected = py_trees.blackboard.CheckBlackboardVariable(
        name="is_obstacle_detected",
        variable_name='robot_state',
        expected_value="obstacle_detected"
    )

    #IS BACK OBSTACLE DETECTED WHILE CHASING TOY
    is_back_obstacle_detected_toy = py_trees.blackboard.CheckBlackboardVariable(
        name="is_back_obstacle_detected_toy",
        variable_name='robot_state',
        expected_value="back_obstacle_detected_toy"
    )

 
    return {
        'objects_to_bb': objects_to_bb,
        'rotate_random_social': rotate_random_social,
        'move_linearly_social': move_linearly_social,
        'rotate_person_social_0':rotate_person_social_0,
        'rotate_person_social_1': rotate_person_social_1,
        'rotate_person_social_2': rotate_person_social_2,
        'social_walk_light': social_walk_light,
        'looking_for_toy_light': looking_for_toy_light,
        'rotate_looking_for_toy_0': rotate_looking_for_toy_0,
        'rotate_looking_for_toy_1': rotate_looking_for_toy_1,
        'rotate_looking_for_toy_2': rotate_looking_for_toy_2,
        'move_linearly_looking_for_toy': move_linearly_looking_for_toy,
        'chasing_toy_light': chasing_toy_light,
        'chasing_person_light': chasing_person_light,
        'is_robot_looking_for_person': is_robot_looking_for_person,
        'is_robot_chasing_toy': is_robot_chasing_toy,
        'is_robot_looking_for_toy': is_robot_looking_for_toy,
        'is_robot_chasing_person': is_robot_chasing_person,
        'behavior_master': behavior_master,
        'cmd_to_bb': cmd_to_bb,
        'obstacle_distance_to_bb': obstacle_distance_to_bb,
        'is_obstacle_detected': is_obstacle_detected,
        'obstacle_detected_light': obstacle_detected_light,
        'rotate_obstacle_detected': rotate_obstacle_detected,
        'sonar_to_bb': sonar_to_bb,
        'is_back_obstacle_detected_toy': is_back_obstacle_detected_toy,
        'is_back_obstacle_detected_toy_light': is_back_obstacle_detected_toy_light,
        'sad_sound_lost_toy': sad_sound,
        'move_linearly_obstacle': move_linearly_obstacle
    }

def create_condition_checking_sequence(leaf_nodes):
    # Invert all the blackboard subscriptions
    condition_checking_sequence = py_trees.composites.Sequence(name="condition_checking_sequence")
    inverted_behavior_master = py_trees.decorators.Inverter(leaf_nodes['behavior_master'],name="inverter_behavior_master")

    # Create a parallel with all the inverted blackboard subscriptions
    condition_checking_sequence.add_children([
        leaf_nodes['cmd_to_bb'],
        leaf_nodes['objects_to_bb'],
        leaf_nodes['obstacle_distance_to_bb'],
        leaf_nodes['sonar_to_bb'],
        inverted_behavior_master
    ])

    return condition_checking_sequence

def create_fallback_with_guard(guard, sequence_nodes,name):
    fallback_with_guard = py_trees.composites.Selector(name=name)
    inverted_guard = py_trees.decorators.Inverter(guard)
    sequence = py_trees.composites.Sequence(name=name + "_sequence")
    sequence.add_children(sequence_nodes)
    fallback_with_guard.add_children([inverted_guard, sequence])
    return fallback_with_guard


def create_root():
    leaf_nodes = create_leaf_nodes()
    condition_checking_sequence = create_condition_checking_sequence(leaf_nodes)
    social_walk_fallback = create_fallback_with_guard(leaf_nodes['is_robot_looking_for_person'], [leaf_nodes['social_walk_light'],leaf_nodes['rotate_random_social'],leaf_nodes['move_linearly_social'],leaf_nodes['rotate_person_social_0'],leaf_nodes['rotate_person_social_1'],leaf_nodes['rotate_person_social_2']],"social_walk_fallback")
    looking_for_toy_fallback = create_fallback_with_guard(leaf_nodes['is_robot_looking_for_toy'], [leaf_nodes['sad_sound_lost_toy'],leaf_nodes['looking_for_toy_light'],leaf_nodes['rotate_looking_for_toy_0'],leaf_nodes['rotate_looking_for_toy_1'],leaf_nodes['rotate_looking_for_toy_2'],leaf_nodes['move_linearly_looking_for_toy']],"looking_for_toy_fallback")
    chasing_toy_fallback = create_fallback_with_guard(leaf_nodes['is_robot_chasing_toy'], [leaf_nodes['chasing_toy_light']],"chasing_toy_fallback")
    chasing_person_fallback = create_fallback_with_guard(leaf_nodes['is_robot_chasing_person'], [leaf_nodes['chasing_person_light']],"chasing_person_fallback")
    obstacle_detected_fallback = create_fallback_with_guard(leaf_nodes['is_obstacle_detected'], [leaf_nodes['obstacle_detected_light'],leaf_nodes['move_linearly_obstacle'],leaf_nodes['rotate_obstacle_detected']],"obstacle_detected_fallback")
    back_obstacle_detected_toy_fallback = create_fallback_with_guard(leaf_nodes['is_back_obstacle_detected_toy'], [leaf_nodes['is_back_obstacle_detected_toy_light']],"back_obstacle_detected_toy_fallback")
    behaviors_parallel = py_trees.composites.Parallel(name="behaviors_parallel", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
    behaviors_parallel.add_children([social_walk_fallback,looking_for_toy_fallback,chasing_toy_fallback,chasing_person_fallback,obstacle_detected_fallback,back_obstacle_detected_toy_fallback])
    #ROOT
    root = py_trees.composites.Selector("triskarino_behavior")
    root.add_children([condition_checking_sequence,behaviors_parallel])
    
    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

##############################################################################
# Main
##############################################################################

if __name__ == "__main__":
    """
    Entry point for the demo script.
    """
    rospy.init_node("tree")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    rospy.loginfo("Before Setting up tree")
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        rospy.logerr("Failed to setup behavior tree")
        sys.exit(1)
    rospy.loginfo("Tree setup correctly")
    behaviour_tree.tick_tock(500)