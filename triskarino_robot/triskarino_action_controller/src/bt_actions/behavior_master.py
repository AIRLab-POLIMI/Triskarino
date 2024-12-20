#!/usr/bin/env python3

import py_trees
import rospy
from std_msgs.msg import String
import math
import numpy as np
import time

WAIT_FOR_MESSAGE = 20

#Reads the variables on the blackboard and update the robot state on the blackboard and by publishing
class BehaviorMaster(py_trees.behaviour.Behaviour):
    '''
    BehaviorMaster is a behavior tree node that manages the state of a robot based on obstacles, if the toy is seen or lost etc.
    Changes the robot state variable on the blackboard which then decides which branch of the BT to follow
    (Not the best way to make a BT but it was the fastest way to implement it)
    Attributes:
        blackboard (py_trees.blackboard.Blackboard): Shared memory for behaviors to read/write data.
        topic (str): The topic name for publishing the robot state.
        robot_state_publisher (rospy.Publisher): ROS publisher for the robot state.
        toy_is_lost_time (float): Time threshold to consider the toy as lost.
        toy_not_found_time (float): Time threshold to consider the toy as not found.
        time_toy_seen (float): Time threshold to start chasing the toy after it is seen.
        last_time_toy_seen (float): Timestamp of the last time the toy was seen.
        last_time_obstacle_detected (float): Timestamp of the last time an obstacle was detected.
        time_obstacle_backing (float): Time threshold for backing up when an obstacle is detected.
        warm_up_time_before_chasing_toy (float): Time threshold before starting to chase the toy.
        current_robot_state (str): Current state of the robot.
        looking_for_person_counter (float): Time threshold for looking for a person.
        last_time_person_seen (float): Timestamp of the last time a person was seen.
        min_obstacle_distance (float): Minimum distance to consider an obstacle.
        min_back_sonar_distance (float): Minimum distance for the back sonar to consider an obstacle.
    Methods:
        __init__(self, name, topic="robot_state", toy_is_lost_time=2, toy_not_found_time=10, time_toy_seen=0.5, time_chasing_person=3, min_obstacle_distance=0.5, min_back_sonar_distance=0.5, time_obstacle_backing=2):
            Initializes the BehaviorMaster with the given parameters.
        setup(self, timeout):
            Sets up the behavior. Returns True if setup is successful.
        publish_state(self, robot_state):
            Publishes the current robot state to the specified topic.
        update_robot_state(self, robot_state):
            Updates the robot state and publishes it. Also logs the state change.
        update(self):
            Main update loop for the behavior. Determines the robot's state based on sensor inputs and conditions.
        terminate(self, new_status):
            Terminates the behavior. Cancels any ongoing goals if necessary.
    '''

    def __init__(self, name, topic="robot_state",toy_is_lost_time=2, toy_not_found_time=10, time_toy_seen=0.5, time_chasing_person=3,min_obstacle_distance=0.5, min_back_sonar_distance=0.5, time_obstacle_backing=2):
        super(BehaviorMaster, self).__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.topic = topic
        self.robot_state_publisher = rospy.Publisher(self.topic,String)
        self.toy_is_lost_time = toy_is_lost_time
        self.toy_not_found_time = toy_not_found_time
        self.time_toy_seen = time_toy_seen
        self.last_time_toy_seen = 0
        self.last_time_obstacle_detected = 0
        self.time_obstacle_backing = time_obstacle_backing
        self.warm_up_time_before_chasing_toy = 0
        self.current_robot_state = "idle"
        self.looking_for_person_counter = time_chasing_person
        self.last_time_person_seen = 0
        self.min_obstacle_distance = min_obstacle_distance
        self.min_back_sonar_distance = min_back_sonar_distance

    def setup(self, timeout):
        return True

    def publish_state(self, robot_state):
        state_msg = String()
        state_msg.data = robot_state
        self.robot_state_publisher.publish(state_msg)
    
    def update_robot_state(self, robot_state):
        rospy.loginfo("STATE CHANGED TO " + robot_state)
        self.current_robot_state = robot_state
        self.publish_state(self.current_robot_state)
        self.blackboard.set("robot_state", self.current_robot_state)

    def update(self):
        if self.blackboard.get("on") == False:
            self.update_robot_state("idle")
            return py_trees.common.Status.SUCCESS
        else:
            if self.current_robot_state == "idle":
                self.update_robot_state("looking_for_person")
                return py_trees.common.Status.SUCCESS
        
        curr_obstacle_distance = self.blackboard.get("obstacle_distance").data
        back_sonar_distance = self.blackboard.get("back_sonar_distance")
        no_toy_detected = self.blackboard.get("no_toy_detected")
        no_people_detected = self.blackboard.get("no_people_detected")
        #TOY IS DETECTED
        if no_toy_detected == False:
            self.last_time_toy_seen = time.time()
            #IF I WAS NOT CHASING IT BEFORE AND IT'S THE FIRST TIME SEEING THE TOY
            if self.warm_up_time_before_chasing_toy == 0:
                self.warm_up_time_before_chasing_toy = time.time()
            else:
                #IF I WAS NOT CHASING IT BEFORE AND I HAVE BEEN SEEING THE TOY
                if self.current_robot_state != "chasing_toy":
                    #Check if enough time passed to start the toy
                   if (self.last_time_toy_seen - self.warm_up_time_before_chasing_toy) >= self.time_toy_seen:
                       self.update_robot_state("chasing_toy")
                       self.warm_up_time_before_chasing_toy = 0
                if self.current_robot_state == "chasing_toy" and back_sonar_distance < self.min_back_sonar_distance:
                    self.update_robot_state("back_obstacle_detected_toy")
                    return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.SUCCESS
        #IF THE TOY IS NOT DETECTED
        else:
            time_toy_has_been_lost = time.time() - self.last_time_toy_seen
            #IF THE TOY HAS NOT BEEN SEEN FOR A WHILE THE ROBOT STARTS LOOKING FOR IT
            if time_toy_has_been_lost >= self.toy_is_lost_time and time_toy_has_been_lost < self.toy_not_found_time:
                if curr_obstacle_distance < self.min_obstacle_distance:
                    self.update_robot_state("obstacle_detected")
                    return py_trees.common.Status.SUCCESS
                self.update_robot_state("looking_for_toy")
                return py_trees.common.Status.SUCCESS

            elif time_toy_has_been_lost >= self.toy_not_found_time and (self.current_robot_state == "looking_for_toy") or self.current_robot_state == "obstacle_detected":
                if no_people_detected:
                    if curr_obstacle_distance < self.min_obstacle_distance:
                        self.update_robot_state("obstacle_detected")
                        self.last_time_obstacle_detected = time.time()
                        return py_trees.common.Status.SUCCESS
                    elif time.time() - self.last_time_obstacle_detected >= self.time_obstacle_backing:
                        self.update_robot_state("looking_for_person")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.update_robot_state("chasing_person")
                    return py_trees.common.Status.SUCCESS


            if self.current_robot_state == "looking_for_person" and (not no_people_detected):
                if self.last_time_person_seen == 0:
                    self.last_time_person_seen = time.time()
                self.update_robot_state("chasing_person")
                return py_trees.common.Status.SUCCESS
            
            elif self.current_robot_state == "chasing_person" and no_people_detected:
                if (time.time() - self.last_time_person_seen) >= self.looking_for_person_counter:
                    if curr_obstacle_distance < self.min_obstacle_distance:
                        self.update_robot_state("obstacle_detected")
                        return py_trees.common.Status.SUCCESS
                    self.update_robot_state("looking_for_person")
                    self.last_time_person_seen = 0
                return py_trees.common.Status.SUCCESS
            elif self.current_robot_state == "chasing_person" and (not no_people_detected):
                self.last_time_person_seen = time.time()
                return py_trees.common.Status.SUCCESS
            
            elif self.current_robot_state == "looking_for_person" and (no_people_detected):
                if curr_obstacle_distance < self.min_obstacle_distance:
                    self.update_robot_state("obstacle_detected")
                    self.last_time_obstacle_detected = time.time()
                    return py_trees.common.Status.SUCCESS
        
        rospy.loginfo("Finished without returning current state is " + self.current_robot_state + " no_toy_detected is " + str(no_toy_detected) + " no_people_detected is " + str(no_people_detected) + " current obstacle distance is " + str(curr_obstacle_distance))
        return py_trees.common.Status.SUCCESS
                



    def terminate(self,new_status):
        """
        When terminating, if the goal is still in the move base, cancel the goal
        """
        pass