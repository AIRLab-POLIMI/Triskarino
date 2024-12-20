#!/usr/bin/env python3

from py_trees_ros import subscribers
from triskarino_msgs.msg import Sonar
import py_trees

class SonarToBlackboard(subscribers.ToBlackboard):
    '''
    A class to subscribe to sonar sensor data and write it to the blackboard.
    This class inherits from `subscribers.ToBlackboard` and is responsible for subscribing
    to sonar sensor data, writing the raw data to the blackboard, and updating specific
    blackboard variables with processed sonar data.
    Attributes:
        blackboard (py_trees.blackboard.Blackboard): The blackboard instance where data is stored.
        blackboard.sonar_distance (Sonar): The raw sonar data.
        blackboard.back_sonar_distance (float): The processed back sonar distance in meters.
    Methods:
        __init__(name, topic_name="/sonar"):
            Initializes the SonarToBlackboard instance with the given name and topic name.
        update():
            Updates the blackboard with the latest sonar data and processes the back sonar distance.
    '''
    
    def __init__(self, name, topic_name="/sonar"):
        super(SonarToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=Sonar,
                                           blackboard_variables={"sonar_distance": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.sonar_distance = Sonar()
        self.blackboard.back_sonar_distance = -1

    def update(self):
        """
        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(SonarToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            self.blackboard.back_sonar_distance = self.blackboard.sonar_distance.distances[3] / 100
            self.feedback_message = "Back Sonar Distance is " + str(self.blackboard.back_sonar_distance)
        return status