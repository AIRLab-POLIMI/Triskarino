#!/usr/bin/env python3

from py_trees_ros import subscribers
from std_msgs.msg import Float32
import py_trees

class ObstacleDistanceToBlackboard(subscribers.ToBlackboard):
    '''
    A class that subscribes to a topic providing obstacle distance data and writes this data to a blackboard.
    This class inherits from the `subscribers.ToBlackboard` class and is used to subscribe to a topic that provides
    obstacle distance data from a LiDAR sensor. The data is then written to a blackboard for use by other components
    in the system.
    Attributes:
        blackboard (py_trees.blackboard.Blackboard): The blackboard instance where the obstacle distance data is stored.
        blackboard.obstacle_distance (Float32): The obstacle distance data received from the LiDAR sensor.
    Args:
        name (str): The name of the behavior.
        topic_name (str, optional): The name of the topic to subscribe to. Defaults to "/lidar_obstacle_distance".
    Methods:
        update():
            Updates the blackboard with the latest obstacle distance data and sets a feedback message.
    '''

    def __init__(self, name, topic_name="/lidar_obstacle_distance"):
        super(ObstacleDistanceToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=Float32,
                                           blackboard_variables={"obstacle_distance": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.obstacle_distance = Float32()

    def update(self):
        """
        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ObstacleDistanceToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            self.feedback_message = "Obstacle Distance is " + str(self.blackboard.obstacle_distance.data)
        return status