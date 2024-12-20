#!/usr/bin/env python3

from py_trees_ros import subscribers
from triskarino_msgs.msg import Objects
import py_trees
import rospy

class ObjectsToBlackboard(subscribers.ToBlackboard):
    """
    A class that subscribes to a topic publishing detected objects and updates the blackboard with the detected objects.
    Attributes:
        blackboard (py_trees.blackboard.Blackboard): The blackboard instance to store detected objects and flags.
        blackboard.objects (Objects): The detected objects message.
        blackboard.no_people_detected (bool): Flag indicating if no people are detected.
        blackboard.no_toy_detected (bool): Flag indicating if no toys are detected.
        blackboard.no_stick_detected (bool): Flag indicating if no sticks are detected.
    Methods:
        __init__(name, topic_name="/detected_objects"):
            Initializes the ObjectsToBlackboard instance with the given name and topic name.
        update():
            Updates the blackboard with the detected objects and sets the appropriate flags.
    """
    
    def __init__(self, name, topic_name="/detected_objects"):
        super(ObjectsToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=Objects,
                                           blackboard_variables={"objects": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.objects = Objects()
        self.blackboard.no_people_detected = True  # decision making
        self.blackboard.no_toy_detected = True
        self.blackboard.no_stick_detected = True

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ObjectsToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something
            if len(self.blackboard.objects.objects) > 0:
                #Set no people detected equal to false if in objects message there are objects with class_id equal to "person"
                if any(obj.class_id == "person" for obj in self.blackboard.objects.objects):
                    self.blackboard.no_people_detected = False
                #Set no toy detected equal to false if in objects message there are objects with class_id equal to "toy"
                if any(obj.class_id == "toy" for obj in self.blackboard.objects.objects):
                    self.blackboard.no_toy_detected = False
                #Set no stick detected equal to false if in objects message there are objects with class_id equal to "stick"
                if any(obj.class_id == "stick" for obj in self.blackboard.objects.objects):
                    self.blackboard.no_stick_detected = False
            else:
                self.blackboard.no_people_detected = True
                self.blackboard.no_toy_detected = True
                self.blackboard.no_stick_detected = True
                rospy.logwarn_throttle(60, "%s: No people detected from camera!" % self.name)
            # else don't do anything in between - i.e. avoid the ping pong problems
            self.feedback_message = "No objects detected from camera" if len(self.blackboard.objects.objects) == 0 else str(len(self.blackboard.objects.objects)) +" people are in the frame"
        return status