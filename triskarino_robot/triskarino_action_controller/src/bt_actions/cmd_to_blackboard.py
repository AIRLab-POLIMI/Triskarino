#!/usr/bin/env python3

from py_trees_ros import subscribers
import py_trees
from std_msgs.msg import String
import rospy

class CmdToBlackboard(subscribers.ToBlackboard):
    '''
    CmdToBlackboard is a behavior tree action node that subscribes to a specified topic
    and writes the received command to the blackboard. It also updates a boolean flag
    on the blackboard based on the command received.
    Attributes:
        blackboard (py_trees.blackboard.Blackboard): The blackboard instance where data is stored.
        blackboard.cmd (String): The command received from the topic.
        blackboard.on (bool): A flag indicating whether the command is "on".
    Args:
        name (str): The name of the behavior tree node.
        topic_name (str, optional): The name of the topic to subscribe to. Defaults to "/cmd_autonomous_joy".
    Methods:
        update():
            Writes the raw data to the blackboard and updates the boolean flag based on the command.
    '''
    def __init__(self, name, topic_name="/cmd_autonomous_joy"):
        super(CmdToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=String,
                                           blackboard_variables={"cmd": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.cmd = String()
        self.blackboard.on = False
        
    def update(self):
        """
        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(CmdToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            self.blackboard.on = True if self.blackboard.cmd.data == "on" else False
            self.blackboard.set("on",self.blackboard.on)
            self.feedback_message = "On bool is " + str(self.blackboard.on)
        return status
