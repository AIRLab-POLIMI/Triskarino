#!/usr/bin/env python3

import py_trees

class CheckMovementState(py_trees.behaviour.Behaviour):
    """
    This node checks the state of the robot, and if the movement had success or not!
    returns :attr:`~py_trees.common.Status.RUNNING` if the robot is performing the motion and has not finished
    :attr:`~py_trees.common.Status.SUCCESS` if it finished the movement successfully
    :attr:`~py_trees.common.Status.FAILURE` if the robot is not moving
    When the robot finishes the movement, it changes robot_state in the blackboard to idle

    Args:
        name (:obj:`str`): name of the behaviour

    """
    def __init__(self, name="CheckMovementStateNode"):
        super(CheckMovementState, self).__init__(name=name)

    def setup(self,timeout):
        """
        Instantiates the blackboard variable
        """
        self.blackboard = py_trees.blackboard.Blackboard()

        return True


    def update(self):
        """
        Checks if the robot is not in idle state -> if it is in idle state returns failure, if not checks if the robot is moving:
        if it is moving returns running, if not returns success and changes the robot state to idle
        """
        if self.blackboard.robot_state.data == "idle":
            return py_trees.common.Status.FAILURE
        if self.blackboard.active == True:
            return py_trees.common.Status.RUNNING
        if self.blackboard.succeded == True:
            self.blackboard.robot_state.data = "idle"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
    
    def terminate(self,new_status):
        pass