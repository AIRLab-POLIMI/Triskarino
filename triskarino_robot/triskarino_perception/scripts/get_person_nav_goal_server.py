#!/usr/bin/env python3
import rospy
from triskarino_perception.srv import getPersonNavGoal
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import math
from tf import TransformListener
from tf.transformations import quaternion_multiply

WAIT_DURATION = 0.5
FIXED_FRAME = "map"
class GetPersonNavGoalServer():
	"""
	A ROS service server that provides navigation goals for a robot to approach and face a person. Works only if SLAM, person detection and are enabled and person position is publshed.
	Attributes:
		NODE_NAME (str): The name of the ROS node.
		listener (TransformListener): A listener for transform data.
	Methods:
		__init__():
			Initializes the ROS node and service, and sets up the transform listener.
		handle_get_person_nav_goal(srv_request):
			Handles the service request to get navigation goals for the robot to approach and face a person.
			Args:
				srv_request: The service request containing the person and robot frames and stopping distance.
			Returns:
				PoseArray: An array of poses for the robot to follow.
		get_rotation_goal_to_face_person(person_pos, robot_pos, robot_q):
			Calculates the pose for the robot to rotate and face the person.
			Args:
				person_pos (list): The position of the person.
				robot_pos (list): The position of the robot.
				robot_q (list): The quaternion orientation of the robot.
			Returns:
				tuple: A tuple containing the target pose and the new robot quaternion.
		get_approach_goal(person_pos, robot_pos, robot_q, stopping_distance):
			Calculates the pose for the robot to approach the person while maintaining a stopping distance.
			Args:
				person_pos (list): The position of the person.
				robot_pos (list): The position of the robot.
				robot_q (list): The quaternion orientation of the robot.
				stopping_distance (float): The distance to stop from the person.
			Returns:
				Pose: The target pose for the robot to approach the person.
		get_rotation_goal(person_pos, robot_pos, robot_q):
			Calculates the quaternion for the robot to rotate and face the person.
			Args:
				person_pos (list): The position of the person.
				robot_pos (list): The position of the robot.
				robot_q (list): The quaternion orientation of the robot.
			Returns:
				list: The new quaternion for the robot to face the person.
		get_position_quaternion(target_frame, source_frame):
			Retrieves the position and quaternion of a source frame relative to a target frame.
			Args:
				target_frame (str): The target frame.
				source_frame (str): The source frame.
			Returns:
				tuple: A tuple containing the position and quaternion of the source frame.
	"""

	NODE_NAME = "get_person_nav_goal_server"
	def __init__(self):
		rospy.init_node("get_person_nav_goal_server")
		s = rospy.Service('get_person_nav_goal',getPersonNavGoal, self.handle_get_person_nav_goal)
		self.listener = TransformListener()


	def handle_get_person_nav_goal(self,srv_request):
		target_array = PoseArray()
		person_frame = srv_request.person_frame.data
		robot_frame = srv_request.robot_frame.data
		person_pos, person_q = self.get_position_quaternion(FIXED_FRAME, person_frame)
		robot_pos, robot_q = self.get_position_quaternion(FIXED_FRAME, robot_frame)
		#First: By having person and robot position we can calculate the angle between them: The robot has to rotate by that angle 
		first_target_pose, new_robot_quaternion = self.get_rotation_goal_to_face_person(person_pos, robot_pos, robot_q)
		second_target_pose = self.get_approach_goal(person_pos, robot_pos, new_robot_quaternion, srv_request.stopping_distance.data)
		target_array.poses.append(first_target_pose)
		target_array.poses.append(second_target_pose)
		return target_array		
	
	def get_rotation_goal_to_face_person(self, person_pos, robot_pos, robot_q):
		target_pose = Pose()
		target_pose.position.x = robot_pos[0]
		target_pose.position.y = robot_pos[1]
		target_pose.position.z = robot_pos[2]
		new_robot_quaternion = self.get_rotation_goal(person_pos, robot_pos, robot_q)
		target_pose.orientation.x = new_robot_quaternion[0]
		target_pose.orientation.y = new_robot_quaternion[1]
		target_pose.orientation.z = new_robot_quaternion[2]
		target_pose.orientation.w = new_robot_quaternion[3]
		return target_pose, new_robot_quaternion

	def get_approach_goal(self, person_pos, robot_pos, robot_q, stopping_distance):
		target_pose = Pose()
		#Idea: Using formula from https://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
		#To find intermediate point between robot and person
		rospy.loginfo("person pos is " + str(person_pos) + " robot_pos is " + str(robot_pos))
		distance = math.sqrt((person_pos[0] - robot_pos[0]) ** 2 + (person_pos[1] - robot_pos[1]) ** 2)
		distance_ratio = (distance - float(stopping_distance)) / distance
		target_pose.position.x = (1 - distance_ratio) * robot_pos[0] + distance_ratio * person_pos[0]
		target_pose.position.y = (1 - distance_ratio) * robot_pos[1] + distance_ratio * person_pos[1]
		target_pose.position.z = robot_pos[2]
		target_pose.orientation.x = robot_q[0]
		target_pose.orientation.y = robot_q[1]
		target_pose.orientation.z = robot_q[2]
		target_pose.orientation.w = robot_q[3]
		return target_pose

	def get_rotation_goal(self, person_pos, robot_pos, robot_q):
		#Calculate the angle between person_pos and robot_pos -> x is forward y is left
		#Using formula from here https://www.quora.com/How-would-you-find-the-angle-of-a-line-given-two-points-on-a-coordinate-plan#:~:text=Theoretically%2C%20the%20angle%20is%20obtained,angle%20theta%20can%20be%20calculated.
		A = robot_pos[1] - person_pos[1]
		B = person_pos[0] - robot_pos[0]
		angle = math.acos(B/+ abs(math.sqrt(A ** 2 + B ** 2)))
		#Rotate the quaternion of the robot by that angle on the yaw 
		rotation_quat = [0,0,math.sin(angle/2),math.cos(angle/2)]
		return quaternion_multiply(rotation_quat,robot_q)


	def get_position_quaternion(self,target_frame, source_frame):
		self.listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(WAIT_DURATION))
		t = self.listener.getLatestCommonTime(target_frame, source_frame)
		position, quaternion = self.listener.lookupTransform(target_frame, source_frame, t)
		return position, quaternion

		
if __name__ == '__main__':
	rospy.loginfo("AO")
	node = GetPersonNavGoalServer()
	rospy.loginfo( node.NODE_NAME + " running..." )
	rospy.spin()
	rospy.loginfo( node.NODE_NAME + " stopped." )
	exit(0)
