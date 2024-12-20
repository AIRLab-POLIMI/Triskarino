#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from triskarino_msgs.msg import Objects
from geometry_msgs.msg import Twist
import traceback
from utility import person_chase_predictor
from dynamic_reconfigure.server import Server
from triskarino_action_controller.cfg import chase_reconfigureConfig
from utility.chase_utilities import get_rectangle_area

#Image parameters
img_width = 640
#Node Parameters
RATE = 15
WAIT_DURATION = 1


class PersonChaserNode():
    """
    A ROS node that generates the velocities for the robot to approach a person based on the bbox detected.
    Rule-based system based on a set of parameters which can be tuned through rqt reconfigure.

    Attributes:
        NODE_NAME (str): The name of the ROS node.
        vel_publisher (rospy.Publisher): Publisher for the robot's velocity commands.
        robot_state_subscriber (rospy.Subscriber): Subscriber for the robot's state.
        chase_person (bool): Flag indicating whether the robot should chase a person.
        started (bool): Flag indicating whether the node has just started.
        param_dict (dict): Dictionary containing parameters for the person chase predictor.
        person_chase_predictor (ChasePersonTunablePredictor): Predictor for chasing a person.
        srv (dynamic_reconfigure.server.Server): Dynamic reconfigure server for updating parameters.
    Methods:
        change_param_dict(config, level):
            Updates the parameter dictionary based on dynamic reconfigure input.
        change_robot_state(robot_state):
            Updates the chase_person flag based on the robot's state.
        spin():
            Main loop of the node, which processes detected objects and publishes velocity commands.
        getPersonBboxFromObjects(detected_objects):
            Extracts the bounding box of the largest detected person from the detected objects.
        publishCmdVel(detected_objects):
            Publishes velocity commands based on the detected person's bounding box.
        convert_and_publish_vel(predicted_vel):
            Converts the predicted velocity to a Twist message and publishes it.
    """

    NODE_NAME = "person_chaser_node"
    def __init__(self):
        rospy.init_node("person_chaser_node")
        self.vel_publisher = rospy.Publisher('/cmd_vel_person_chaser',Twist,queue_size=10)
        self.robot_state_subscriber = rospy.Subscriber("/robot_state", String, self.change_robot_state)
        self.chase_person = False
        self.started = True
        self.param_dict = {
            'min_bbox_area': 70000,
            'max_bbox_area': 80000,
            'linear_mov_starter': 0.5,
            'accel_linear_mov' : 1.2,
            'decel_linear_mov' : 0.2,
            'linear_mov_addition': 0.04,
            'tolerance_angle': 10,
            'rotation_unknown_starter': 0.05,
            'rotation_starter': 0.05,
            'rotation_increment': 0.002,
            'decel_rot_movement': 0.2
        }
        self.person_chase_predictor = person_chase_predictor.ChasePersonTunablePredictor(self.param_dict)
        self.srv = Server(chase_reconfigureConfig, self.change_param_dict)


    def change_param_dict(self,config, level):
        #If it's just started maintaining the old parameters
        if self.started:
            return config
        self.param_dict['min_bbox_area'] = config.min_bbox_area
        self.param_dict['max_bbox_area'] = config.max_bbox_area
        self.param_dict['linear_mov_starter'] = config.linear_mov_starter
        self.param_dict['accel_linear_mov'] = config.accel_linear_mov
        self.param_dict['decel_linear_mov'] = config.decel_linear_mov
        self.param_dict['linear_mov_addition'] = config.linear_mov_addition
        self.param_dict['tolerance_angle'] = config.tolerance_angle
        self.param_dict['rotation_unknown_starter'] = config.rotation_unknown_starter
        self.param_dict['rotation_starter'] = config.rotation_starter
        self.param_dict['rotation_increment'] = config.rotation_increment
        self.param_dict['decel_rot_movement'] = config.decel_rot_movement
        self.person_chase_predictor = person_chase_predictor.ChasePersonTunablePredictor(self.param_dict)
        rospy.loginfo("CHANGED PREDICTOR, NEW PARAMS: " + str(self.param_dict))
        return config
    
    def change_robot_state(self,robot_state):
        rospy.loginfo("CHANGING CHASING PERSON TO " + str(robot_state.data))
        self.chase_person = True if "chasing_person" == robot_state.data else False

    def spin(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            try:
                detected_objects = rospy.wait_for_message('/detected_objects',Objects,timeout=rospy.Duration.from_sec(WAIT_DURATION))
                self.publishCmdVel(detected_objects)
                rate.sleep()
            except Exception as e:
                rospy.logerr("Exception caught while trying to get detected objects from chase_person " + str(e))
                continue
    
    def getPersonBboxFromObjects(self,detected_objects):
        max_bbox_idx = -1
        max_bbox_area = -1
        #Filter the objects keeping only the ones with class id person
        people = list(filter(lambda x: x.class_id == "person",detected_objects.objects))
        if len(people) == 0 :
            return None
        for idx,person in enumerate(people):
            person_bbox_area = get_rectangle_area(person.bbox)
            if person_bbox_area >= max_bbox_area:
                max_bbox_area = person_bbox_area
                max_bbox_idx = idx
        
        return people[max_bbox_idx].bbox
    
    def publishCmdVel(self,detected_objects):
        person_bbox = self.getPersonBboxFromObjects(detected_objects)
        if person_bbox == None:
            self.convert_and_publish_vel([0.0,0.0,0.0])
            return
        #Function that decides which person to follow and gives back its bbox            
        if self.chase_person:
            predicted_vel = self.person_chase_predictor.predict(person_bbox)
            rospy.loginfo("Predicted vel: " + str(predicted_vel))
            self.convert_and_publish_vel(predicted_vel)

    def convert_and_publish_vel(self,predicted_vel):
        twist_msg = Twist()
        twist_msg.linear.x = predicted_vel[0]
        twist_msg.linear.y = predicted_vel[1]
        twist_msg.angular.z = predicted_vel[2]
        self.vel_publisher.publish(twist_msg)
        
if __name__ == '__main__':
    rospy.loginfo("AO")
    node = PersonChaserNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    node.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
