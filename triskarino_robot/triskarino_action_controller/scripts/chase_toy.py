#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from triskarino_msgs.msg import Objects, Object
from geometry_msgs.msg import Twist
from utility import toy_chase_predictor
from dynamic_reconfigure.server import Server
from triskarino_action_controller.cfg import chase_reconfigureConfig

#Image parameters
img_width = 640
#Node Parameters
RATE = 15
WAIT_DURATION = 1


class ToyChaserNode():
    """
    A ROS node for generating the velocities for the robot to chase either the toy or the stick based on what is in the bbox. 
    Rule-based system based on a set of parameters which can be tuned through rqt reconfigure.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        vel_publisher (rospy.Publisher): Publisher for the robot's velocity commands.
        cmd_chase_subscriber (rospy.Subscriber): Subscriber for the robot's state.
        chase_toy_state (int): The current state of chasing the toy (0: Not Chasing, 1: Chasing but not going backwards, 2: Fully Chasing).
        param_dict (dict): Dictionary containing parameters for the toy chase predictor.
        toy_chase_predictor (ToyChaseTunablePredictor): An instance of the toy chase predictor.
        srv (dynamic_reconfigure.server.Server): Dynamic reconfigure server for changing parameters.
    Methods:
        change_param_dict(config, level): Callback to update parameters for the toy chase predictor.
        change_chase_toy(robot_state): Callback to update the chase state based on the robot's state.
        spin(): Main loop to process detected objects and publish velocity commands.
        filter_predicted_vel_not_going_backwards(predicted_vel): Filters out backward velocities.
        publishCmdVel(objects): Processes detected objects and publishes velocity commands.
        convert_and_publish_vel(predicted_vel): Converts predicted velocities to a Twist message and publishes it.
    """

    NODE_NAME = "toy_chaser_node"
    def __init__(self):
        rospy.init_node("toy_chaser_node")
        self.vel_publisher = rospy.Publisher('/cmd_vel_chaser',Twist,queue_size=10)
        self.cmd_chase_subscriber = rospy.Subscriber("robot_state", String, self.change_chase_toy)
        #3 States: 0 - Not Chasing Toy, 1 - Chasing Toy but not going backwards, 2 - Chasing Toy Fully
        self.chase_toy_state = 0
        self.param_dict = {
            'min_bbox_area': 3000,
            'max_bbox_area': 7000,
            'linear_mov_starter': 0.6,
            'accel_linear_mov' : 1.03,
            'decel_linear_mov' : 0.3,
            'linear_mov_addition': 0.02,
            'tolerance_angle': 20,
            'rotation_unknown_starter': 0.02,
            'rotation_starter': 0.01,
            'rotation_increment': 0.002,
            'decel_rot_movement': 0.04
        }
        self.toy_chase_predictor = toy_chase_predictor.ToyChaseTunablePredictor(self.param_dict)
        self.srv = Server(chase_reconfigureConfig, self.change_param_dict)


    def change_param_dict(self,config, level):
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
        self.toy_chase_predictor = toy_chase_predictor.ToyChaseTunablePredictor(self.param_dict)
        rospy.loginfo("CHANGED PREDICTOR, NEW PARAMS: " + str(self.param_dict))
        return config
    
    def change_chase_toy(self,robot_state):
        if "chasing_toy" ==  robot_state.data:
            self.chase_toy_state = 2
        elif "back_obstacle_detected_toy" == robot_state.data:
            self.chase_toy_state = 1
        else:
            self.chase_toy_state = 0

    def spin(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            try:
                objects = rospy.wait_for_message('/detected_objects',Objects,timeout=rospy.Duration.from_sec(WAIT_DURATION))
                self.publishCmdVel(objects)
                rate.sleep()
            except Exception as e:
                rospy.logerr("Exception caught while trying to get detected toy from chase_toy " + str(e))
                continue
   
    def filter_predicted_vel_not_going_backwards(self,predicted_vel):
        if predicted_vel[0] < 0:
            predicted_vel[0] = 0
        return predicted_vel
    
    def publishCmdVel(self,objects):
        if self.chase_toy_state == 2 or self.chase_toy_state == 1:
            #Gets the toy object within the objects.objects list, by filtering for the class_id equal to toy, in case of no toy creates an object with bbox [-1,-1,-1,-1]
            toys = [obj for obj in objects.objects if obj.class_id == "toy"]
            sticks = [obj for obj in objects.objects if obj.class_id == "stick"]
            if len(toys) == 0 and len(sticks) == 0:
                rospy.loginfo("No toy or stick detected")
                toy = Object()
                toy.bbox = [-1,-1,-1,-1, -1,-1,-1,-1]
                type = "toy"
            elif len(toys) == 0 and len(sticks) != 0:
                toy = sticks[0]
                type = "stick"
            elif len(toys) != 0:
                toy = toys[0]
                type = "toy"
            predicted_vel = self.toy_chase_predictor.predict(toy.bbox, type)
            if self.chase_toy_state == 1:
                rospy.loginfo("BACK VELOCITY CUT")
                predicted_vel = self.filter_predicted_vel_not_going_backwards(predicted_vel)
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
    node = ToyChaserNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    node.spin()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
