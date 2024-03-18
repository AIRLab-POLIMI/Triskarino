#!/usr/bin/env python3

# ------------------------------
# Libraries to import
# ------------------------------
import rospy, time, math, pandas as pd, numpy as np                  # Import standard Python libraries

from triskarino_msgs.msg import touchdataarray          # Import custom message types
from triskarino_msgs.msg import touchdata               # Import custom message types
#from triskarino_msgs.msg import touchClassification    # Import custom message types
from std_msgs.msg import String                         # Import standard message types
from geometry_msgs.msg import Twist                     # Import standard ROS message types for Twist
from sensor_msgs.msg import Imu                         # Import standard ROS message types for IMU

from triskarino_perception.srv import getTouch                  # Import custom service types

# ------------------------------
# Constants
# ------------------------------

PUBLISHER_QUEUE_SIZE = 100  # Publisher queue size
WAIT_DURATION = 10         # Wait duration for messages

MAX_DATA_LENGTH = 50       # Maximum length of the data array

# ------------------------------
# Definition of the Node class
# ------------------------------

class TouchClassifierManager:

    # Constants: names of topics, services and Node
    TOUCH_TOPIC = 'touch_data_array'        # Touch
    TOUCH_HEAD_TOPIC = 'touch_data_head'    # Touch head
    MOVEMENT_TOPIC = '/cmd_vel_out_filled'  # Movement
    IMU_TOPIC = '/imu/data_filtered'        # IMU

    STA_SERVICE_NAME = 'static_classifier'  # Classification 
    DYN_SERVICE_NAME = 'dynamic_detector'   # Detection

    NODE_NAME = 'touchclassifier_manager'   # Node


    def __init__(self):

        ##### Variables #####

        # Calibration
        self.Calibration = False
        self.calibration = np.zeros(6)
    
        # Set the threshold for overturning
        self.threshold_overturning = 0.3

        # Initialize a dataframe to store the touch data
        self.touch_data = pd.DataFrame(columns=['Time', 'Pressure1', 'FlexSx1', 'FlexDx1', 'Pressure2', 'FlexSx2', 'FlexDx2', 'Pressure3', 'FlexSx3', 'FlexDx3', 'Pressure4', 'FlexSx4', 'FlexDx4', 'Pressure5', 'FlexSx5', 'FlexDx5', 'PressureHead', 'FlexSxHead', 'FlexDxHead'])
        self.start_time = time.time()    

        
        ##### ROS initialization #####    
        # Initialize ROS node
        rospy.init_node(self.NODE_NAME)

        # Initialize the publisher to a topic that will be used to publish the classification result
        self.classification_pub = rospy.Publisher('/touchclassification', String, queue_size=PUBLISHER_QUEUE_SIZE)

        # Initialize the subscriber to the touch sensor topic
        self.touch_subscriber = rospy.Subscriber(self.TOUCH_TOPIC, touchdataarray, self.touch_callback, queue_size=PUBLISHER_QUEUE_SIZE)
        
        # Print a message to inform that the node is ready
        rospy.loginfo("TouchClassifierManager node ready")


        # Initialize the service: first wait for the services to be available, then create the proxies
        rospy.wait_for_service(self.STA_SERVICE_NAME)
        rospy.wait_for_service(self.DYN_SERVICE_NAME)
        self.static_classification_service = rospy.ServiceProxy(self.STA_SERVICE_NAME, getTouch)
        self.dynamic_classification_service = rospy.ServiceProxy(self.DYN_SERVICE_NAME, getTouch)
             
        # Print a message to inform that the services are ready
        rospy.loginfo("!!! SERVICES ARE READY !!!")


    def touch_callback(self, data):

        # Wait for messages from the head topic (timeout is used to avoid waiting forever)    
        try:
            robot_head = rospy.wait_for_message(self.TOUCH_HEAD_TOPIC, touchdata, timeout=rospy.Duration.from_sec(WAIT_DURATION))     # Wait for messages from the IMU topic
            robot_head_values = [robot_head.pressure_value, robot_head.flexSx_value, robot_head.flexDx_value]
        except:
            rospy.logwarn('[HEAD]: Timeout reached while waiting for messages')
            robot_head_values = [0.0, 0.0, 0.0]
    
        # Concat the message
        time_diff = time.time() - self.start_time if hasattr(self, 'start_time') else 0
        new_row = [{'Time': time_diff,'Pressure1': data.pressure_values[0] - self.calibration[0], 'FlexSx1': data.flex_sx_values[0], 'FlexDx1': data.flex_dx_values[0],
                                        'Pressure2': data.pressure_values[1] - self.calibration[1], 'FlexSx2': data.flex_sx_values[1], 'FlexDx2': data.flex_dx_values[1],
                                        'Pressure3': data.pressure_values[2] - self.calibration[2], 'FlexSx3': data.flex_sx_values[2], 'FlexDx3': data.flex_dx_values[2],
                                        'Pressure4': data.pressure_values[3] - self.calibration[3], 'FlexSx4': data.flex_sx_values[3], 'FlexDx4': data.flex_dx_values[3],
                                        'Pressure5': data.pressure_values[4] - self.calibration[4], 'FlexSx5': data.flex_sx_values[4], 'FlexDx5': data.flex_dx_values[4],
                                        'PressureHead': robot_head_values[0] - self.calibration[5], 'FlexSxHead': robot_head_values[1], 'FlexDxHead': robot_head_values[2]}]
        df_new = pd.DataFrame(new_row)
        self.touch_data = pd.concat([self.touch_data, df_new], ignore_index=True)

        if self.Calibration:
            try:            
                robot_orientation = rospy.wait_for_message(self.IMU_TOPIC, Imu, timeout=rospy.Duration.from_sec(WAIT_DURATION))     # Wait for messages from the IMU topic
                robot_RollPitch = self.quaternion2euler(robot_orientation.orientation.x, robot_orientation.orientation.y, robot_orientation.orientation.z, robot_orientation.orientation.w) # Convert the quaternion to Euler angles
            except Exception as a:
                rospy.loginfo(a)
                rospy.logwarn('[IMU]: Timeout reached while waiting for messages')
                robot_RollPitch= [0.0, 0.0]
            
            # Check if the robot is overturning
            if any(abs(value)> self.threshold_overturning for value in robot_RollPitch):
                rospy.logwarn('Robot is overturning')           # Print a warning message because the robot is overturning
                self.touch_data = self.touch_data.iloc[0:0]     # Delete the dataframe to avoid wrong classification
            else:
                self.classify(self.touch_data)
        else:
            # Calibrate teh value of the pressure by calculating the average of the first 100 values
            if len(self.touch_data) > 100:
                self.calibration = np.array([self.touch_data['Pressure1'].mean(), self.touch_data['Pressure2'].mean(), self.touch_data['Pressure3'].mean(), self.touch_data['Pressure4'].mean(), self.touch_data['Pressure5'].mean(), self.touch_data['PressureHead'].mean()]) - 1.1
                self.touch_data = self.touch_data.iloc[0:0]     # Delete the dataframe to avoid wrong classification
                self.Calibration = True
                rospy.logwarn('Calibration complited')

    
    def classify(self, data):
        # Check if the arrays are full
        if len(data) == MAX_DATA_LENGTH:

            try: 
                robot_motion = rospy.wait_for_message(self.MOVEMENT_TOPIC, Twist, timeout=rospy.Duration.from_sec(WAIT_DURATION))   # Wait for messages from the movement topic
                robot_AccRot = [robot_motion.linear.x, robot_motion.linear.y, robot_motion.angular.z]
            except Exception as e:
                rospy.loginfo(e)
                rospy.logwarn('[MOTION]: Timeout reached while waiting for messages')
                robot_AccRot = [0.0, 0.0, 0.0]
            
            # Check if the robot is moving
            if any(abs(value) > 0.0 for value in robot_AccRot):
                rospy.loginfo('Robot is moving')

                # Call the detection service for each position
                self.label_1 = self.call_service_detection(data[['Pressure1', 'FlexSx1', 'FlexDx1','Time']], 'Position 1')
                self.label_2 = self.call_service_detection(data[['Pressure2', 'FlexSx2', 'FlexDx2','Time']], 'Position 2')
                self.label_3 = self.call_service_detection(data[['Pressure3', 'FlexSx3', 'FlexDx3','Time']], 'Position 3')
                self.label_4 = self.call_service_detection(data[['Pressure4', 'FlexSx4', 'FlexDx4','Time']], 'Position 4')
                self.label_5 = self.call_service_detection(data[['Pressure5', 'FlexSx5', 'FlexDx5','Time']], 'Position 5')
                self.label_Head = self.call_service_detection(data[['PressureHead', 'FlexSxHead', 'FlexDxHead','Time']], 'Head')
            else:
                # Call the classification service for each position
                self.label_1 = self.call_service_classification(data[['Pressure1', 'FlexSx1', 'FlexDx1','Time']], 'Position 1')
                self.label_2 = self.call_service_classification(data[['Pressure2', 'FlexSx2', 'FlexDx2','Time']], 'Position 1')
                self.label_3 = self.call_service_classification(data[['Pressure3', 'FlexSx3', 'FlexDx3','Time']], 'Position 1')
                self.label_4 = self.call_service_classification(data[['Pressure4', 'FlexSx4', 'FlexDx4','Time']], 'Position 1')
                self.label_5 = self.call_service_classification(data[['Pressure5', 'FlexSx5', 'FlexDx5','Time']], 'Head')
                self.label_Head = self.call_service_classification(data[['PressureHead', 'FlexSxHead', 'FlexDxHead','Time']], 'Bottom')
            
            # Publish the classification result
            classification_time = time.time() - self.start_time if hasattr(self, 'start_time') else 0
            pos1 = self.label_1 if self.label_1 is not None else ""
            pos2 = self.label_2 if self.label_2 is not None else ""
            pos3 = self.label_3 if self.label_3 is not None else ""
            pos4 = self.label_4 if self.label_4 is not None else ""
            pos5 = self.label_5 if self.label_5 is not None else ""
            head = self.label_Head if self.label_Head is not None else ""

            messaggio = "Time: {}; Pos1: {}; Pos2: {}; Pos3: {}; Pos4: {}; Pos5: {}; Head: {}".format(classification_time, pos1, pos2, pos3, pos4, pos5, head)


            self.class_msg = String()
            rospy.loginfo(messaggio)
            self.class_msg.data = messaggio
            self.classification_pub.publish(self.class_msg)
            

            # Delete the first row of the dataframe
            self.touch_data = self.touch_data.iloc[1:] ###### Check if it is fast enough to delete only the first row of the dataframe 

    

    # Call the classification service
    def call_service_classification(self, data_sensor, position):
        try:
            pressure_sensor = data_sensor.iloc[:, 0].values.astype(float)
            flexsx_sensor = data_sensor.iloc[:, 1].values.astype(float)
            flexdx_sensor = data_sensor.iloc[:, 2].values.astype(float)
            time_sensor = data_sensor.iloc[:, 3].values.astype(float)

            response = self.static_classification_service(pressure_sensor, flexsx_sensor, flexdx_sensor, time_sensor)
            classification_data = response.Label

            if classification_data != "":
                rospy.loginfo('%s: %s', position, classification_data) # Print the response

            return classification_data
                    
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
    
    
    # Call the detection service
    def call_service_detection(self, data_sensor, position):
        try:
            pressure_sensor = data_sensor.iloc[:, 0].values.astype(float)
            flexsx_sensor = data_sensor.iloc[:, 1].values.astype(float)
            flexdx_sensor = data_sensor.iloc[:, 2].values.astype(float)
            time_sensor = data_sensor.iloc[:, 3].values.astype(float)
            
            response = self.dynamic_classification_service(pressure_sensor, flexsx_sensor, flexdx_sensor, time_sensor)
            detection_data = response.Label
            if detection_data != "":
                rospy.loginfo('%s: %s', position, detection_data)            # Print the response
            
            return detection_data
                    
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
    
    
    # Convert a quaternion to Euler angles
    def quaternion2euler(self, robot_orientationX, robot_orientationY, robot_orientationZ, robot_orientationW):
        # Source: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-euler-angle/
        
        # Unpack the quaternion values
        x = robot_orientationX
        y = robot_orientationY
        z = robot_orientationZ
        w = robot_orientationW

        # Convert to Euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        return [roll, pitch]

if __name__ == '__main__':
    # Create the node object
    node = TouchClassifierManager()

    # Start&Stop
    rospy.loginfo('Touch manager node started')
    rospy.spin()
    rospy.loginfo('Touch manager node stopped')
    
    exit(0)
