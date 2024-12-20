#!/usr/bin/env python3

# ------------------------------
# Libraries to import
# ------------------------------
import rospy, time, math, pandas as pd, numpy as np                  # Import standard Python libraries

from triskarino_msgs.msg import touchdataarray          # Import custom message types
from triskarino_msgs.msg import touchdata               # Import custom message types
#from triskarino_msgs.msg import touchClassification    # Import custom message types
from std_msgs.msg import String, Float32                      # Import standard message types
from geometry_msgs.msg import Twist                     # Import standard ROS message types for Twist
from sensor_msgs.msg import Imu                         # Import standard ROS message types for IMU

from triskarino_perception.srv import getTouch                  # Import custom service types

# ------------------------------
# Constants
# ------------------------------

MIN_CALIBRATION_DATA = 500
PUBLISHER_QUEUE_SIZE = 100  # Publisher queue size
WAIT_DURATION = 10         # Wait duration for messages

MAX_DATA_LENGTH = 50  # Maximum length of the data array
# Constants: names of topics, services and Node
TOUCH_TOPIC = 'touch_data_array'        # Touch
TOUCH_HEAD_TOPIC = 'touch_data_head'    # Touch head
MOVEMENT_TOPIC = '/cmd_vel_out_filled'  # Movement
IMU_TOPIC = '/imu/data_filtered'        # IMU

STA_SERVICE_NAME = 'static_classifier'  # Classification 
DYN_SERVICE_NAME = 'dynamic_detector'   # Detection

NODE_NAME = 'touchclassifier_manager'   # Node
RATE = 2
WINDOW_SIZE=5

# ------------------------------
# Definition of the Node class
# ------------------------------

class TouchClassifierManager:
    """
    Manages the touch classification process for a robot using the static and dynamic service classifiers.
    Attributes:
        calibration_offsets (np.ndarray): Calibration offsets for the touch sensors.
        is_calibration_done (bool): Indicates if the calibration is completed.
        threshold_overturning (float): Threshold for detecting if the robot is overturning.
        touch_data (pd.DataFrame): DataFrame to store touch data.
        start_time (float): Start time for recording touch data.
        rate (rospy.Rate): ROS rate for controlling the loop frequency.
        static_classification_service (rospy.ServiceProxy): Proxy for the static classification service.
        dynamic_classification_service (rospy.ServiceProxy): Proxy for the dynamic classification service.
        classification_pub (rospy.Publisher): Publisher for the classification results.
        touch_subscriber (rospy.Subscriber): Subscriber for the touch sensor data.
    Methods:
        __init__(): Initializes the TouchClassifierManager.
        spin(): Main loop that runs the classification process.
        add_row_to_data(data): Adds a new row of touch data to the DataFrame.
        smoothing_and_cut(): Applies smoothing to the touch data and ensures the DataFrame does not exceed the maximum size.
        check_for_calibration_data(): Checks if there is enough data to perform calibration and updates the calibration offsets.
        touch_callback(data): Callback function for the touch sensor data.
        classify(data): Classifies the touch data and publishes the results.
        transform_float_to_float_msg(num): Converts a float to a ROS Float32 message.
        call_service_classification(data_sensor): Calls the static classification service.
        call_service_detection(data_sensor): Calls the dynamic detection service.
        quaternion2euler(robot_orientationX, robot_orientationY, robot_orientationZ, robot_orientationW): Converts a quaternion to Euler angles.
    """



    def __init__(self):

        ##### Variables #####
        rospy.init_node(NODE_NAME)

        # Calibration
        self.calibration_offsets = np.zeros(6)
        self.is_calibration_done = False
    
        # Set the threshold for overturning
        self.threshold_overturning = 0.3

        # Initialize a dataframe to store the touch data
        self.touch_data = pd.DataFrame(columns=['Time', 'Pressure1', 'FlexSx1', 'FlexDx1', 'Pressure2', 'FlexSx2', 'FlexDx2', 'Pressure3', 'FlexSx3', 'FlexDx3', 'Pressure4', 'FlexSx4', 'FlexDx4', 'Pressure5', 'FlexSx5', 'FlexDx5', 'PressureHead', 'FlexSxHead', 'FlexDxHead'])
        self.start_time = time.time()    
        self.rate = rospy.Rate(RATE)
        

        


        # Initialize the service: first wait for the services to be available, then create the proxies
        rospy.wait_for_service(STA_SERVICE_NAME)
        rospy.wait_for_service(DYN_SERVICE_NAME)
        self.static_classification_service = rospy.ServiceProxy(STA_SERVICE_NAME, getTouch,persistent=True)
        self.dynamic_classification_service = rospy.ServiceProxy(DYN_SERVICE_NAME, getTouch,persistent=True)
             
        # Print a message to inform that the services are ready
        rospy.loginfo("!!! SERVICES ARE READY !!!")

        # Initialize the publisher to a topic that will be used to publish the classification result
        self.classification_pub = rospy.Publisher('/touchclassification', String, queue_size=PUBLISHER_QUEUE_SIZE)

        # Initialize the subscriber to the touch sensor topic
        self.touch_subscriber = rospy.Subscriber(TOUCH_TOPIC, touchdataarray, self.touch_callback, queue_size=PUBLISHER_QUEUE_SIZE)
        
        # Print a message to inform that the node is ready
        rospy.loginfo("TouchClassifierManager node ready")
    
               
    def spin(self):
        while not rospy.is_shutdown():
            if self.is_calibration_done:
                self.classify(self.touch_data)
                self.rate.sleep()
        


    def add_row_to_data(self, data):
         # Wait for messages from the head topic (timeout is used to avoid waiting forever)    
        try:
            robot_head = rospy.wait_for_message(TOUCH_HEAD_TOPIC, touchdata, timeout=rospy.Duration.from_sec(WAIT_DURATION))     # Wait for messages from the IMU topic
            robot_head_values = [robot_head.pressure_value, robot_head.flexSx_value, robot_head.flexDx_value]
        except:
            rospy.logwarn('[HEAD]: Timeout reached while waiting for messages')
            robot_head_values = [0.0, 0.0, 0.0]
        
        
        
        # Concat the message
        time_diff = time.time() - self.start_time if hasattr(self, 'start_time') else 0
        new_row = [{'Time': time_diff,'Pressure1': data.pressure_values[0] - self.calibration_offsets[0], 'FlexSx1': data.flex_sx_values[0], 'FlexDx1': data.flex_dx_values[0],
                                        'Pressure2': data.pressure_values[1] - self.calibration_offsets[1], 'FlexSx2': data.flex_sx_values[1], 'FlexDx2': data.flex_dx_values[1],
                                        'Pressure3': data.pressure_values[2] - self.calibration_offsets[2], 'FlexSx3': data.flex_sx_values[2], 'FlexDx3': data.flex_dx_values[2],
                                        'Pressure4': data.pressure_values[3] - self.calibration_offsets[3], 'FlexSx4': data.flex_sx_values[3], 'FlexDx4': data.flex_dx_values[3],
                                        'Pressure5': data.pressure_values[4] - self.calibration_offsets[4], 'FlexSx5': data.flex_sx_values[4], 'FlexDx5': data.flex_dx_values[4],
                                        'PressureHead': robot_head_values[0] - self.calibration_offsets[5], 'FlexSxHead': robot_head_values[1], 'FlexDxHead': robot_head_values[2]}]
        df_new = pd.DataFrame(new_row)
        self.touch_data = pd.concat([self.touch_data, df_new], ignore_index=True)
        
    
    #Uses touch data to perform smoothing and cut of the touch_data dataframe before sending it to the classifier
    def smoothing_and_cut(self):
        #Not smoothing Time column
        columns_to_smooth = self.touch_data.columns.difference(['Time'])
        # Apply rolling mean for all columns and keep only smoothed columns
        smoothed_df = self.touch_data[columns_to_smooth].rolling(window=WINDOW_SIZE, min_periods=1).mean()
        #Re-concatenating time column
        self.touch_data = pd.concat([self.touch_data['Time'], smoothed_df], axis=1)

        # Ensure DataFrame doesn't exceed max size
        #TODO: Check how many times it's efficient to do this operation (Maybe it's ok to not do it everytime)
        if len(self.touch_data) > MAX_DATA_LENGTH:
            self.touch_data = self.touch_data.tail(MAX_DATA_LENGTH)  # Keep only the last max_rows entries
    
    #Checks if the data gathered it's enough to update the calibration offsets, if it's calculates the offsets updates the is_calibration_done boolean and deletes 
    #the dataset previous rows
    def check_for_calibration_data(self):
        if len(self.touch_data) % 100 == 0:
            rospy.loginfo('Calibrating...Now we have ' + str(len(self.touch_data)) + ' data points')
        if len(self.touch_data) > MIN_CALIBRATION_DATA:
                #TODO: If nothing works put back the -1.1 after the np.array... Removing it cause I don't think it makes sense
                self.calibration_offsets = np.array([self.touch_data['Pressure1'].mean(), self.touch_data['Pressure2'].mean(), self.touch_data['Pressure3'].mean(), self.touch_data['Pressure4'].mean(), self.touch_data['Pressure5'].mean(), self.touch_data['PressureHead'].mean()]) 
                self.touch_data = self.touch_data.iloc[0:0]     # Delete the dataframe to avoid wrong classification
                self.is_calibration_done = True
                rospy.logwarn('Calibration complited')


    #Touch callback only fills up the dataframe and does the smoothing (Keeping only fixed amount of values), then the spin function will call classify 
    #Before doing this, the touch callback fetches the values and does calibration
    def touch_callback(self, data):
        #If data is not ready I accumulate data
        if not self.is_calibration_done:
            self.add_row_to_data(data)
            self.check_for_calibration_data()

        #If the calibration was performed, then I plan to only accumulate a fixed amount of points and smooth them out
        #TODO: Check out what happens to the classification if the robot moves
        #TODO: Try out different smoothings/different numbers of max data length kept and sent to the classifier (they influnce smoothing)
        if self.is_calibration_done:
            try:            
                robot_orientation = rospy.wait_for_message(IMU_TOPIC, Imu, timeout=rospy.Duration.from_sec(WAIT_DURATION))     # Wait for messages from the IMU topic
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
                self.add_row_to_data(data)

 
        
    
    def classify(self, data):
        # Check if the arrays are full
        if len(data) >= MAX_DATA_LENGTH:
            rospy.loginfo('In classification function, current data length is: ' + str(len(data)))
            #Performs smoothing and cuts rouch data
            self.smoothing_and_cut()
            try: 
                robot_motion = rospy.wait_for_message(MOVEMENT_TOPIC, Twist, timeout=rospy.Duration.from_sec(WAIT_DURATION))   # Wait for messages from the movement topic
                robot_AccRot = [robot_motion.linear.x, robot_motion.linear.y, robot_motion.angular.z]
            except Exception as e:
                rospy.loginfo(e)
                rospy.logwarn('[MOTION]: Timeout reached while waiting for messages')
                robot_AccRot = [0.0, 0.0, 0.0]
            
            # Check if the robot is moving
            if any(abs(value) > 0.0 for value in robot_AccRot):
                rospy.loginfo('Robot is moving')
                try:
                    # Call the detection service for each position
                    self.label_1 = self.call_service_detection(data[['Pressure1', 'FlexSx1', 'FlexDx1','Time']])
                    self.label_2 = self.call_service_detection(data[['Pressure2', 'FlexSx2', 'FlexDx2','Time']])
                    self.label_3 = self.call_service_detection(data[['Pressure3', 'FlexSx3', 'FlexDx3','Time']])
                    self.label_4 = self.call_service_detection(data[['Pressure4', 'FlexSx4', 'FlexDx4','Time']])
                    self.label_5 = self.call_service_detection(data[['Pressure5', 'FlexSx5', 'FlexDx5','Time']])
                    self.label_Head = self.call_service_detection(data[['PressureHead', 'FlexSxHead', 'FlexDxHead','Time']])
                    rospy.loginfo('SERVICES RETURNED DYNAMIC')

                except:
                    rospy.logwarn("Dynamic Service Not Available Yet")
                    return
                
            else:
                try:
                    # Call the classification service for each position
                    self.label_1 = self.call_service_classification(data[['Pressure1', 'FlexSx1', 'FlexDx1','Time']])
                    self.label_2 = self.call_service_classification(data[['Pressure2', 'FlexSx2', 'FlexDx2','Time']])
                    self.label_3 = self.call_service_classification(data[['Pressure3', 'FlexSx3', 'FlexDx3','Time']])
                    self.label_4 = self.call_service_classification(data[['Pressure4', 'FlexSx4', 'FlexDx4','Time']])
                    self.label_5 = self.call_service_classification(data[['Pressure5', 'FlexSx5', 'FlexDx5','Time']])
                    self.label_Head = self.call_service_classification(data[['PressureHead', 'FlexSxHead', 'FlexDxHead','Time']])
                    rospy.loginfo('SERVICES RETURNED STATIC')

                except:
                    rospy.logwarn("Static Service Not Available Yet")
                    return
                
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
            rospy.loginfo("Msg Prepated before classification")
            self.class_msg.data = messaggio
            self.classification_pub.publish(self.class_msg)

    
    def transform_float_to_float_msg(self, num):
        float_msg = Float32()
        float_msg.data = num
        return float_msg

    # Call the classification service
    def call_service_classification(self, data_sensor):
         
        try:
            pressure_sensor = data_sensor.iloc[:, 0].values.astype(float).tolist()
            flexsx_sensor = data_sensor.iloc[:, 1].values.astype(float).tolist()
            flexdx_sensor = data_sensor.iloc[:, 2].values.astype(float).tolist()
            time_sensor = data_sensor.iloc[:, 3].values.astype(float).tolist()

            pressure_sensor_float_list = [self.transform_float_to_float_msg(num) for num in pressure_sensor]
            flexsx_sensor_float_list = [self.transform_float_to_float_msg(num) for num in flexsx_sensor]
            flexdx_sensor_float_list = [self.transform_float_to_float_msg(num) for num in flexdx_sensor]
            time_sensor_float_list = [self.transform_float_to_float_msg(num) for num in time_sensor]

            response = self.static_classification_service(pressure_sensor_float_list, flexsx_sensor_float_list, flexdx_sensor_float_list, time_sensor_float_list)
            classification_data = response.Label

            return classification_data
                    
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
            classification_data = ""
            return classification_data
    
    # Call the detection service
    def call_service_detection(self, data_sensor):
        
        try:
            pressure_sensor = data_sensor.iloc[:, 0].values.astype(float).tolist()
            flexsx_sensor = data_sensor.iloc[:, 1].values.astype(float).tolist()
            flexdx_sensor = data_sensor.iloc[:, 2].values.astype(float).tolist()
            time_sensor = data_sensor.iloc[:, 3].values.astype(float).tolist()

            pressure_sensor_float_list = [self.transform_float_to_float_msg(num) for num in pressure_sensor]
            flexsx_sensor_float_list = [self.transform_float_to_float_msg(num) for num in flexsx_sensor]
            flexdx_sensor_float_list = [self.transform_float_to_float_msg(num) for num in flexdx_sensor]
            time_sensor_float_list = [self.transform_float_to_float_msg(num) for num in time_sensor]

            response = self.dynamic_classification_service(pressure_sensor_float_list, flexsx_sensor_float_list, flexdx_sensor_float_list, time_sensor_float_list)
            detection_data = response.Label
            
            return detection_data
                    
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s', e)
            detection_data = ""
            return detection_data
    
    
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
    node.spin()
    rospy.loginfo('Touch manager node stopped')
    
    exit(0)
