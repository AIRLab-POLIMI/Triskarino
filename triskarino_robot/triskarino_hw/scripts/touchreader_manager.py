#!/usr/bin/env python3

# ------------------------------
# Libraries to import
# ------------------------------

# Built-in libraries
import rospy, time, math, pandas as pd
# ROS message libraries
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from triskarino_msgs.msg import touchdataarray
from triskarino_msgs.msg import touchdata


# ------------------------------
# Constants
# ------------------------------

PUBLISHER_QUEUE_SIZE = 100  # Publisher queue size
WAIT_DURATION = 5 # Wait duration for messages

# ------------------------------
# Definition of the Node class
# ------------------------------
class TouchReaderManager:

    # Constants: names of Topics and Node
    TOUCH_TOPIC = 'touch_data_array'        # Touch
    TOUCH_HEAD_TOPIC = 'touch_data_head'    # Touch head
    MOVEMENT_TOPIC = '/cmd_vel_out_filled'  # Movement
    IMU_TOPIC = '/imu/data_filtered'        # IMU
    
    NODE_NAME = "touchreader_manager"       # Node name
    
    # Constructor
    def __init__(self):
        
        # Initialize the start time
        self.start_time = time.time()          
        
        # Create DataFrame
        self.df = pd.DataFrame(columns=['Time', 'Roll', 'Pitch', 'Yaw', 
                                        'AccX','AccY', 'RotZ',
                                        'Pressure1', 'FlexSx1', 'FlexDx1',
                                        'Pressure2', 'FlexSx2', 'FlexDx2',
                                        'Pressure3', 'FlexSx3', 'FlexDx3',
                                        'Pressure4', 'FlexSx4', 'FlexDx4',
                                        'Pressure5', 'FlexSx5', 'FlexDx5',
                                        'PressureHead', 'FlexSxHead', 'FlexDxHead'])    
        
        # Initialize the node
        rospy.init_node(self.NODE_NAME)  # Initialize the node
        # Initialize the Subscriber
        self.touch_subscriber = rospy.Subscriber(self.TOUCH_TOPIC, touchdataarray, self.touch_callback, queue_size=PUBLISHER_QUEUE_SIZE)
        # Print a message to the console
        rospy.loginfo("TouchReader node initialized")

    
    def touch_callback(self, data):

        try:
            robot_head = rospy.wait_for_message(self.TOUCH_HEAD_TOPIC, touchdata, timeout=rospy.Duration.from_sec(WAIT_DURATION))     # Wait for messages from the IMU topic
            robot_head_values = [robot_head.pressure_value, robot_head.flexSx_value, robot_head.flexDx_value]
        except:
            rospy.logwarn('[HEAD]:Timeout reached while waiting for messages')
            robot_head_values = [0, 0, 0]

        try:
            robot_motion = rospy.wait_for_message(self.MOVEMENT_TOPIC, Twist, timeout=rospy.Duration.from_sec(WAIT_DURATION))   # Wait for messages from the movement topic
            robot_AccRot = [robot_motion.linear.x, robot_motion.linear.y, robot_motion.angular.z]
        except Exception as e:
            rospy.loginfo(e)
            rospy.logwarn('[TELEOP]:Timeout reached while waiting for messages')
            robot_AccRot = [0, 0, 0]
       
        try:
            robot_orientation = rospy.wait_for_message(self.IMU_TOPIC, Imu, timeout=rospy.Duration.from_sec(WAIT_DURATION))     # Wait for messages from the IMU topic
            robot_RollPitchYaw = self.quaternion2euler(robot_orientation.orientation.x, robot_orientation.orientation.y, robot_orientation.orientation.z, robot_orientation.orientation.w) # Convert the quaternion to Euler angles
        except Exception as a:
            rospy.loginfo(a)
            rospy.logwarn('[IMU]:Timeout reached while waiting for messages')
            robot_RollPitchYaw = [0, 0, 0]

        # Calculate the time difference
        time_diff = time.time() - self.start_time if hasattr(self, 'start_time') else 0

        # Save a new row in the DataFrame
        new_row = [{'Time': time_diff, 'Roll': robot_RollPitchYaw[0], 'Pitch': robot_RollPitchYaw[1], 'Yaw': robot_RollPitchYaw[2],
                     'AccX': robot_AccRot[0], 'AccY': robot_AccRot[1],'RotZ': robot_AccRot[2],
                     'Pressure1': data.pressure_values[0], 'FlexSx1': data.flex_sx_values[0], 'FlexDx1': data.flex_dx_values[0],
                     'Pressure2': data.pressure_values[1], 'FlexSx2': data.flex_sx_values[1], 'FlexDx2': data.flex_dx_values[1],
                     'Pressure3': data.pressure_values[2], 'FlexSx3': data.flex_sx_values[2], 'FlexDx3': data.flex_dx_values[2],
                     'Pressure4': data.pressure_values[3], 'FlexSx4': data.flex_sx_values[3], 'FlexDx4': data.flex_dx_values[3],
                     'Pressure5': data.pressure_values[4], 'FlexSx5': data.flex_sx_values[4], 'FlexDx5': data.flex_dx_values[4],
                     'PressureHead': robot_head_values[0], 'FlexSxHead': robot_head_values[1], 'FlexDxHead': robot_head_values[2]}]
        df_new = pd.DataFrame(new_row)
        self.df = pd.concat([self.df, df_new], ignore_index=True)


    # Convert a quaternion to Euler angles
    def quaternion2euler(self, orientationX, orientationY, orientationZ, orientationW):
        # Source: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-euler-angle/
        
        # Unpack the quaternion values
        x = orientationX
        y = orientationY
        z = orientationZ
        w = orientationW

        # Convert to Euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return [roll, pitch, yaw]
    
    # Save the data to a CSV file
    def save_data(self):
        filename  =  time.strftime("%Y-%m-%d_%H.%M.%S") + '_touchData.csv'
        self.df.set_index('Time', inplace=True)
        self.df.to_csv('../Desktop/TouchData/' + filename)
        rospy.loginfo('Data saved to file')
        return

# ------------------------------
# Main function
# ------------------------------

if __name__ == '__main__':
    # Create the node object
    node = TouchReaderManager()

    # Start&Stop
    rospy.loginfo('Touch manager node started...')
    rospy.spin()
    node.save_data()
    rospy.loginfo('Touch manager node stopped')
    
    exit(0)