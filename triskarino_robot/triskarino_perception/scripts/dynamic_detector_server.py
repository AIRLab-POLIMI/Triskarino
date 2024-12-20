#!/usr/bin/env python3
import rospy, pandas as pd, numpy as np, pickle

from triskarino_perception.srv import getTouch          # Import custom service types
from std_msgs.msg import String                         # Import standard message types


class TouchDetectionnServer():
    """
    A ROS server for detecting touch motion while the robot is moving.
    Attributes:
        mse_threshold (int): The threshold for the Mean Squared Error (MSE) to detect touch motion.
        NODE_NAME (str): The name of the ROS node.
        STA_SERVICE_NAME (str): The name of the ROS service.
    Methods:
        __init__(): Initializes the ROS node and service.
        handle_dyn_detection(req): Handles the dynamic detection service request.
        calculate_mse(dataDx, dataSx): Calculates the MSE for the right and left sensor data.
    """


    mse_threshold = 5 * 1000000 # Set the threshold for the MSE
    
    NODE_NAME = 'dynamic_detector_server'
    STA_SERVICE_NAME = 'dynamic_detector'

    def __init__(self):        
        rospy.init_node(self.NODE_NAME)
        self.service = rospy.Service(self.STA_SERVICE_NAME, getTouch, self.handle_dyn_detection)

    def handle_dyn_detection(self, req):

        
        flexsx_list = [msg.data for msg in req.FlexSx]
        flexdx_list = [msg.data for msg in req.FlexDx]

        # Initialization of the message
        detection_data = String()
        
        # Calculate the MSE of the window
        mse_Dx, mse_Sx = self.calculate_mse(flexdx_list, flexsx_list)

        # Detect the touch in motion on the right sensor
        for mse_value in mse_Dx:
            if mse_value > self.mse_threshold:
                detection_data.data = "Touch Motion"
                return detection_data
        
        # Detect the touch in motion on the left sensor
        for mse_value in mse_Sx:
            if mse_value > self.mse_threshold:
                detection_data.data = "Touch Motion"
                return detection_data
            
        # If no touch is detected
        detection_data.data = ""
        return detection_data
        
    
    def calculate_mse(self, dataDx, dataSx):
        # Calculate the MSE between the window and the ema for the right sensor
        emaDx = pd.Series(dataDx).ewm(span=5, adjust=True).mean().to_numpy()
        mseDx = np.square(np.subtract(dataDx, emaDx))
        
        # Calculate the MSE between the window and the ema for the left sensor
        emaSx = pd.Series(dataDx).ewm(span=5, adjust=True).mean().to_numpy()
        mseSx = np.square(np.subtract(dataSx, emaSx))

        return mseDx, mseSx
    
        
        

if __name__ == '__main__':    
    node = TouchDetectionnServer()
    rospy.loginfo( "Server detection ready" )
    rospy.spin()
    rospy.loginfo( "Server detection stopped" )
    exit(0)
