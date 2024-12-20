#!/usr/bin/env python3
import rospy, pandas as pd, numpy as np, pickle
from scipy.signal import find_peaks
from itertools import product
from tsfresh import extract_features


from triskarino_perception.srv import getTouch          # Import custom service types
from std_msgs.msg import String                         # Import standard message types


class TouchClassificationServer():
    """
    A ROS service server for classifying different types of touch interactions based on sensor data.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        STA_SERVICE_NAME (str): The name of the ROS service.
        model_carezze_path (str): Path to the Carezza model file.
        kind_carezze_path (str): Path to the Carezza parameters file.
        model_taptap_path (str): Path to the TapTap model file.
        kind_taptap_path (str): Path to the TapTap parameters file.
        th_tocco (float): Threshold for the Tocco interaction.
        prominence_tocco (int): Prominence threshold for the Tocco interaction.
        th_cip (float): Threshold pressure for the Colpo Istantaneo interaction.
        th_cif (int): Threshold flex for the Colpo Istantaneo interaction.
        win_cif (int): Window for the flex of the Colpo Istantaneo interaction.
        th_spinta_up (float): Prominence threshold (Up) for the Spinta interaction.
        th_spinta_down (float): Prominence threshold (Down) for the Spinta interaction.
        dist_spinta_min (int): Minimum distance between peaks for the Spinta interaction.
        dist_spinta_max (int): Maximum distance between peaks for the Spinta interaction.
        th_solletico (int): Prominence threshold for peaks for the Solletico interaction.
        n_solletico (int): Number of peaks for the Solletico interaction.
        model_carezze (object): Loaded Carezza model.
        kind_carezze (dict): Parameters for the Carezza model.
        model_taptap (object): Loaded TapTap model.
        kind_taptap (dict): Parameters for the TapTap model.
        service (rospy.Service): ROS service for static classification.
    Methods:
        __init__(): Initializes the TouchClassificationServer, loads models, and sets up the ROS node and service.
        filter_predictions_by_priority(predictions, priority): Filters the touch predictions based on their priority.
        handle_static_classification(req): Handles the static classification service request and returns the classification result.
    """

    NODE_NAME = 'static_classifier_server'
    STA_SERVICE_NAME = 'static_classifier'
    #TODO: Get the paths to the model with rospack
    model_carezze_path = '/home/jetson/catkin_ws/src/triskarino_robot/triskarino_perception/resources/carezze_model.pkl'
    kind_carezze_path = '/home/jetson/catkin_ws/src/triskarino_robot/triskarino_perception/resources/kind_carezze.pkl'
    model_taptap_path = '/home/jetson/catkin_ws/src/triskarino_robot/triskarino_perception/resources/taptap_model.pkl'
    kind_taptap_path = '/home/jetson/catkin_ws/src/triskarino_robot/triskarino_perception/resources/kind_taptap.pkl'


    def __init__(self):

        #### Parameters ####

        ## Tocco ##
        self.th_tocco, self.prominence_tocco = 2.5, 1300  # Threshold for the Tocco

        ## Colpo Istantaneo ##
        self.th_cip, self.th_cif, self.win_cif = 7, 7500, 1 # Threshold pressure, threshold flex, window for the flex of the Colpo Istantaneo

        ## Spinta ##
        self.th_spinta_up, self.th_spinta_down, self.dist_spinta_min, self.dist_spinta_max = 0.75, 0.5, 3, 12  # Prominance thresholds (Up and Down), min and max distance between peaks for the Spinta

        ## Solletico ##
        self.th_solletico, self.n_solletico = 1300, 5  # Prominance threshold for peaks, number of peaks for the Solletico

        ## Carezza & TapTap ##
        with open(self.model_carezze_path, 'rb') as model_file:
            self.model_carezze = pickle.load(model_file)
        with open(self.kind_carezze_path, 'rb') as params_file:
            self.kind_carezze = pickle.load(params_file)
        
        with open(self.model_taptap_path, 'rb') as model_file:
            self.model_taptap = pickle.load(model_file)
        with open(self.kind_taptap_path, 'rb') as params_file:
            self.kind_taptap = pickle.load(params_file)


        #### Initialization of the node ####
        rospy.init_node(self.NODE_NAME)
        self.service = rospy.Service(self.STA_SERVICE_NAME, getTouch, self.handle_static_classification)

    def filter_predictions_by_priority(self, predictions, priority={"Carezza": 5, "TapTap": 4, "Spinta": 3, "Colpo Forte": 2, "Colpo Istantaneo": 1, "Tocco": 0}):
        #No Touch
        curr_type_of_touch = ""
        curr_priority = -1
        for type_of_touch, value in predictions.items():
            if value:
                if priority[type_of_touch] > curr_priority:
                    curr_type_of_touch = type_of_touch
                    curr_priority = priority[type_of_touch]
        return curr_type_of_touch



    def handle_static_classification(self, req): 
        #### Initialization of the message ####
        classification_message = String()

        pressure_list = [msg.data if msg.data != float('nan') else 0  for msg in req.Pressure]
        flexsx_list = [msg.data if msg.data != float('nan') else 0 for msg in req.FlexSx]
        flexdx_list = [msg.data if msg.data != float('nan') else 0 for msg in req.FlexDx]
        time_list = [msg.data if msg.data != float('nan') else 0 for msg in req.Time]

        #### Negative pressure ####
        neg_pressure = [-x for x in pressure_list]

        
        #### Dataframe ####
        df = pd.DataFrame({'Time': time_list, 'Flexsx': flexsx_list, 'Flexdx': flexdx_list, 'Pressure': pressure_list, 'id': [0]*(len(time_list))})

        # ---------------------------------------------------------------------------#
                                    # REGOLE #
        # ---------------------------------------------------------------------------#
        predictions = {"Carezza":False, "TapTap":False, "Spinta": False, "Colpo Forte": False, "Colpo Istantaneo": False, "Solletico": False, "Tocco": False}

        #### Carezza ####
        try:
            features_carezze = extract_features(df, column_id='id', column_sort='Time', kind_to_fc_parameters = self.kind_carezze)
            predictions_carezze = self.model_carezze.predict(features_carezze.values)
        except: 
            rospy.logwarn("Nan in input of carezze extract features")
            predictions_carezze = [False]

        if predictions_carezze[0]:
            predictions["Carezza"] = True
        
        
        #### TapTap ####
        try:
            features_taptap = extract_features(df, column_id='id', column_sort='Time', kind_to_fc_parameters = self.kind_taptap)
            predictions_taptap = self.model_taptap.predict(features_taptap.values)
        except:
            rospy.logwarn("Nan in input of taptap extract features")
            predictions_taptap = [False]

        rospy.loginfo("Predictions TapTap are " + str(predictions_taptap))
        if predictions_taptap[0]:
            predictions["TapTap"] = True
        
        rospy.logwarn("BEFORE SPINTA")
        #### Spinta ####
        peaks_spinta, _ = find_peaks(pressure_list, prominence=self.th_spinta_up)
        peaks_spinta_neg, _ = find_peaks(neg_pressure, prominence=self.th_spinta_down)
        array_up = peaks_spinta.astype(int)
        array_down = peaks_spinta_neg.astype(int)
        valid_pairs = [(m,n) for m,n in product(array_up, array_down) if n-m>self.dist_spinta_min and n-m<self.dist_spinta_max]
        if valid_pairs:
            predictions["Spinta"] = True
        
        
       #### Colpo Forte #####
        if any(x>self.th_cip for x in pressure_list):
            predictions["Colpo Forte"] = True


        #### Colpo Istantaneo ####
        if any(x>self.th_cip for x in pressure_list):
            diffSx = np.ptp(flexsx_list[-self.win_cif:])
            diffDx = np.ptp(flexdx_list[-self.win_cif:])
            if diffSx > self.th_cif or diffDx > self.th_cif:
                predictions["Colpo Istantaneo"] = True
        

        #### Solletico ####
        peaks_solletico, _ = find_peaks(pressure_list, prominence=self.th_solletico)
        if len(peaks_solletico) > self.n_solletico:
            predictions["Solletico"] = True
        #### Tocco ####
        if any(x>self.th_tocco for x in pressure_list) or len(peaks_solletico) > 3:
            predictions["Tocco"] = True

        #### No touch ####
        rospy.loginfo("Predictions are " + str(predictions))
        classification_message.data = self.filter_predictions_by_priority(predictions)
        return classification_message
        

if __name__ == '__main__':
    static_classifier = TouchClassificationServer()
    rospy.loginfo( "Server classification ready" )
    rospy.spin()
    rospy.loginfo( "Server classification stopped" )
    exit(0)
