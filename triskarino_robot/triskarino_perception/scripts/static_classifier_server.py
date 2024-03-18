#!/usr/bin/env python3
import rospy, pandas as pd, numpy as np, pickle
from scipy.signal import find_peaks
from itertools import product
from tsfresh import extract_features


from triskarino_perception.srv import getTouch          # Import custom service types
from std_msgs.msg import String                         # Import standard message types


class TouchClassificationServer():
    NODE_NAME = 'static_classifier_server'
    STA_SERVICE_NAME = 'static_classifier'

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


    def handle_static_classification(self, req): 

        #### Initialization of the message ####
        classification_message = String()

        #### Negative pressure ####
        neg_pressure = [-x for x in req.Pressure]

        #### Dataframe ####
        df = pd.DataFrame({'Time': req.Time, 'Flexsx': req.FlexSx, 'Flexdx': req.FlexDx, 'Pressure': req.Pressure, 'id': [0]*(len(req.Time))})

        # ---------------------------------------------------------------------------#
                                    # REGOLE #
        # ---------------------------------------------------------------------------#


        #### Carezza ####
        #features_carezze = extract_features(df, column_id='id', column_sort='Time', kind_to_fc_parameters = self.kind_carezze)
        #predictions_carezze = self.model_carezze.predict(features_carezze)
        predictions_carezze = ['Ahia']
        if 'Carezza' in predictions_carezze:
            classification_message.data = "Carezza"
            return classification_message
        
        
        #### TapTap ####
        #features_taptap = extract_features(df, column_id='id', column_sort='Time', kind_to_fc_parameters = self.kind_taptap)
        #predictions_taptap = self.model_taptap.predict(features_taptap)
        predictions_taptap = ['Ahia']
        if 'TapTap' in predictions_taptap:
            classification_message.data = "TapTap"
            return classification_message
        
        
        #### Spinta ####
        peaks_spinta, _ = find_peaks(req.Pressure, prominence=self.th_spinta_up)
        peaks_spinta_neg, _ = find_peaks(neg_pressure, prominence=self.th_spinta_down)
        array_up = peaks_spinta.astype(int)
        array_down = peaks_spinta_neg.astype(int)
        valid_pairs = [(m,n) for m,n in product(array_up, array_down) if n-m>self.dist_spinta_min and n-m<self.dist_spinta_max]
        if valid_pairs:
            classification_message.data = "Spinta"
            return classification_message
        
        
       #### Colpo Forte #####
        if any(x>self.th_cip for x in req.Pressure):
            classification_message.data = "Colpo Forte"
            return classification_message
        
        #### Colpo Istantaneo ####
        if any(x>self.th_cip for x in req.Pressure):
            diffSx = np.ptp(req.FlexSx[-self.win_cif:])
            diffDx = np.ptp(req.FlexDx[-self.win_cif:])
            if diffSx > self.th_cif or diffDx > self.th_cif:
                classification_message.data = "Colpo Istantaneo"
                return classification_message
        

        #### Solletico ####
        peaks_solletico, _ = find_peaks(req.Pressure, prominence=self.th_solletico)
        if len(peaks_solletico) > self.n_solletico:
            classification_message.data = "Solletico"
            return classification_message
        
        #### Tocco ####
        if any(x>self.th_tocco for x in req.Pressure) or len(peaks_solletico) > 3:
            classification_message.data = "Tocco"
            return classification_message
        
        #### No touch ####
        classification_message.data = ""
        return classification_message


if __name__ == '__main__':
    static_classifier = TouchClassificationServer()
    rospy.loginfo( "Server classification ready" )
    rospy.spin()
    rospy.loginfo( "Server classification stopped" )
    exit(0)
