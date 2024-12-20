#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import re
import pandas as pd
import seaborn as sns
import time 
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import numpy as np
import math
import os 

PUBLISHER_QUEUE_SIZE = 100

class DebugArduinoNode():
    """
    A ROS node for debugging and Tuning PID from the Arduino. This node subscribes to the 'arduinoDebug' topic, processes the incoming
    messages, logs the data, and generates plots for analysis and contains functions to use Ziegler Method.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
    """
    NODE_NAME = "debugArduino"
    def __init__(self):
        rospy.init_node("debugArduino")
        self.debug_sub = rospy.Subscriber('arduinoDebug', String, self.log_debug)
        #SP is the Set Point, PV is the Process Value each for wheel 0,1,2
        self.debug_df = pd.DataFrame(columns=['SP Speed 0','SP Speed 1','SP Speed 2','PV Speed 0','PV Speed 1','PV Speed 2','Time'],dtype=object)
        self.start_time = time.time()

    def reset_graph_settings(self):
        #Setting color_palette, diagram style(background), font and font size
        sns.set_theme()
        fig_size=(11.7,8.27)
        sns.set(rc={'figure.figsize':fig_size})
        sns.set_context("paper")
        sns.color_palette("colorblind")
        sns.despine()

    def log_debug(self,debug_msg):
        """
        Callback function for the 'arduinoDebug' topic. Processes the incoming debug message, extracts the data,
        and appends it to the DataFrame.
        Args:
            debug_msg (std_msgs.msg.String): The debug message received from the 'arduinoDebug' topic.
        """
        rospy.loginfo("Received " + debug_msg.data)
        if len(debug_msg.data) == 0:
           return
        elapsed_time = time.time() - self.start_time
        debug_list = re.findall(r"[-+]?(?:\d*\.*\d+)",debug_msg.data)
        new_row = {'SP Speed 0':float(debug_list[0]),
                  'SP Speed 1':float(debug_list[1]),
                  'SP Speed 2':float(debug_list[2]),
                  'PV Speed 0':float(debug_list[3]),
                  'PV Speed 1':float(debug_list[4]),
                  'PV Speed 2':float(debug_list[5]),
                  'Time':elapsed_time
                  }
        self.debug_df = self.debug_df.append(new_row,ignore_index=True)
        rospy.loginfo("Processed " + str(new_row))
    
    def save_plots(self):
        """
        Generates and saves plots for the debug data. The plots include speed setpoints and process values over time.
        """
        fig, axes = plt.subplots(3,1)
        #Preparing df
        melted_x_df = pd.melt(self.debug_df[["Time","SP Speed 0","PV Speed 0"]],id_vars="Time",value_vars=["SP Speed 0","PV Speed 0"],var_name="",value_name="Speed 0")
        melted_y_df = pd.melt(self.debug_df[["Time","SP Speed 1","PV Speed 1"]],id_vars="Time",value_vars=["SP Speed 1","PV Speed 1"],var_name="",value_name="Speed 1")
        melted_th_df = pd.melt(self.debug_df[["Time","SP Speed 2","PV Speed 2"]],id_vars="Time",value_vars=["SP Speed 2","PV Speed 2"],var_name="",value_name="Speed 2")
        #X Plot
        self.reset_graph_settings()
        sns.lineplot(x="Time",y="Speed 0",hue="",data=melted_x_df,ax=axes[0])
        #Y Plot
        self.reset_graph_settings()
        sns.lineplot(x="Time",y="Speed 1",hue="",data=melted_y_df,ax=axes[1])
        #Th Plot
        self.reset_graph_settings()
        sns.lineplot(x="Time",y="Speed 2",hue="",data=melted_th_df,ax=axes[2])
        if os.path.isfile("./speeds_control_fig.png"):
            os.remove("./speeds_control_fig.png")
        fig.savefig("./speeds_control_fig.png",bbox_inches='tight')
    
    
    def find_peaks(self,data_x,data_y):
        """
        Finds the peaks in the given data.
        Args:
            data_x (array-like): The x-coordinates of the data points.
            data_y (array-like): The y-coordinates of the data points.
        Returns:
            tuple: The x and y coordinates of the peaks.
        """
        peaks, _ = find_peaks(data_y, height=0)
        return data_x[peaks],data_y[peaks]
    
    def find_period(self,peaks_x):
        """
        Finds the average period of oscillations based on the x-coordinates of the peaks.
        Args:
            peaks_x (array-like): The x-coordinates of the peaks.
        Returns:
            float: The average period of the oscillations.
        """
        #Finds the difference between each subsequent peak's x coordinate
        periods = [peak - peaks_x[idx - 1] for idx, peak in enumerate(peaks_x) if idx > 0] 
        #Between all of the differences, creates a mask that highlights subsequent peaks with similar distances (continuous oscillations)
        mask = [1 if math.isclose(element,periods[idx-1],rel_tol=0.1) else 0 for idx, element in enumerate(periods) if idx > 0]
        #Finds the idxs of the periods that highlight the longest continuous oscillation
        idxs = self.find_idxs_mask(mask)
        #Gets the average period of the sequence
        if len(idxs) > 0:
            average_period = np.asarray(periods)[idxs].mean()
        else:
            average_period = 0
        return average_period

    #Returns the idxs of the longest continuous sequences of 1s in the mask 
    def find_idxs_mask(self,mask):
        """
        Finds the indices of the longest continuous sequence of 1s in the mask.
        Args:
            mask (array-like): The mask array containing 1s and 0s.
        Returns:
            list: The indices of the longest continuous sequence of 1s.
        """
        counter = 0
        max_val = 0
        idxs_max = []
        idxs_temp = []
        for idx, elem in enumerate(mask):
            if elem == 1:
                counter = counter + 1 
                idxs_temp.append(idx)
                max_val = max(max_val,counter)
                idxs_max = idxs_temp.copy() if len(idxs_temp) > len(idxs_max) else idxs_max
            if elem == 0:
                counter = 0
                idxs_temp.clear()
        if len(idxs_max) > 0:
            idxs_max.append(idxs_max[-1]+1)
        return idxs_max








if __name__ == '__main__':
    print("AO")
    node = DebugArduinoNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    node.save_plots()
    node.debug_df.to_csv("./pid_df.csv")
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
