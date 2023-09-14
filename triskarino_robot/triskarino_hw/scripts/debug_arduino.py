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

PUBLISHER_QUEUE_SIZE = 100

class DebugArduinoNode():
    NODE_NAME = "debugArduino"
    def __init__(self):
        rospy.init_node("debugArduino")
        self.debug_sub = rospy.Subscriber('arduinoDebug', String, self.log_debug)
        self.debug_df = pd.DataFrame(columns=['SP Speed X','SP Speed Y','SP Speed Th','PV Speed X','PV Speed Y','PV Speed Th','Time'],dtype=object)
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
       rospy.loginfo("Received " + debug_msg.data)
       if len(debug_msg.data) == 0:
           return
       elapsed_time = time.time() - self.start_time
       debug_list = re.findall(r"[-+]?(?:\d*\.*\d+)",debug_msg.data)
       new_row = {'SP Speed X':float(debug_list[0]),
                  'SP Speed Y':float(debug_list[1]),
                  'SP Speed Th':float(debug_list[2]),
                  'PV Speed X':float(debug_list[3]),
                  'PV Speed Y':float(debug_list[4]),
                  'PV Speed Th':float(debug_list[5]),
                  'Time':elapsed_time
                  }
       self.debug_df = self.debug_df.append(new_row,ignore_index=True)
       rospy.loginfo("Processed " + str(new_row))
    
    def save_plots(self):
        fig, axes = plt.subplots(3,1)
        #Preparing df
        melted_x_df = pd.melt(self.debug_df[["Time","SP Speed X","PV Speed X"]],id_vars="Time",value_vars=["SP Speed X","PV Speed X"],var_name="",value_name="Speed X")
        melted_y_df = pd.melt(self.debug_df[["Time","SP Speed Y","PV Speed Y"]],id_vars="Time",value_vars=["SP Speed Y","PV Speed Y"],var_name="",value_name="Speed Y")
        melted_th_df = pd.melt(self.debug_df[["Time","SP Speed Th","PV Speed Th"]],id_vars="Time",value_vars=["SP Speed Th","PV Speed Th"],var_name="",value_name="Speed Th")
        #X Plot
        self.reset_graph_settings()
        sns.lineplot(x="Time",y="Speed X",hue="",data=melted_x_df,ax=axes[0])
        #Getting Peaks for X
        data_x, data_y = self.find_peaks(self.debug_df["Time"].to_numpy(),self.debug_df["PV Speed X"].to_numpy())
        axes[0].plot(data_x, data_y, marker="x")
        self.period = self.find_period(data_x)
        #Y Plot
        self.reset_graph_settings()
        sns.lineplot(x="Time",y="Speed Y",hue="",data=melted_y_df,ax=axes[1])
        #Th Plot
        self.reset_graph_settings()
        sns.lineplot(x="Time",y="Speed Th",hue="",data=melted_th_df,ax=axes[2])
        fig.savefig("./speeds_control.png",bbox_inches='tight')
    
    def find_peaks(self,data_x,data_y):
        peaks, _ = find_peaks(data_y, height=0)
        return data_x[peaks],data_y[peaks]
    
    def find_period(self,peaks_x):
        #Finds the difference between each subsequent peak's x coordinate
        periods = [peak - peaks_x[idx - 1] for idx, peak in enumerate(peaks_x) if idx > 0] 
        #Between all of the differences, creates a mask that highlights subsequent peaks with similar distances (continuous oscillations)
        mask = [1 if math.isclose(element,periods[idx-1],rel_tol=0.001) else 0 for idx, element in enumerate(periods) if idx > 0]
        #Finds the idxs of the periods that highlight the longest continuous oscillation
        idxs = self.find_idxs_mask(mask)
        #Gets the average period of the sequence
        average_period = np.asarray(periods)[idxs].mean()
        return average_period

    #Returns the idxs of the longest continuous sequences of 1s in the mask 
    def find_idxs_mask(self,mask):
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
        idxs_max.append(idxs_max[-1]+1)
        return idxs_max








if __name__ == '__main__':
    print("AO")
    node = DebugArduinoNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    node.save_plots()
    rospy.loginfo("Period of oscillation is " + str(node.period))
    node.debug_df.to_csv("./pid_df.csv")
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
