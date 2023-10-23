#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import re
import pandas as pd
import seaborn as sns
import time 
import matplotlib.pyplot as plt
import numpy as np
import math
import os 
from triskarino_msgs.msg import RawOdometry


PUBLISHER_QUEUE_SIZE = 100

class DebugOdometryNode():
    NODE_NAME = "debugOdometry"
    def __init__(self):
        rospy.init_node("debugOdometry")
        self.debug_sub = rospy.Subscriber('rawOdometry', RawOdometry, self.log_odom)
        self.debug_df = pd.DataFrame(columns=['Pos X','Pos Y','Pos Th','Speed X','Speed Y','Speed Th','Time'],dtype=object)
        self.start_time = time.time()

    def reset_graph_settings(self):
        #Setting color_palette, diagram style(background), font and font size
        sns.set_theme()
        fig_size=(11.7,8.27)
        sns.set(rc={'figure.figsize':fig_size})
        sns.set_context("paper")
        sns.color_palette("colorblind")
        sns.despine()

    def log_odom(self,debug_msg):
       rospy.loginfo("Received " + str(debug_msg))
       elapsed_time = time.time() - self.start_time
       new_row = {'Pos X':debug_msg.odometryPos[0],
                  'Pos Y':debug_msg.odometryPos[1],
                  'Pos Th':debug_msg.odometryPos[2],
                  'Speed X':debug_msg.odometryVel[0],
                  'Speed Y':debug_msg.odometryPos[1],
                  'Speed Th':debug_msg.odometryPos[2],
                  'Time':elapsed_time
                  }
       self.debug_df = self.debug_df.append(new_row,ignore_index=True)
       rospy.loginfo("Processed " + str(new_row))
    
    def save_plots(self):
        self.reset_graph_settings()
        fig, ax = plt.subplots()
        ax.plot(self.debug_df["Pos X"],
           self.debug_df["Pos Y"],
           linestyle="solid",
           label="Odometry",
            color="#728FA5")
        if os.path.isfile("./odometry_fig.png"):
            os.remove("./odometry_fig.png")
        fig.savefig("./odometry_fig.png",bbox_inches='tight')
    
    
   

if __name__ == '__main__':
    print("AO")
    node = DebugOdometryNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    rospy.spin()
    node.save_plots()
    node.debug_df.to_csv("./odom_df.csv")
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
