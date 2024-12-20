#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
import os 
import cv2
import numpy as np

RATE=20
WAIT_DURATION = 0.1
DIR_NAME ="images_recording"
class ImageSaverNode():
    """
    A ROS node for saving compressed images and creating a video from them.
    Attributes:
        NODE_NAME (str): The name of the ROS node.
        compressedImages (dict): A dictionary to store compressed images and their corresponding names.
        out (cv2.VideoWriter): A VideoWriter object to write images into a video file. Video is saved in the Directory DIR_NAME=images_recording with name output_video,mp4
    Methods:
        __init__(): Initializes the ROS node and sets up the VideoWriter.
        save_image(image_msg): Saves a compressed image message to the internal storage.
        save_images(): Saves all stored images to a video file and logs the process.
        spin(): Continuously listens for image messages and saves them until the node is shut down.
    """

    NODE_NAME = "image_saver"
    def __init__(self):
        rospy.init_node("image_saver")
        self.compressedImages = {"images":[], "names":[]}
        self.out = cv2.VideoWriter("./"+DIR_NAME+'/output_video.mp4',cv2.VideoWriter_fourcc(*'MP4V'), RATE, (640,360))


    def save_image(self,image_msg):
        self.compressedImages["images"].append(image_msg.data)
        self.compressedImages["names"].append(str(image_msg.header.frame_id))
        rospy.loginfo("Received " + image_msg.header.frame_id)

    
    def save_images(self):
        print("Saving images...")
        rospy.loginfo("Saving images...")
        if not os.path.exists("./"+DIR_NAME):
            os.makedirs("./"+DIR_NAME)
        img_number = len(self.compressedImages["images"])
        for i in range(img_number):
            self.out.write(cv2.imdecode(np.frombuffer(self.compressedImages["images"][i], np.uint8), cv2.IMREAD_COLOR))
        self.out.release()
        rospy.loginfo("Saved " + str(img_number) + " images in folder " + DIR_NAME)
        rospy.loginfo("First image name is " + self.compressedImages["names"][0])
        rospy.loginfo("Last image name is " + self.compressedImages["names"][-1])

    def spin(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            try:
                image_msg = rospy.wait_for_message('/rpi_camera/image_raw/compressed',CompressedImage, timeout=WAIT_DURATION)
                self.save_image(image_msg)
                rate.sleep()
            except Exception as e:
                print(e)
                continue
        rospy.loginfo("Exiting spin")
        print("Exiting spin")

if __name__ == '__main__':
    print("AO")
    node = ImageSaverNode()
    rospy.loginfo( node.NODE_NAME + " running..." )
    node.spin()
    node.save_images()
    rospy.loginfo( node.NODE_NAME + " stopped." )
    exit(0)
