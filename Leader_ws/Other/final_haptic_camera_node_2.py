#!/usr/bin/env python3

import rospy
import math
# from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback
from omni_msgs.msg import OmniState
import numpy as np
from cv2 import waitKey
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import sys

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String

from std_msgs.msg import Int32
import time 
import csv
from os import path
from std_msgs.msg import Float32MultiArray
from rospy import Time

def resize_image(cv_image, width, height):
     return cv2.resize(cv_image, (width, height))

def image_callback(msg10):
        global rate
        
        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()
   
        image_sizee = sys.getsizeof(msg10.data)
        #rospy.loginfo(f"size of received image packet: {image_sizee} bytes")
        #print('image size',image_sizee)

        cv_image = bridge.imgmsg_to_cv2(msg10,desired_encoding= 'passthrough')
        cv_image_resized = resize_image(cv_image, 1280, 720 )  # 640 480  # right now 800 600

        # Display the image
        cv2.imshow("Server Side: Camera Feed", cv_image_resized)
        key = waitKey(1) & 0xFF 

        if key == ord('q') or key == ord('Q'):  # Check if 'Q' key is pressed
            rospy.signal_shutdown("Q key pressed")
        
        #print('raj')
        rate.sleep()
        

 

def image_subscriber():
    global rate
    rospy.init_node('image_subscriber', anonymous=True)
    #### Trial
    average_message_size = 307233  # Replace with the actual average message size
    buffer_size = 30 * average_message_size + 100  # Adjust the multiplier based on your specific use case 600
    #### Trial
    rate = rospy.Rate(30)
    #rospy.Subscriber('/camera_feed', Image, image_callback, queue_size=None, buff_size= buffer_size)  ## 60 2, 
    rospy.Subscriber('/camera_feed', Image, image_callback, queue_size=None)
    rospy.spin()


    

if __name__ == '__main__':
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass
