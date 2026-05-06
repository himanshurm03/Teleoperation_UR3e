#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
import time
import os

global first_string
first_string = "0"

global t_force_round
t_force_round = 0.0

global pub 
pub = rospy.Publisher('time_out', String , queue_size=10)

global teleop_file_ur3e
teleop_file_ur3e = 'Teleopertaion file'

global js_start
js_start = True

global t_init
t_init = 0.0

def resize_image(cv_image, width, height):
     return cv2.resize(cv_image, (width, height))

def image_callback(msg10):
        global rate
        
        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(msg10,desired_encoding= 'passthrough')
        cv_image_resized = resize_image(cv_image, 640, 480 ) #320, 240,  # 640 480  # right now 800 600

#1280, 720
        # Display the image
        cv2.imshow("Server Side: Camera Feed", cv_image_resized)
        key = cv2.waitKey(1) & 0xFF 

        if key == ord('q') or key == ord('Q'):  # Check if 'Q' key is pressed
            rospy.signal_shutdown("Q key pressed")

        rate.sleep()
        
def time_callback(arg3):

    global first_string,t_force_round,pub,js_start,teleop_file_ur3e,t_init

    if js_start:
         js_start = False
         t_init = time.time()
    t_now = time.time() - t_init

    time_data = arg3.data
    separated_string = time_data.split(" ")
    first_string = separated_string[0]
    print(f"first_string: {first_string}")
    second_string = separated_string[1]
    print(f"second_string: {second_string}")
    t_force =  (rospy.get_time() - 1719404870.74)
    print(f"t_force: {t_force}")
   
    t_force_round = t_force - (float(second_string))
    print('first string',first_string,'separated string',separated_string,'roundtrip',t_force_round)  ## as usual
    
    if t_now > 0.5:
        data_1 = str(t_now) + ',' + str(t_force_round)
        data_2 = data_1 + ',' + '\n'
        file = open(teleop_file_ur3e, 'a')
        file.write(data_2)
        file.close()

    hello_str = "%s" % (rospy.get_time() - 1719404870.74)
    hello_str += " " + first_string
    pub.publish(hello_str)####wireshark 
    print(f"hello_str: {hello_str}")



def image_subscriber():

    global rate, teleop_file_ur3e

    secs = time.time()
    tt = time.localtime(secs)
    t = time.asctime(tt)

    file_name = 'RoboLab_hussernet_circle_v2_k8.5_camera'+str(t)+'.csv'
    save_path_os = '/home/aditya/Downloads/data_files_delay'
    teleop_file_ur3e = os.path.join(save_path_os, file_name)
    file = open(teleop_file_ur3e,'w')
    file.close()

    rospy.Subscriber("time_out_slave", String,time_callback)
    rospy.init_node('image_subscriber', anonymous=True)
    rate = rospy.Rate(30)
    rospy.Subscriber('/camera_feed', Image, image_callback, queue_size=None)
    rospy.spin()


if __name__ == '__main__':
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass
