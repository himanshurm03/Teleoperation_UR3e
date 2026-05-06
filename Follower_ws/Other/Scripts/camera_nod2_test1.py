#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os 
import sys
from rospy import Time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

global first_string
first_string='0'

global round_way
round_way = 0.0

global pub
pub = rospy.Publisher("time_out_slave",String,queue_size=10)

global js_start
js_start = True

global teleop_file_ur3e
teleop_file_ur3e = "Teleoperation file"

global t_init
t_init = 0.0

def capture_and_publish():
    global first_string, pub

    rospy.init_node('camera_client_node', anonymous=True)
    image_pub = rospy.Publisher('/camera_feed', Image, queue_size= None)

    # Set camera resolution to 640x480
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240) # Modifies from 480 to 360

    cap.set(cv2.CAP_PROP_FPS, 30)  # Adjust the desired FPS
    
    bridge = CvBridge()

    rate = rospy.Rate(30)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            image_message = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            t_now_1 = "%s" % (rospy.get_time() - 1719404870.74)
            t_now_1 += " " + first_string
            pub.publish(t_now_1) 
            image_pub.publish(image_message)
            print('t_now_1:',t_now_1)
        rate.sleep()

def Time_callback(msg5):

    global first_string,round_way,t_init,teleop_file_ur3e,js_start

    if js_start:
        js_start = False
        t_init = time.time()

    t_now = time.time() - t_init


    time_data = msg5.data
    separated_string = time_data.split(" ")
    first_string = separated_string[0]
    second_string = separated_string[1]
    t_ur5 = (rospy.get_time() - 1719404870.74)
    round_way =  ( t_ur5 - float(second_string))


    if t_now > 0.5 :

        data_1 = str(t_now) + ',' + str(round_way) 
        data_2 = data_1 + ',' + '\n'

        file = open(teleop_file_ur3e, 'a')   #a = append
        file.write(data_2)
        file.close()


    print('round_way',round_way,'t_ur5',t_ur5,'second_string',separated_string)


def main():

    secs = time.time()
    tt = time.localtime(secs)
    t = time.asctime(tt)
    global teleop_file_ur3e
    file_name = 'teleop_vel_control_electrical'  + str(t) + '.csv'
    save_path_os = '/home/user/catkin_UR_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/data_files/files_through_os_module'
    teleop_file_ur3e = os.path.join(save_path_os, file_name)
    file = open(teleop_file_ur3e,'w')
    file.close()

    rospy.Subscriber("time_out",String,Time_callback)
    capture_and_publish()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
