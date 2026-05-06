#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
import time
import os

class ft_master():

    def __init__(self):
        rospy.init_node('force_delay_calculator', anonymous = True)
        self.subscriber = rospy.Subscriber('ft_time_slave', String, self.sub_callback)
        self.pub = rospy.Publisher('ft_time_master', String, queue_size = 1)

        self.first_string = "0"

    def sub_callback(self, arg3):

        if self.js_start:
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
    
        hello_str = String()
        hello_str = "%s" % (rospy.get_time() - 1719404870.74)
        hello_str += " " + first_string
        self.pub.publish(hello_str)
        print(f"hello_str: {hello_str}")

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = ft_master()
        node.main()
    except rospy.RosInterruptException:
        pass