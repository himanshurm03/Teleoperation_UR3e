#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

global temp 
temp = 0

def timer(seconds):
    global temp 
    temp = temp + 0.5
    if temp >= seconds:
        rospy.signal_shutdown("Timer finished")

if __name__ == "__main__":

    try:
        rospy.init_node("velocity_control_aditya", anonymous=True)
        rospy.loginfo("The node has been started!")
        pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray,  queue_size = 10)
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            msg = Float64MultiArray()
            msg.data = [0.1, 0, 0, 0, 0, 0]
            pub.publish(msg)
            rate.sleep()
            timer(5) 
            
        rospy.loginfo("The node has been ended!")

    except rospy.ROSInterruptException:
        pass