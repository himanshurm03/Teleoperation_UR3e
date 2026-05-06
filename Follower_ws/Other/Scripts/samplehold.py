#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import time

class SampleAndHold:
    def __init__(self):
        rospy.init_node('sample_and_hold_node', anonymous=True)

        # Parameters
        self.pose_timeout_threshold = rospy.get_param("~timeout", 0.05)  # 50 ms default
        self.input_topic = rospy.get_param("~input_topic", "/master_pose")
        self.output_topic = rospy.get_param("~output_topic", "/pose_processed")

        # Subscribers and Publishers
        self.pose_sub = rospy.Subscriber(self.input_topic, PoseStamped, self.pose_callback)
        self.pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)

        self.last_pose_msg = None
        self.last_pose_time = time.time()

        self.rate = rospy.Rate(1000)  # 1000 Hz loop

    def pose_callback(self, msg):
        self.last_pose_msg = msg
        self.last_pose_time = time.time()

    def run(self):
        while not rospy.is_shutdown():
            current_time = time.time()
            if self.last_pose_msg is not None:
                if (current_time - self.last_pose_time) > self.pose_timeout_threshold:
                    rospy.logwarn_throttle(1.0, "Sample-and-hold: Holding last valid pose")
                self.pose_pub.publish(self.last_pose_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SampleAndHold()
        node.run()
    except rospy.ROSInterruptException:
        pass
