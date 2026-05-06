#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseStamped
import csv
import os


class TimestampLoggerNode:
   def __init__(self):
       # Initialize the ROS node
       rospy.init_node('timestamp_logger_node', anonymous=True)


       # File path for saving
       self.filepath = "/home/user/Desktop/poselog.csv"


       # In-memory list to store timestamps
       self.timestamp_list = []


       # Time reference for zero
       self.start_time = None


       # Subscribe to the pose topic
       self.pose_sub = rospy.Subscriber('/pose_from_master_to_slave', PoseStamped, self.pose_callback)


       # Register shutdown hook
       rospy.on_shutdown(self.save_csv)


       rospy.loginfo("Timestamp Logger Node started. Logging header.stamp from /pose_from_master_to_slave")


   def pose_callback(self, msg):
       header_stamp = msg.header.stamp
       timestamp_float = header_stamp.to_sec()


       # Set the start time on first message
       if self.start_time is None:
           self.start_time = timestamp_float


       time_since_start = timestamp_float - self.start_time


       # Save the values
       self.timestamp_list.append([
           header_stamp.secs,
           header_stamp.nsecs,
           round(time_since_start, 9)
       ])


       rospy.loginfo(f"Logged header.stamp (relative): {time_since_start:.9f} seconds")


   def save_csv(self):
       # Ensure directory exists
       os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
       try:
           with open(self.filepath, 'w', newline='') as f:
               writer = csv.writer(f)
               writer.writerow(["Header Sec", "Header Nsec", "Time Since Start (s)"])
               writer.writerows(self.timestamp_list)
           rospy.loginfo(f"Timestamps saved to: {self.filepath}")
       except Exception as e:
           rospy.logerr(f"Failed to write timestamps to CSV: {e}")


if __name__ == '__main__':
   try:
       node = TimestampLoggerNode()
       rospy.spin()
   except rospy.ROSInterruptException:
       pass



