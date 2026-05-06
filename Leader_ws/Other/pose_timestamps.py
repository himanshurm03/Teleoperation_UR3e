#!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import PoseStamped
# import csv
# import os


# class TimestampLoggerNode:
#    def __init__(self):
#        # Initialize the ROS node
#        rospy.init_node('timestamp_logger_node', anonymous=True)


#        # File path for saving
#        self.filepath = "/home/autonomous-lab/Desktop/poselog1.csv"


#        # In-memory list to store timestamps
#        self.timestamp_list = []


#        # Subscribe to the pose topic
#        self.pose_sub = rospy.Subscriber('/pose_from_master_to_slave', PoseStamped, self.pose_callback)


#        # Register shutdown hook
#        rospy.on_shutdown(self.save_csv)


#        rospy.loginfo("Timestamp Logger Node started. Logging header.stamp from /pose_from_master_to_slave")


#    def pose_callback(self, msg):
#        header_stamp = msg.header.stamp
#        timestamp_sec = header_stamp.secs
#        timestamp_nsec = header_stamp.nsecs
#        timestamp_float = header_stamp.to_sec()


#        # Save in memory
#        self.timestamp_list.append([timestamp_sec, timestamp_nsec, timestamp_float])


#        rospy.loginfo(f"Logged header.stamp: {timestamp_float:.9f}")


#    def save_csv(self):
#        # Ensure directory exists
#        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
#        try:
#            with open(self.filepath, 'w', newline='') as f:
#                writer = csv.writer(f)
#                writer.writerow(["Header Sec", "Header Nsec", "Header Time (s)"])
#                writer.writerows(self.timestamp_list)
#            rospy.loginfo(f"Timestamps saved to: {self.filepath}")
#        except Exception as e:
#            rospy.logerr(f"Failed to write timestamps to CSV: {e}")


# if __name__ == '__main__':
#    try:
#        node = TimestampLoggerNode()
#        rospy.spin()
#    except rospy.ROSInterruptException:
#        pass


import rospy
from geometry_msgs.msg import PoseStamped
import csv
import os


class TimestampLoggerNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('timestamp_logger_node', anonymous=True)

        # File path for saving
        self.filepath = "/home/autonomous-lab/Desktop/packetloss/posetimestamps.csv"

        # In-memory list to store timestamps
        self.timestamp_list = []

        # Subscribe to the pose topic
        self.pose_sub = rospy.Subscriber('/pose_from_master_to_slave', PoseStamped, self.pose_callback)

        # Register shutdown hook
        rospy.on_shutdown(self.save_csv)

        rospy.loginfo("Timestamp Logger Node started. Logging only header.stamp.to_sec() from /pose_from_master_to_slave")

    def pose_callback(self, msg):
        timestamp_float = msg.header.stamp.to_sec()

        # Save in memory as a list with one element (for CSV writer)
        self.timestamp_list.append([timestamp_float])

        rospy.loginfo(f"Logged header.stamp.to_sec(): {timestamp_float:.9f}")

    def save_csv(self):
        # Ensure directory exists
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        try:
            with open(self.filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Header Time (s)"])
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



