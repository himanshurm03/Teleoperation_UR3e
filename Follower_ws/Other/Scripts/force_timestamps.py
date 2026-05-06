#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Float64
# import csv
# import os

# class ForceTimestampLogger:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('force_timestamp_logger', anonymous=True)

#         # File path for saving CSV
#         self.csv_path = "/home/user/Desktop/force_timestamp2.csv"

#         # In-memory lists to store timestamps
#         self.timestamps_sent_to_master = []  # from /time_from_slave_to_master_for_force
#         self.timestamps_received_back = []   # from /time_from_master_to_slave_for_force

#         # Subscribers
#         rospy.Subscriber('/time_from_slave_to_master_for_force', Float64, self.sent_timestamp_callback)  #fix this
#         rospy.Subscriber('/time_from_master_to_slave_for_force', Float64, self.received_timestamp_callback)

#         # Register shutdown hook to save CSV
#         rospy.on_shutdown(self.save_csv)

#         rospy.loginfo("ForceTimestampLogger node started. Listening to timestamp topics.")

#     def sent_timestamp_callback(self, msg):
#         timestamp = msg.data
#         self.timestamps_sent_to_master.append([timestamp])  # Store as list for CSV row
#         rospy.loginfo(f"Logged Sent to Master: {timestamp:.9f}")

#     def received_timestamp_callback(self, msg):
#         timestamp = msg.data
#         self.timestamps_received_back.append([timestamp])  # Store as list for CSV row
#         rospy.loginfo(f"Logged Received from Master: {timestamp:.9f}")

#     def save_csv(self):
#         # Ensure directory exists
#         os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

#         try:
#             max_len = max(len(self.timestamps_sent_to_master), len(self.timestamps_received_back))

#             # Pad shorter list with empty rows
#             sent = self.timestamps_sent_to_master + [['']] * (max_len - len(self.timestamps_sent_to_master))
#             received = self.timestamps_received_back + [['']] * (max_len - len(self.timestamps_received_back))

#             with open(self.csv_path, 'w', newline='') as f:
#                 writer = csv.writer(f)
#                 writer.writerow(["Timestamp Sent to Master (s)", "Timestamp Received from Master (s)"])
#                 for s, r in zip(sent, received):
#                     writer.writerow([s[0], r[0]])

#             rospy.loginfo(f"Timestamps saved successfully to: {self.csv_path}")
#         except Exception as e:
#             rospy.logerr(f"Failed to save timestamps to CSV: {e}")

# if __name__ == '__main__':
#     try:
#         node = ForceTimestampLogger()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass


import rospy
from omni_msgs.msg import OmniFeedback
import csv
import os


class TimestampLoggerNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('force_timestamp_logger_node', anonymous=True)

        # File path for saving
        self.filepath = "/home/user/Desktop/force_timestamp2.csv"

        # In-memory list to store timestamps
        self.timestamp_list = []

        # Subscribe to the force topic
        self.force_pub = rospy.Subscriber('/force_from_slave_to_master', OmniFeedback, self.force_callback)

        # Register shutdown hook
        rospy.on_shutdown(self.save_csv)

        rospy.loginfo("Timestamp Logger Node started. Logging header.stamp.to_sec() from /force_from_slave_to_master")

    def force_callback(self, msg):
        # Log current ROS time as the timestamp (OmniFeedback doesn't contain a header)
        timestamp_float = rospy.get_time()
        self.timestamp_list.append([timestamp_float])
        rospy.loginfo(f"Logged timestamp: {timestamp_float:.9f}")

    def save_csv(self):
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        try:
            with open(self.filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["ROS Time (s)"])
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
