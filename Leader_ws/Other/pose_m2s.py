#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import csv
import os

class PoseTimestampLogger:
    def __init__(self):
        rospy.init_node('pose_timestamp_logger', anonymous=True)

        # CSV file setup
        self.filepath = "/home/autonomous-lab/Desktop/delay/pose_timestamps.csv"
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        self.csv_file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["Master Timestamp (s)"])  # Only the timestamp sent from master

        # Subscriber
        rospy.Subscriber('time_from_master_to_slave_for_pose', Float64, self.timestamp_callback)

        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("PoseTimestampLogger started. Logging only master timestamps...")

    def timestamp_callback(self, msg):
        self.writer.writerow([msg.data])
        self.csv_file.flush()  # Ensure immediate write
        rospy.loginfo(f"Logged master timestamp: {msg.data}")

    def shutdown_hook(self):
        rospy.loginfo("Shutting down PoseTimestampLogger...")
        if not self.csv_file.closed:
            self.csv_file.close()
        rospy.loginfo(f"Timestamps saved to: {self.filepath}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        logger = PoseTimestampLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
