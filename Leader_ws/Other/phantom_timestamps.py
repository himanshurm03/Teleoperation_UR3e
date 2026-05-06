#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import csv
import os

class PhantomPoseTimestampLogger:
    def __init__(self):
        rospy.init_node('phantom_pose_timestamp_logger', anonymous=True)

        # File path for saving CSV
        self.filepath = "/home/autonomous-lab/Desktop/delay/phantom_pose_timestamps.csv"
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)

        # Open CSV for writing immediately
        self.csv_file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["Phantom Pose Header Time (s)"])  # Column header

        # Subscribe to /phantom/phantom/pose
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_callback)

        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo("Started logging timestamps from /phantom/phantom/pose")

    def pose_callback(self, msg):
        timestamp = msg.header.stamp.to_sec()
        self.writer.writerow([timestamp])
        self.csv_file.flush()  # Write immediately to avoid data loss
        rospy.loginfo(f"Logged: {timestamp:.9f}")

    def shutdown_hook(self):
        rospy.loginfo("Shutting down logger...")
        if not self.csv_file.closed:
            self.csv_file.close()
        rospy.loginfo(f"Timestamps saved to: {self.filepath}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = PhantomPoseTimestampLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
