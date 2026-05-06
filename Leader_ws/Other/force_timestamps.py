#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import csv
import os

class ForceTimestampLogger:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('force_timestamp_logger_from_slave', anonymous=True)

        # Filepath to save the CSV
        self.csv_path = "/home/autonomous-lab/Desktop/delay/forcetimestamps.csv"

        # In-memory list to store timestamps
        self.timestamps = []

        # Subscriber
        rospy.Subscriber('/time_from_slave_to_master_for_force', Float64, self.callback)

        # Register shutdown hook
        rospy.on_shutdown(self.save_to_csv)

        rospy.loginfo("Started logging timestamps from /time_from_slave_to_master_for_force...")

    def callback(self, msg: Float64):
        timestamp = msg.data
        rospy.loginfo(f"Received force timestamp from slave: {timestamp}")
        self.timestamps.append(timestamp)

    def save_to_csv(self):
        try:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            with open(self.csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Timestamp from Slave (s)"])
                for t in self.timestamps:
                    writer.writerow([t])
            rospy.loginfo(f"Timestamps saved to: {self.csv_path}")
        except Exception as e:
            rospy.logerr(f"Error saving CSV: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ForceTimestampLogger()
        node.run()
    except rospy.ROSInterruptException:
        pass
