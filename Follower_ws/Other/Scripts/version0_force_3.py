#!/usr/bin/env python3

import rospy
import csv
import os
from geometry_msgs.msg import WrenchStamped
import time

class FTWrenchLogger:
    def __init__(self):
        rospy.init_node("ft_wrench_logger", anonymous=True)

        # Path for CSV file
        save_dir = "/home/user/Desktop/delay"
        os.makedirs(save_dir, exist_ok=True)
        self.csv_path = os.path.join(save_dir, "ft_wrench_log.csv")

        # Open CSV file
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "ROS_Time (s)",
            "Wall_Time (s)",
            "Force_X (N)", "Force_Y (N)", "Force_Z (N)",
            "Torque_X (Nm)", "Torque_Y (Nm)", "Torque_Z (Nm)"
        ])

        # Subscriber to FT sensor topic
        rospy.Subscriber("/ft_wrench", WrenchStamped, self.callback)

        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo(f"Logging /ft_wrench data to {self.csv_path}")

    def callback(self, msg: WrenchStamped):
        ros_time = msg.header.stamp.to_sec()
        wall_time = time.time()

        force = msg.wrench.force
        torque = msg.wrench.torque

        self.csv_writer.writerow([
            f"{ros_time:.9f}", f"{wall_time:.9f}",
            f"{force.x:.6f}", f"{force.y:.6f}", f"{force.z:.6f}",
            f"{torque.x:.6f}", f"{torque.y:.6f}", f"{torque.z:.6f}"
        ])

    def shutdown_hook(self):
        rospy.loginfo("Shutting down FTWrenchLogger, closing CSV file.")
        self.csv_file.close()

if __name__ == "__main__":
    try:
        FTWrenchLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
