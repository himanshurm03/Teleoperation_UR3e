#!/usr/bin/env python3
import rospy
import time
from omni_msgs.msg import OmniFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64 
import csv
import os

class MasterController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('master', anonymous=True)
        
        # Delay storage
        self.pose_delays = []
        self.pose_delay_entries = []
        self.force_delays = []
        self.force_delay_entries = []
        self.pose_pub_timestamps = []
        self.pose_gen_send_log = []
        self.latest_pose_msg = None


        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
        self.pose_pub = rospy.Publisher('pose_from_master_to_slave', PoseStamped, queue_size=1)
        self.time_pub_pose = rospy.Publisher('time_from_master_to_slave_for_pose', Float64, queue_size=1)
        self.time_pub_force = rospy.Publisher('time_from_master_to_slave_for_force', Float64, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('force_from_slave_to_master', OmniFeedback, self.force_from_slave_to_master_callback)
        #rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_from_master_to_slave_callback)
        rospy.Subscriber('time_from_slave_to_master_for_pose', Float64, self.time_from_slave_to_master_for_pose_callback)
        rospy.Subscriber('time_from_slave_to_master_for_force', Float64, self.time_from_slave_to_master_for_force_callback)
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_callback)


        # Shutdown flag
        self.shutdown_flag = False
        rospy.on_shutdown(self.shutdown_hook)

    # def time_from_slave_to_master_for_force_callback(self, msg: Float64):
    #     delay = rospy.get_time() - msg.data
    #     print(f"Delay for force: {delay}")
    #     self.force_delays.append(delay)
    #     self.force_delay_entries.append((msg.data, delay))


    def time_from_slave_to_master_for_force_callback(self, msg: Float64):
        delay = rospy.get_time() - msg.data
        print(f"Delay for force: {delay}")
        self.force_delays.append(delay)
        self.force_delay_entries.append((msg.data, delay))

        # Send the same timestamp back to the slave
        self.time_pub_force.publish(msg)

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose_msg = msg



    def time_from_slave_to_master_for_pose_callback(self, msg: Float64):
        delay = rospy.get_time() - msg.data  #round trip delay
        print(f"Delay for pose: {delay}")
        self.pose_delays.append(delay)
        self.pose_delay_entries.append((msg.data, delay)) 

    def force_from_slave_to_master_callback(self, msg: OmniFeedback):
        if not self.shutdown_flag:
            self.force_pub.publish(msg)

    def pose_from_master_to_slave_callback(self, msg: PoseStamped):
        current_time = rospy.get_time()
        self.pose_pub.publish(msg)
        self.time_pub_pose.publish(current_time)

    def save_delays_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/posed.csv"):
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)  # Ensure the directory exists
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Timestamp from Slave", "Pose Delay (s)"])
                for timestamp, delay in self.pose_delay_entries:
                    writer.writerow([timestamp, delay])
            rospy.loginfo(f"Pose delay log saved to: {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to write pose delays to CSV: {e}")

# new .csv file
    def save_pose_pub_timestamps_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_pub_timestamps.csv"):
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Pose Publish Timestamp (s)"])
                for ts in self.pose_pub_timestamps:
                    writer.writerow([ts])
            rospy.loginfo(f"Pose publish timestamps saved to: {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to write pose publish timestamps to CSV: {e}")

    def save_pose_gen_send_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_gen_send.csv"):
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Generation Time', 'Sent Time', 'Difference (s)'])
                for gen_time, sent_time, diff in self.pose_gen_send_log:
                    writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
            rospy.loginfo(f"Pose gen→send log saved to: {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to write pose gen→send log to CSV: {e}")



    def shutdown_hook(self):
        self.shutdown_flag = True
        zero_force_msg = OmniFeedback()
        zero_force_msg.force.x = 0
        zero_force_msg.force.y = 0
        zero_force_msg.force.z = 0

        rospy.loginfo("Sending zero force to haptic device...")
        for _ in range(10):
            self.force_pub.publish(zero_force_msg)
            time.sleep(0.1)
        rospy.loginfo("Shutdown complete.")

        # Save delays on shutdown
        self.save_delays_to_csv()
        self.save_pose_pub_timestamps_to_csv()
        self.save_pose_gen_send_to_csv()



#     def main_loop(self):
#         rospy.spin()

    def main_loop(self):
        rate = rospy.Rate(500)  # 500 Hz
        while not rospy.is_shutdown():
            if self.latest_pose_msg is not None:
                gen_time = self.latest_pose_msg.header.stamp.to_sec()

    # Sent time (when we actually publish to slave)
                sent_time = rospy.get_time()

    # Publish pose and timestamp
                self.pose_pub.publish(self.latest_pose_msg)
                self.time_pub_pose.publish(gen_time)

    # Store sent timestamp for separate CSV
                self.pose_pub_timestamps.append(sent_time)

    # Store (gen, sent, diff)
                diff = sent_time - gen_time
                self.pose_gen_send_log.append((gen_time, sent_time, diff))
            rate.sleep()

#     def main_loop(self):
#         desired_dt = 1.0 / 500.0  # 2 ms
#         next_time = rospy.get_time()

#         while not rospy.is_shutdown():
#             if self.latest_pose_msg is not None:
#                 current_time = rospy.get_time()
#                 self.pose_pub.publish(self.latest_pose_msg)
#                 self.time_pub_pose.publish(current_time)

#             next_time += desired_dt
#             sleep_duration = next_time - rospy.get_time()
#             if sleep_duration > 0:
#                 rospy.sleep(sleep_duration)
#             else:
#             # We are behind schedule, skip sleep to catch up
#                 next_time = rospy.get_time()



if __name__ == "__main__":
    try:
        controller = MasterController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass