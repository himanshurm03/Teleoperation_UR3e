#!/usr/bin/env python3
# import rospy
# import time
# from omni_msgs.msg import OmniFeedback
# from std_msgs.msg import Float64
# import csv
# import os

# class MasterController:
#     def __init__(self):
#         rospy.init_node('master', anonymous=True)

#         # Delay storage
#         self.pose_delays = []
#         self.pose_delay_entries = []
#         self.force_delays = []
#         self.force_delay_entries = []

#         # Publishers
#         self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
#         self.time_pub_pose = rospy.Publisher('time_from_master_to_slave_for_pose', Float64, queue_size=1)
#         self.time_pub_force = rospy.Publisher('time_from_master_to_slave_for_force', Float64, queue_size=1)
        
#         # Subscribers
#         rospy.Subscriber('force_from_slave_to_master', OmniFeedback, self.force_from_slave_to_master_callback)
#         rospy.Subscriber('time_from_slave_to_master_for_pose', Float64, self.time_from_slave_to_master_for_pose_callback)
#         rospy.Subscriber('time_from_slave_to_master_for_force', Float64, self.time_from_slave_to_master_for_force_callback)

#         # Shutdown flag
#         self.shutdown_flag = False
#         rospy.on_shutdown(self.shutdown_hook)

#     def time_from_slave_to_master_for_force_callback(self, msg: Float64):
#         delay = rospy.get_time() - msg.data  # round trip delay
#         print(f"Delay for force: {delay}")
#         self.force_delays.append(delay)
#         self.force_delay_entries.append((msg.data, delay))

#         # Send the same timestamp back to the slave
#         self.time_pub_force.publish(msg)

#     def time_from_slave_to_master_for_pose_callback(self, msg: Float64):
#         delay = rospy.get_time() - msg.data  # round trip delay
#         print(f"Delay for pose: {delay}")
#         self.pose_delays.append(delay)
#         self.pose_delay_entries.append((msg.data, delay))

#     def force_from_slave_to_master_callback(self, msg: OmniFeedback):
#         if not self.shutdown_flag:
#             self.force_pub.publish(msg)

#     def save_delays_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/posed.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Timestamp from Slave", "Pose Delay (s)"])
#                 for timestamp, delay in self.pose_delay_entries:
#                     writer.writerow([timestamp, delay])
#             rospy.loginfo(f"Pose delay log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose delays to CSV: {e}")

#     def shutdown_hook(self):
#         self.shutdown_flag = True
#         zero_force_msg = OmniFeedback()
#         zero_force_msg.force.x = 0
#         zero_force_msg.force.y = 0
#         zero_force_msg.force.z = 0

#         rospy.loginfo("Sending zero force to haptic device...")
#         for _ in range(10):
#             self.force_pub.publish(zero_force_msg)
#             time.sleep(0.1)
#         rospy.loginfo("Shutdown complete.")

#         self.save_delays_to_csv()

#     def main_loop(self):
#         rate = rospy.Rate(500)  # 500 Hz
#         while not rospy.is_shutdown():
#             # Just send timestamps, no pose packets
#             current_time = rospy.get_time()
#             self.time_pub_pose.publish(current_time)
#             rate.sleep()


# if __name__ == "__main__":
#     try:
#         controller = MasterController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass


#updated

import rospy
import time
from omni_msgs.msg import OmniFeedback
from std_msgs.msg import Float64
import csv
import os

class MasterController:
    def __init__(self):
        rospy.init_node('master', anonymous=True)

        # Delay storage
        self.pose_delays = []
        self.pose_delay_entries = []
        self.force_delays = []
        self.force_delay_entries = []
        self.last_publish_time_force = 0.0

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
        self.time_pub_pose = rospy.Publisher('time_from_master_to_slave_for_pose', Float64, queue_size=1)
        self.time_pub_force = rospy.Publisher('time_from_master_to_slave_for_force', Float64, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('force_from_slave_to_master', OmniFeedback, self.force_from_slave_to_master_callback)
        rospy.Subscriber('time_from_slave_to_master_for_pose', Float64, self.time_from_slave_to_master_for_pose_callback)
        rospy.Subscriber('time_from_slave_to_master_for_force', Float64, self.time_from_slave_to_master_for_force_callback)

        # Storage for last received force timestamp
        self.last_force_timestamp = None

        # Shutdown flag
        self.shutdown_flag = False
        rospy.on_shutdown(self.shutdown_hook)

    def time_from_slave_to_master_for_force_callback(self, msg: Float64):
        delay = rospy.get_time() - msg.data  # round trip delay
        print(f"Delay for force: {delay}")
        self.force_delays.append(delay)
        self.force_delay_entries.append((msg.data, delay))

        # Store the last received timestamp (will be echoed in main loop)
        #self.last_force_timestamp = msg
        now = rospy.get_time()
        if now - self.last_publish_time_force >= 1.0 / 500.0:  # 500 Hz
            self.time_pub_force.publish(msg)   # <-- same timestamp echoed back
            self.last_publish_time_force = now




    # def time_from_slave_to_master_for_force_callback(self, msg: Float64):
    #     delay = rospy.get_time() - msg.data
    #     print(f"Delay for force: {delay}")
    #     self.force_delays.append(delay)
    #     self.force_delay_entries.append((msg.data, delay))

    # # Echo it back immediately (1:1)
    #     self.time_pub_force.publish(msg)


    def time_from_slave_to_master_for_pose_callback(self, msg: Float64):
        delay = rospy.get_time() - msg.data  # round trip delay
        print(f"Delay for pose: {delay}")
        self.pose_delays.append(delay)
        self.pose_delay_entries.append((msg.data, delay))

    def force_from_slave_to_master_callback(self, msg: OmniFeedback):
        if not self.shutdown_flag:
            self.force_pub.publish(msg)

    def save_delays_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/posed.csv"):
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Timestamp from Slave", "Pose Delay (s)"])
                for timestamp, delay in self.pose_delay_entries:
                    writer.writerow([timestamp, delay])
            rospy.loginfo(f"Pose delay log saved to: {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to write pose delays to CSV: {e}")

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

        self.save_delays_to_csv()

    def main_loop(self):
        rate = rospy.Rate(500)  # 500 Hz
        while not rospy.is_shutdown():
            # Always publish master timestamp for pose
            current_time = rospy.get_time()
            self.time_pub_pose.publish(current_time)

            # Echo the last received slave force timestamp at 500 Hz
            if self.last_force_timestamp is not None:
                self.time_pub_force.publish(self.last_force_timestamp)

            rate.sleep()


if __name__ == "__main__":
    try:
        controller = MasterController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
