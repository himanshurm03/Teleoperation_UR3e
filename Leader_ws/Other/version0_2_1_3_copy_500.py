#!/usr/bin/env python3
# import rospy
# import time
# from omni_msgs.msg import OmniFeedback
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64 
# import csv
# import os

# class MasterController:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('master', anonymous=True)
        
#         # Delay storage
#         self.pose_delays = []
#         self.pose_delay_entries = []
#         self.force_delays = []
#         self.force_delay_entries = []
#         self.pose_pub_timestamps = []
#         self.pose_gen_send_log = []
#         self.force_receive_log = []
#         self.latest_pose_msg = None


#         # Publishers
#         self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
#         self.pose_pub = rospy.Publisher('pose_from_master_to_slave', PoseStamped, queue_size=1)
#         self.time_pub_pose = rospy.Publisher('time_from_master_to_slave_for_pose', Float64, queue_size=1)
#         self.time_pub_force = rospy.Publisher('time_from_master_to_slave_for_force', Float64, queue_size=1)
        
#         # Subscribers
#         rospy.Subscriber('force_from_slave_to_master', OmniFeedback, self.force_from_slave_to_master_callback)
#         #rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_from_master_to_slave_callback)
#         rospy.Subscriber('time_from_slave_to_master_for_pose', Float64, self.time_from_slave_to_master_for_pose_callback)
#         rospy.Subscriber('time_from_slave_to_master_for_force', Float64, self.time_from_slave_to_master_for_force_callback)
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_callback)


#         # Shutdown flag
#         self.shutdown_flag = False
#         rospy.on_shutdown(self.shutdown_hook)

#     # def time_from_slave_to_master_for_force_callback(self, msg: Float64):
#     #     delay = rospy.get_time() - msg.data
#     #     print(f"Delay for force: {delay}")
#     #     self.force_delays.append(delay)
#     #     self.force_delay_entries.append((msg.data, delay))


#     def time_from_slave_to_master_for_force_callback(self, msg: Float64):
#         delay = rospy.get_time() - msg.data
#         print(f"Delay for force: {delay}")
#         self.force_delays.append(delay)
#         self.force_delay_entries.append((msg.data, delay))

#         # Send the same timestamp back to the slave
#         self.time_pub_force.publish(msg)

#     # def pose_callback(self, msg: PoseStamped):
#     #     self.latest_pose_msg = msg
    
#     def pose_callback(self, msg: PoseStamped):
#     # Time when we receive the pose from Geomagic (local ROS clock)
#         gen_time = rospy.get_time()

#     # Keep a copy if you need it elsewhere
#         self.latest_pose_msg = msg

#     # Publish pose directly to the slave
#         self.pose_pub.publish(msg)

#     # Send the generation time to the slave for delay calculation
#         self.time_pub_pose.publish(gen_time)

#     # Time after sending (very close to gen_time)
#         sent_time = rospy.get_time()

#     # Log publish timestamp
#         self.pose_pub_timestamps.append(sent_time)

#     # Log (gen, sent, diff) using the same clock
#         diff = sent_time - gen_time
#         self.pose_gen_send_log.append((gen_time, sent_time, diff))



#     def time_from_slave_to_master_for_pose_callback(self, msg: Float64):
#         delay = rospy.get_time() - msg.data  #round trip delay
#         print(f"Delay for pose: {delay}")
#         self.pose_delays.append(delay)
#         self.pose_delay_entries.append((msg.data, delay)) 

#     # def force_from_slave_to_master_callback(self, msg: OmniFeedback):
#     #     if not self.shutdown_flag:
#     #         self.force_pub.publish(msg)
#     def force_from_slave_to_master_callback(self, msg: OmniFeedback):
#         if not self.shutdown_flag:
#             receive_time = rospy.get_time()  # when received from slave
#             self.force_pub.publish(msg)      # existing logic (send to haptic device)
#             applied_time = rospy.get_time()  # when applied to device
#             diff = applied_time - receive_time
#             self.force_receive_log.append((receive_time, applied_time, diff))

#     def pose_from_master_to_slave_callback(self, msg: PoseStamped):
#         current_time = rospy.get_time()
#         self.pose_pub.publish(msg)
#         self.time_pub_pose.publish(current_time)

#     def save_delays_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/posed.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)  # Ensure the directory exists
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Timestamp from Slave", "Pose Delay (s)"])
#                 for timestamp, delay in self.pose_delay_entries:
#                     writer.writerow([timestamp, delay])
#             rospy.loginfo(f"Pose delay log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose delays to CSV: {e}")

# # new .csv file
#     def save_pose_pub_timestamps_to_csv(self, filepath="/home/autonomous-lab/Desktop/camera/pose_pub_timestamps.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Pose Publish Timestamp (s)"])
#                 for ts in self.pose_pub_timestamps:
#                     writer.writerow([ts])
#             rospy.loginfo(f"Pose publish timestamps saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose publish timestamps to CSV: {e}")

#     def save_pose_gen_send_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_gen_send.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Generation Time', 'Sent Time', 'Difference (s)'])
#                 for gen_time, sent_time, diff in self.pose_gen_send_log:
#                     writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Pose gen→send log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose gen→send log to CSV: {e}")

#     def save_force_receive_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/force_processing.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Receive Time', 'Applied Time', 'Difference (s)'])
#                 for recv, applied, diff in self.force_receive_log:
#                     writer.writerow([f"{recv:.9f}", f"{applied:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Force receive log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write force receive log to CSV: {e}")



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

#         # Save delays on shutdown
#         self.save_delays_to_csv()
#         self.save_pose_pub_timestamps_to_csv()
#         self.save_pose_gen_send_to_csv()
#         self.save_force_receive_to_csv()


#     def main_loop(self):
#         rospy.spin()

# ##THIS ONE WAS TESTED
#     # def main_loop(self):
#     #     rate = rospy.Rate(1000)  # 500 Hz
#     #     while not rospy.is_shutdown():
#     #         if self.latest_pose_msg is not None:
#     #             gen_time = self.latest_pose_msg.header.stamp.to_sec()

#     # # Sent time (when we actually publish to slave)
#     #             sent_time = rospy.get_time()

#     # # Publish pose and timestamp
#     #             self.pose_pub.publish(self.latest_pose_msg)
#     #             self.time_pub_pose.publish(gen_time)

#     # # Store sent timestamp for separate CSV
#     #             self.pose_pub_timestamps.append(sent_time)

#     # # Store (gen, sent, diff)
#     #             diff = sent_time - gen_time
#     #             self.pose_gen_send_log.append((gen_time, sent_time, diff))
#     #         rate.sleep()

# #     def main_loop(self):
# #         desired_dt = 1.0 / 500.0  # 2 ms
# #         next_time = rospy.get_time()

# #         while not rospy.is_shutdown():
# #             if self.latest_pose_msg is not None:
# #                 current_time = rospy.get_time()
# #                 self.pose_pub.publish(self.latest_pose_msg)
# #                 self.time_pub_pose.publish(current_time)

# #             next_time += desired_dt
# #             sleep_duration = next_time - rospy.get_time()
# #             if sleep_duration > 0:
# #                 rospy.sleep(sleep_duration)
# #             else:
# #             # We are behind schedule, skip sleep to catch up
# #                 next_time = rospy.get_time()

# #THIS MAIN LOOP IS CAUSING ERROR(REASON STILL NOT KNOWN):
# # ROBOT MOVEMENTS ARE NOT SYNCHRONISED HERE(TESTED WITH CAMERA)
# #NEW MAIN LOOP
#     # def main_loop(self):
#     #     desired_rate = 1000.0  #500 Hz
#     #     desired_dt = 1.0 / desired_rate  # seconds (4 ms)
#     #     next_time = rospy.get_time()

#     #     while not rospy.is_shutdown():
#     #         current_time = rospy.get_time()

#     #         if self.latest_pose_msg is not None:
#     #             gen_time = self.latest_pose_msg.header.stamp.to_sec()
#     #             sent_time = current_time

#     #         # Publish the pose and timestamp
#     #             self.pose_pub.publish(self.latest_pose_msg)
#     #             self.time_pub_pose.publish(gen_time)

#     #         # Log for CSV output
#     #             self.pose_pub_timestamps.append(sent_time)
#     #             diff = sent_time - gen_time
#     #             self.pose_gen_send_log.append((gen_time, sent_time, diff))

#     #     # Enforce 500 Hz timing
#     #         next_time += desired_dt
#     #         sleep_duration = next_time - rospy.get_time()
#     #         if sleep_duration > 0:
#     #             rospy.sleep(sleep_duration)
#     #         else:
#     #         # If behind schedule, skip sleep to catch up
#     #             next_time = rospy.get_time()


# if __name__ == "__main__":
#     try:
#         controller = MasterController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass


#This code is working perfectly file.
#all the data rates are same(outgoing topics going towards the Slave side)
#one issue here is pose_gen_send is higher here

# import rospy
# import time
# from omni_msgs.msg import OmniFeedback
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64 
# import csv
# import os

# class MasterController:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('master', anonymous=True)
        
#         # Delay storage
#         self.pose_delays = []
#         self.pose_delay_entries = []
#         self.force_delays = []
#         self.force_delay_entries = []
#         self.pose_pub_timestamps = []
#         self.pose_gen_send_log = []
#         self.force_receive_log = []
#         self.latest_pose_msg = None
#         self.latest_force_msg = None
#         self.latest_force_timestamp_from_slave = None
#         #self.latest_pose_msg = None
#         self.latest_pose_recv_time = None


#         # Publishers
#         self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
#         self.pose_pub = rospy.Publisher('pose_from_master_to_slave', PoseStamped, queue_size=1)
#         self.time_pub_pose = rospy.Publisher('time_from_master_to_slave_for_pose', Float64, queue_size=1)
#         self.time_pub_force = rospy.Publisher('time_from_master_to_slave_for_force', Float64, queue_size=1)
        
#         # Subscribers
#         rospy.Subscriber('force_from_slave_to_master', OmniFeedback, self.force_from_slave_to_master_callback)
#         #rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_from_master_to_slave_callback)
#         rospy.Subscriber('time_from_slave_to_master_for_pose', Float64, self.time_from_slave_to_master_for_pose_callback)
#         rospy.Subscriber('time_from_slave_to_master_for_force', Float64, self.time_from_slave_to_master_for_force_callback)
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_callback)


#         # Shutdown flag
#         self.shutdown_flag = False
#         rospy.on_shutdown(self.shutdown_hook)

#     # def time_from_slave_to_master_for_force_callback(self, msg: Float64):
#     #     delay = rospy.get_time() - msg.data
#     #     print(f"Delay for force: {delay}")
#     #     self.force_delays.append(delay)
#     #     self.force_delay_entries.append((msg.data, delay))


#     def time_from_slave_to_master_for_force_callback(self, msg: Float64):
#         delay = rospy.get_time() - msg.data
#         print(f"Delay for force: {delay}")
#         self.force_delays.append(delay)
#         self.force_delay_entries.append((msg.data, delay))

#     # Store the latest timestamp from slave,
#     # we'll send it back at 500 Hz from main_loop
#         self.latest_force_timestamp_from_slave = msg.data

#     # def pose_callback(self, msg: PoseStamped):
#     #     self.latest_pose_msg = msg
    
#     def pose_callback(self, msg: PoseStamped):
#         self.latest_pose_msg = msg
#         self.latest_pose_recv_time = rospy.get_time()


#     def time_from_slave_to_master_for_pose_callback(self, msg: Float64):
#         delay = rospy.get_time() - msg.data  #round trip delay
#         print(f"Delay for pose: {delay}")
#         self.pose_delays.append(delay)
#         self.pose_delay_entries.append((msg.data, delay)) 

#     # def force_from_slave_to_master_callback(self, msg: OmniFeedback):
#     #     if not self.shutdown_flag:
#     #         self.force_pub.publish(msg)
#     def force_from_slave_to_master_callback(self, msg: OmniFeedback):
#         if not self.shutdown_flag:
#             self.latest_force_msg = msg

#         # Optionally: record receive time (for processing log)
#             receive_time = rospy.get_time()
#         # We will treat "applied time" as when we actually publish to device in main_loop,
#         # or you can just store receive_time for now.
#             self.force_receive_log.append((receive_time, receive_time, 0.0))

#     def pose_from_master_to_slave_callback(self, msg: PoseStamped):
#         current_time = rospy.get_time()
#         self.pose_pub.publish(msg)
#         self.time_pub_pose.publish(current_time)

#     def save_delays_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/posed.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)  # Ensure the directory exists
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Timestamp from Slave", "Pose Delay (s)"])
#                 for timestamp, delay in self.pose_delay_entries:
#                     writer.writerow([timestamp, delay])
#             rospy.loginfo(f"Pose delay log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose delays to CSV: {e}")

# # new .csv file
#     def save_pose_pub_timestamps_to_csv(self, filepath="/home/autonomous-lab/Desktop/camera/pose_pub_timestamps.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Pose Publish Timestamp (s)"])
#                 for ts in self.pose_pub_timestamps:
#                     writer.writerow([ts])
#             rospy.loginfo(f"Pose publish timestamps saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose publish timestamps to CSV: {e}")

#     def save_pose_gen_send_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_gen_send.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Generation Time', 'Sent Time', 'Difference (s)'])
#                 for gen_time, sent_time, diff in self.pose_gen_send_log:
#                     writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Pose gen→send log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose gen→send log to CSV: {e}")

#     def save_force_receive_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/force_processing.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Receive Time', 'Applied Time', 'Difference (s)'])
#                 for recv, applied, diff in self.force_receive_log:
#                     writer.writerow([f"{recv:.9f}", f"{applied:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Force receive log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write force receive log to CSV: {e}")



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

#         # Save delays on shutdown
#         self.save_delays_to_csv()
#         self.save_pose_pub_timestamps_to_csv()
#         self.save_pose_gen_send_to_csv()
#         self.save_force_receive_to_csv()


#     def main_loop(self):
#         rate = rospy.Rate(250)

#         while not rospy.is_shutdown():
#         # ---- POSE PATH (to slave) ----
#             if self.latest_pose_msg is not None and self.latest_pose_recv_time is not None:
#             # Time when pose was generated at Geomagic
#                 gen_time = self.latest_pose_recv_time

#             # Time when we actually send it to the slave
#                 sent_time = rospy.get_time()

#             # Pose to slave at fixed 500 Hz
#                 self.pose_pub.publish(self.latest_pose_msg)

#             # Generation time for delay calculations
#                 self.time_pub_pose.publish(gen_time)

#             # Logging
#                 self.pose_pub_timestamps.append(sent_time)
#                 diff = sent_time - gen_time
#                 self.pose_gen_send_log.append((gen_time, sent_time, diff))

#         # ---- FORCE PATH (to haptic + force timestamps) ----
#             if not self.shutdown_flag and self.latest_force_msg is not None:
#             # Send latest force to haptic device at 500 Hz
#                 self.force_pub.publish(self.latest_force_msg)

#             if self.latest_force_timestamp_from_slave is not None:
#             # Echo back the latest timestamp from slave at 500 Hz
#                t_msg = Float64()
#                t_msg.data = self.latest_force_timestamp_from_slave
#                self.time_pub_force.publish(t_msg)

#             rate.sleep()


# if __name__ == "__main__":
#     try:
#         controller = MasterController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass

#3
# changes in the downsampling logic
#This is working perfectly

# import rospy
# import time
# from omni_msgs.msg import OmniFeedback
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64 
# import csv
# import os

# class MasterController:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('master', anonymous=True)
        
        # # Sampling configuration
        # self.HAPTIC_DEVICE_RATE = 1000  # Hz - haptic device native rate
        # self.DESIRED_SAMPLING_RATE = 250  # Hz - target rate (CHANGE THIS VALUE)
        # self.DOWNSAMPLE_FACTOR = self.HAPTIC_DEVICE_RATE // self.DESIRED_SAMPLING_RATE  # = 2
        # self.sample_counter = 0
        
#         # Delay storage
#         self.pose_delays = []
#         self.pose_delay_entries = []
#         self.force_delays = []
#         self.force_delay_entries = []
#         self.pose_pub_timestamps = []
#         self.pose_gen_send_log = []
#         self.force_receive_log = []
#         self.latest_pose_msg = None

#         # Publishers
#         self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
#         self.pose_pub = rospy.Publisher('pose_from_master_to_slave', PoseStamped, queue_size=1)
#         self.time_pub_pose = rospy.Publisher('time_from_master_to_slave_for_pose', Float64, queue_size=1)
#         self.time_pub_force = rospy.Publisher('time_from_master_to_slave_for_force', Float64, queue_size=1)
        
#         # Subscribers
#         rospy.Subscriber('force_from_slave_to_master', OmniFeedback, self.force_from_slave_to_master_callback)
#         rospy.Subscriber('time_from_slave_to_master_for_pose', Float64, self.time_from_slave_to_master_for_pose_callback)
#         rospy.Subscriber('time_from_slave_to_master_for_force', Float64, self.time_from_slave_to_master_for_force_callback)
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_callback)

#         # Shutdown flag
#         self.shutdown_flag = False
#         rospy.on_shutdown(self.shutdown_hook)
        
#         rospy.loginfo(f"Master Controller initialized with {self.DESIRED_SAMPLING_RATE} Hz sampling rate (downsampling factor: {self.DOWNSAMPLE_FACTOR})")

#     def time_from_slave_to_master_for_force_callback(self, msg: Float64):
#         delay = rospy.get_time() - msg.data
#         print(f"Delay for force: {delay}")
#         self.force_delays.append(delay)
#         self.force_delay_entries.append((msg.data, delay))
#         # Send the same timestamp back to the slave
#         self.time_pub_force.publish(msg)

#     def pose_callback(self, msg: PoseStamped):
#         # """
#         # Callback for pose messages from haptic device.
#         # Implements downsampling by counter: only processes every Nth sample.
#         # """
#         # Increment counter
#         self.sample_counter += 1
        
#         # Only process every DOWNSAMPLE_FACTOR sample (e.g., every 4th sample)
#         if self.sample_counter >= self.DOWNSAMPLE_FACTOR:
#             self.latest_pose_msg = msg
#             self.sample_counter = 0  # Reset counter
            
#             # Immediately publish the selected sample
#             gen_time = msg.header.stamp.to_sec()
#             sent_time = rospy.get_time()
            
#             # Publish pose and timestamp
#             self.pose_pub.publish(msg)
#             self.time_pub_pose.publish(gen_time)
            
#             # Log the data
#             self.pose_pub_timestamps.append(sent_time)
#             diff = sent_time - gen_time
#             self.pose_gen_send_log.append((gen_time, sent_time, diff))

#     def time_from_slave_to_master_for_pose_callback(self, msg: Float64):
#         delay = rospy.get_time() - msg.data  # round trip delay
#         print(f"Delay for pose: {delay}")
#         self.pose_delays.append(delay)
#         self.pose_delay_entries.append((msg.data, delay)) 

#     def force_from_slave_to_master_callback(self, msg: OmniFeedback):
#         if not self.shutdown_flag:
#             receive_time = rospy.get_time()  # when received from slave
#             self.force_pub.publish(msg)      # send to haptic device
#             applied_time = rospy.get_time()  # when applied to device
#             diff = applied_time - receive_time
#             self.force_receive_log.append((receive_time, applied_time, diff))

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

#     def save_pose_pub_timestamps_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_pub_timestamps.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Pose Publish Timestamp (s)"])
#                 for ts in self.pose_pub_timestamps:
#                     writer.writerow([ts])
#             rospy.loginfo(f"Pose publish timestamps saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose publish timestamps to CSV: {e}")

#     def save_pose_gen_send_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_gen_send.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Generation Time', 'Sent Time', 'Difference (s)'])
#                 for gen_time, sent_time, diff in self.pose_gen_send_log:
#                     writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Pose gen→send log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose gen→send log to CSV: {e}")

#     def save_force_receive_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/force_processing.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Receive Time', 'Applied Time', 'Difference (s)'])
#                 for recv, applied, diff in self.force_receive_log:
#                     writer.writerow([f"{recv:.9f}", f"{applied:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Force receive log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write force receive log to CSV: {e}")

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

#         # Save delays on shutdown
#         self.save_delays_to_csv()
#         self.save_pose_pub_timestamps_to_csv()
#         self.save_pose_gen_send_to_csv()
#         self.save_force_receive_to_csv()

#     def main_loop(self):
#         # """
#         # Main loop - now just spins to keep node alive.
#         # All pose processing happens in the callback with downsampling.
#         # """
#         rospy.spin()


# if __name__ == "__main__":
#     try:
#         controller = MasterController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass

#4.updated code
#message
#This code is also working perfectly 
# import rospy
# import time
# from omni_msgs.msg import OmniFeedback
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64 
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData  # CHANGE 'your_package_name'
# import csv
# import os


# class MasterController:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('master', anonymous=True)
        
#         # Sampling configuration - read from launch file parameters
#         self.HAPTIC_DEVICE_RATE = rospy.get_param('~haptic_device_rate', 1000)  # Hz - default 1000
#         self.DESIRED_SAMPLING_RATE = rospy.get_param('~sampling_rate', 500)  # Hz - default 500
#         self.DOWNSAMPLE_FACTOR = self.HAPTIC_DEVICE_RATE // self.DESIRED_SAMPLING_RATE
#         self.sample_counter = 0
        
#         # Delay storage
#         self.pose_delays = []
#         self.pose_delay_entries = []
#         self.force_delays = []
#         self.force_delay_entries = []
#         self.pose_pub_timestamps = []
#         self.pose_gen_send_log = []
#         self.force_receive_log = []
#         self.latest_pose_msg = None
#         #self.latest_force_timestamp = 0.0  # Store latest force timestamp for bundling
#         self.latest_force_timestamp = rospy.get_time() 

#         # Publishers
#         self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)  #Sending to the haptic device
#         # Single publisher for all data to slave
#         self.master_to_slave_pub = rospy.Publisher('master_to_slave_data', MasterToSlaveData, queue_size=1)
        
#         # Subscribers
#         # NEW: Subscribe to bundled data from slave (replaces 3 separate subscriptions)
#         rospy.Subscriber('slave_to_master_data', SlaveToMasterData, self.slave_to_master_callback)
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_callback)  #Receiving to the haptic device(internally)

#         # Shutdown flag
#         self.shutdown_flag = False
#         rospy.on_shutdown(self.shutdown_hook)
        
#         rospy.loginfo(f"Master Controller initialized with {self.DESIRED_SAMPLING_RATE} Hz sampling rate (downsampling factor: {self.DOWNSAMPLE_FACTOR})")
#         rospy.loginfo("Using bundled messaging (MasterToSlaveData <-> SlaveToMasterData)")

#     def slave_to_master_callback(self, msg: SlaveToMasterData):
#         """
#         Receives bundled data from slave containing:
#         - Force feedback (x, y, z)
#         - Pose timestamp (echo)
#         - Force timestamp
#         """
#         if self.shutdown_flag:
#             return
        
#         # 1. Extract and publish force to haptic device
#         receive_time = rospy.get_time()
        
#         force_msg = OmniFeedback()
#         force_msg.force.x = msg.force.x
#         force_msg.force.y = msg.force.y
#         force_msg.force.z = msg.force.z
        
#         self.force_pub.publish(force_msg)
        
#         applied_time = rospy.get_time()
#         diff = applied_time - receive_time
#         self.force_receive_log.append((receive_time, applied_time, diff))
        
#         # 2. Calculate pose RTT (Round Trip Time)
#         pose_timestamp = msg.pose_timestamp
#         if pose_timestamp > 0:  # Valid timestamp
#             current_time = rospy.get_time()
#             pose_delay = current_time - pose_timestamp
#             print(f"Delay for pose: {pose_delay}")
#             self.pose_delays.append(pose_delay)
#             self.pose_delay_entries.append((pose_timestamp, pose_delay))
        
#         # 3. Calculate force RTT
#         force_timestamp = msg.force_timestamp
#         if force_timestamp > 0:  # Valid timestamp
#             current_time = rospy.get_time()
#             force_delay = current_time - force_timestamp
#             print(f"Delay for force: {force_delay}")
#             self.force_delays.append(force_delay)
#             self.force_delay_entries.append((force_timestamp, force_delay))
            
#             # Store for bundling with next pose
#             self.latest_force_timestamp = force_timestamp

#     def pose_callback(self, msg: PoseStamped):
#         """
#         Callback for pose messages from haptic device.
#         Implements downsampling by counter: only processes every Nth sample.
#         """
#         # Increment counter
#         self.sample_counter += 1
        
#         # Only process every DOWNSAMPLE_FACTOR sample (e.g., every 2nd sample for 500Hz)
#         if self.sample_counter >= self.DOWNSAMPLE_FACTOR:
#             self.latest_pose_msg = msg
#             self.sample_counter = 0  # Reset counter
            
#             # Create bundled message with ALL data
#             bundled_msg = MasterToSlaveData()
            
#             # 1. Add pose timestamp in header
#             bundled_msg.header = msg.header  # Contains pose generation timestamp
            
#             # 2. Add pose data (position + orientation = 7 values)
#             bundled_msg.pose = msg.pose
            
#             # 3. Add force timestamp
#             bundled_msg.force_timestamp = self.latest_force_timestamp
            
#             # Publish the bundled message
#             gen_time = msg.header.stamp.to_sec()
#             sent_time = rospy.get_time()
            
#             self.master_to_slave_pub.publish(bundled_msg)
            
#             # Log the data
#             self.pose_pub_timestamps.append(sent_time)
#             diff = sent_time - gen_time
#             self.pose_gen_send_log.append((gen_time, sent_time, diff))
            
#             rospy.loginfo_throttle(1.0, f"Sent bundled message: pose_time={gen_time:.3f}, force_time={self.latest_force_timestamp:.3f}")

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

#     def save_force_delays_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/forced.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Timestamp from Slave", "Force Delay (s)"])
#                 for timestamp, delay in self.force_delay_entries:
#                     writer.writerow([timestamp, delay])
#             rospy.loginfo(f"Force delay log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write force delays to CSV: {e}")

#     def save_pose_pub_timestamps_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_pub_timestamps.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Pose Publish Timestamp (s)"])
#                 for ts in self.pose_pub_timestamps:
#                     writer.writerow([ts])
#             rospy.loginfo(f"Pose publish timestamps saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose publish timestamps to CSV: {e}")

#     def save_pose_gen_send_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_gen_send.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Generation Time', 'Sent Time', 'Difference (s)'])
#                 for gen_time, sent_time, diff in self.pose_gen_send_log:
#                     writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Pose gen→send log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose gen→send log to CSV: {e}")

#     def save_force_receive_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/force_processing.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Receive Time', 'Applied Time', 'Difference (s)'])
#                 for recv, applied, diff in self.force_receive_log:
#                     writer.writerow([f"{recv:.9f}", f"{applied:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Force receive log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write force receive log to CSV: {e}")

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

#         # Save delays on shutdown
#         self.save_delays_to_csv()
#         self.save_force_delays_to_csv()
#         self.save_pose_pub_timestamps_to_csv()
#         self.save_pose_gen_send_to_csv()
#         self.save_force_receive_to_csv()

#     def main_loop(self):
#         """
#         Main loop - now just spins to keep node alive.
#         All pose processing happens in the callback with downsampling.
#         """
#         rospy.spin()


# if __name__ == "__main__":
#     try:
#         controller = MasterController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass


#5.
#Message Debug 
#Removed forced.csv
#This code is also working fine

# import rospy
# import time
# from omni_msgs.msg import OmniFeedback
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64 
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# import csv
# import os


# class MasterController:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('master', anonymous=True)
        
#         # Sampling configuration - read from launch file parameters
#         self.HAPTIC_DEVICE_RATE = rospy.get_param('~haptic_device_rate', 1000)  # Hz - default 1000
#         self.DESIRED_SAMPLING_RATE = rospy.get_param('~sampling_rate', 100)  # Hz - default 500
#         self.DOWNSAMPLE_FACTOR = self.HAPTIC_DEVICE_RATE // self.DESIRED_SAMPLING_RATE
#         self.sample_counter = 0
        
#         # Delay storage
#         self.pose_delays = []
#         self.pose_delay_entries = []
#         self.force_delays = []
#         self.force_delay_entries = []
#         self.pose_pub_timestamps = []
#         self.pose_gen_send_log = []
#         self.force_receive_log = []
#         self.latest_pose_msg = None
#         self.latest_force_timestamp = rospy.get_time() 

#         # Publishers
#         self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)  # Sending to the haptic device
#         # Single publisher for all data to slave
#         self.master_to_slave_pub = rospy.Publisher('master_to_slave_data', MasterToSlaveData, queue_size=1)
        
#         # Subscribers
#         # Subscribe to bundled data from slave
#         rospy.Subscriber('slave_to_master_data', SlaveToMasterData, self.slave_to_master_callback)
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_callback)  # Receiving from the haptic device(internally)

#         # Shutdown flag
#         self.shutdown_flag = False
#         rospy.on_shutdown(self.shutdown_hook)
        
#         rospy.loginfo(f"Master Controller initialized with {self.DESIRED_SAMPLING_RATE} Hz sampling rate (downsampling factor: {self.DOWNSAMPLE_FACTOR})")
#         rospy.loginfo("Using bundled messaging (MasterToSlaveData <-> SlaveToMasterData)")

#     def slave_to_master_callback(self, msg: SlaveToMasterData):
#         """
#         Receives bundled data from slave containing:
#         - Force feedback (x, y, z)
#         - Pose timestamp (echo)
#         - Force timestamp
#         """
#         if self.shutdown_flag:
#             return
        
#         # 1. Extract and publish force to haptic device
#         receive_time = rospy.get_time()
        
#         force_msg = OmniFeedback()
#         force_msg.force.x = msg.force.x
#         force_msg.force.y = msg.force.y
#         force_msg.force.z = msg.force.z
        
#         self.force_pub.publish(force_msg)
        
#         applied_time = rospy.get_time()
#         diff = applied_time - receive_time
#         self.force_receive_log.append((receive_time, applied_time, diff))
        
#         # 2. Calculate pose RTD (Round Trip Delay)
#         pose_timestamp = msg.pose_timestamp
#         if pose_timestamp > 0:  # Valid timestamp
#             current_time = rospy.get_time()
#             pose_delay = current_time - pose_timestamp
#             print(f"Delay for pose: {pose_delay}")
#             self.pose_delays.append(pose_delay)
#             self.pose_delay_entries.append((pose_timestamp, pose_delay))
        
#         # 3. Store force timestamp (still track it, just don't save to CSV)
#         force_timestamp = msg.force_timestamp
#         if force_timestamp > 0:  # Valid timestamp
#             current_time = rospy.get_time()
#             force_delay = current_time - force_timestamp
#             print(f"Delay for force: {force_delay}")
#             self.force_delays.append(force_delay)
#             # REMOVED: self.force_delay_entries.append() - don't store for CSV
            
#             # Store for bundling with next pose
#             self.latest_force_timestamp = force_timestamp

#     def pose_callback(self, msg: PoseStamped):
#         """
#         Callback for pose messages from haptic device.
#         Implements downsampling by counter: only processes every Nth sample.
#         """
#         # Increment counter
#         self.sample_counter += 1
        
#         # Only process every DOWNSAMPLE_FACTOR sample (e.g., every 2nd sample for 500Hz)
#         if self.sample_counter >= self.DOWNSAMPLE_FACTOR:
#             self.latest_pose_msg = msg
#             self.sample_counter = 0  # Reset counter
            
#             # Create bundled message with ALL data
#             bundled_msg = MasterToSlaveData()
            
#             # 1. Add pose timestamp in header
#             bundled_msg.header = msg.header  # Contains pose generation timestamp
            
#             # 2. Add pose data (position + orientation = 7 values)
#             bundled_msg.pose = msg.pose
            
#             # 3. Add force timestamp
#             bundled_msg.force_timestamp = self.latest_force_timestamp
            
#             # Publish the bundled message
#             gen_time = msg.header.stamp.to_sec()
#             sent_time = rospy.get_time()
            
#             self.master_to_slave_pub.publish(bundled_msg)
            
#             # Log the data
#             self.pose_pub_timestamps.append(sent_time)
#             diff = sent_time - gen_time
#             self.pose_gen_send_log.append((gen_time, sent_time, diff))
            
#             rospy.loginfo_throttle(1.0, f"Sent bundled message: pose_time={gen_time:.3f}, force_time={self.latest_force_timestamp:.3f}")

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

#     # REMOVED: save_force_delays_to_csv() function - no longer needed

#     def save_pose_pub_timestamps_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_pub_timestamps.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Pose Publish Timestamp (s)"])
#                 for ts in self.pose_pub_timestamps:
#                     writer.writerow([ts])
#             rospy.loginfo(f"Pose publish timestamps saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose publish timestamps to CSV: {e}")

#     def save_pose_gen_send_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_gen_send.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Generation Time', 'Sent Time', 'Difference (s)'])
#                 for gen_time, sent_time, diff in self.pose_gen_send_log:
#                     writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Pose gen→send log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose gen→send log to CSV: {e}")

#     def save_force_receive_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/force_processing.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Receive Time', 'Applied Time', 'Difference (s)'])
#                 for recv, applied, diff in self.force_receive_log:
#                     writer.writerow([f"{recv:.9f}", f"{applied:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Force receive log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write force receive log to CSV: {e}")

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

#         # Save delays on shutdown
#         self.save_delays_to_csv()
#         # REMOVED: self.save_force_delays_to_csv() - no forced.csv
#         self.save_pose_pub_timestamps_to_csv()
#         self.save_pose_gen_send_to_csv()
#         self.save_force_receive_to_csv()

#     def main_loop(self):
#         """
#         Main loop - now just spins to keep node alive.
#         All pose processing happens in the callback with downsampling.
#         i.e. in the pose callback.
#         """
#         rospy.spin()


# if __name__ == "__main__":
#     try:
#         controller = MasterController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass

#Potentially Improved Code

# import rospy
# import time
# from omni_msgs.msg import OmniFeedback
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64 
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# import csv
# import os


# class MasterController:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('master', anonymous=True)
        
#         # Sampling configuration - read from launch file parameters
#         self.HAPTIC_DEVICE_RATE = rospy.get_param('~haptic_device_rate', 1000)  # Hz - default 1000
#         self.DESIRED_SAMPLING_RATE = rospy.get_param('~sampling_rate', 250)  # Hz - default 500
#         self.DOWNSAMPLE_FACTOR = self.HAPTIC_DEVICE_RATE // self.DESIRED_SAMPLING_RATE
#         self.sample_counter = 0
        
#         # Delay storage
#         self.pose_delays = []
#         self.pose_delay_entries = []
#         self.force_delays = []
#         self.force_delay_entries = []
#         self.pose_pub_timestamps = []
#         self.pose_gen_send_log = []
#         self.force_receive_log = []
#         self.latest_pose_msg = None
#         self.latest_force_timestamp = 0.0  # Initialize to 0
        
#         # ADDED: Track latest force data
#         self.latest_force_data = None  # Store latest forceVector3

#         # Publishers
#         self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
#         self.master_to_slave_pub = rospy.Publisher('master_to_slave_data', MasterToSlaveData, queue_size=1)
        
#         # Subscribers
#         rospy.Subscriber('slave_to_master_data', SlaveToMasterData, self.slave_to_master_callback)
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_callback)

#         # Shutdown flag
#         self.shutdown_flag = False
#         rospy.on_shutdown(self.shutdown_hook)
        
#         rospy.loginfo(f"Master Controller initialized with {self.DESIRED_SAMPLING_RATE} Hz sampling rate (downsampling factor: {self.DOWNSAMPLE_FACTOR})")
#         rospy.loginfo("Using bundled messaging (MasterToSlaveData <-> SlaveToMasterData)")

#     def slave_to_master_callback(self, msg: SlaveToMasterData):
#         """
#         Receives bundled data from slave containing:
#         - Force feedback (x, y, z)
#         - Pose timestamp (echo)
#         - Force timestamp
#         """
#         if self.shutdown_flag:
#             return
        
#         # 1. Extract and publish force to haptic device
#         receive_time = rospy.get_time()
        
#         force_msg = OmniFeedback()
#         force_msg.force.x = msg.force.x
#         force_msg.force.y = msg.force.y
#         force_msg.force.z = msg.force.z
        
#         self.force_pub.publish(force_msg)
        
#         applied_time = rospy.get_time()
#         diff = applied_time - receive_time
#         self.force_receive_log.append((receive_time, applied_time, diff))
        
#         # ADDED: Store latest force data (not used for bundling, just for local application)
#         self.latest_force_data = msg.force
        
#         # 2. Calculate pose RTD (Round Trip Delay)
#         pose_timestamp = msg.pose_timestamp
#         if pose_timestamp > 0:  # Valid timestamp
#             current_time = rospy.get_time()
#             pose_delay = current_time - pose_timestamp
#             print(f"Delay for pose: {pose_delay}")
#             self.pose_delays.append(pose_delay)
#             self.pose_delay_entries.append((pose_timestamp, pose_delay))
        
#         # 3. Echo back force timestamp for slave's RTD calculation
#         force_timestamp = msg.force_timestamp
#         if force_timestamp > 0:  # Valid timestamp
#             # Just store it - don't calculate delay here
#             # The slave will calculate its own RTD when it receives the echo
#             self.latest_force_timestamp = force_timestamp
            
#             rospy.loginfo_throttle(2.0, f"Received force_timestamp to echo: {force_timestamp:.6f}")

#     def pose_callback(self, msg: PoseStamped):
#         """
#         Callback for pose messages from haptic device.
#         Implements downsampling by counter: only processes every Nth sample.
#         Bundles pose + force_timestamp and sends together.
#         """
#         # Increment counter
#         self.sample_counter += 1
        
#         # Only process every DOWNSAMPLE_FACTOR sample (e.g., every 2nd sample for 500Hz)
#         if self.sample_counter >= self.DOWNSAMPLE_FACTOR:
#             self.latest_pose_msg = msg
#             self.sample_counter = 0  # Reset counter
            
#             # Create bundled message with ALL data
#             bundled_msg = MasterToSlaveData()
            
#             # 1. Add pose timestamp in header
#             bundled_msg.header = msg.header  # Contains pose generation timestamp
            
#             # 2. Add pose data (position + orientation = 7 values)
#             bundled_msg.pose = msg.pose
            
#             # 3. Add force timestamp (echo back the latest received from slave)
#             # This ensures every pose packet carries the most recent force_timestamp
#             bundled_msg.force_timestamp = self.latest_force_timestamp
            
#             # Publish the bundled message
#             gen_time = msg.header.stamp.to_sec()
#             sent_time = rospy.get_time()
            
#             self.master_to_slave_pub.publish(bundled_msg)
            
#             # Log the data
#             self.pose_pub_timestamps.append(sent_time)
#             diff = sent_time - gen_time
#             self.pose_gen_send_log.append((gen_time, sent_time, diff))
            
#             rospy.loginfo_throttle(1.0, 
#                 f"Sent bundled @ {self.DESIRED_SAMPLING_RATE}Hz: "
#                 f"pose_time={gen_time:.6f}, force_ts_echo={self.latest_force_timestamp:.6f}")

#     def save_delays_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/posed.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Timestamp from Slave", "Pose Delay (s)"])
#                 for timestamp, delay in self.pose_delay_entries:
#                     writer.writerow([f"{timestamp:.9f}", f"{delay:.9f}"])
#             rospy.loginfo(f"Pose delay log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose delays to CSV: {e}")

#     def save_pose_pub_timestamps_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_pub_timestamps.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(["Pose Publish Timestamp (s)"])
#                 for ts in self.pose_pub_timestamps:
#                     writer.writerow([f"{ts:.9f}"])
#             rospy.loginfo(f"Pose publish timestamps saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose publish timestamps to CSV: {e}")

#     def save_pose_gen_send_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_gen_send.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Generation Time', 'Sent Time', 'Difference (s)'])
#                 for gen_time, sent_time, diff in self.pose_gen_send_log:
#                     writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Pose gen→send log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose gen→send log to CSV: {e}")

#     def save_force_receive_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/force_processing.csv"):
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow(['Receive Time', 'Applied Time', 'Difference (s)'])
#                 for recv, applied, diff in self.force_receive_log:
#                     writer.writerow([f"{recv:.9f}", f"{applied:.9f}", f"{diff:.9f}"])
#             rospy.loginfo(f"Force receive log saved to: {filepath}")
#         except Exception as e:
#             rospy.logerr(f"Failed to write force receive log to CSV: {e}")

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

#         # Save delays on shutdown
#         self.save_delays_to_csv()
#         self.save_pose_pub_timestamps_to_csv()
#         self.save_pose_gen_send_to_csv()
#         self.save_force_receive_to_csv()

#     def main_loop(self):
#         """
#         Main loop - now just spins to keep node alive.
#         All pose processing happens in the callback with downsampling.
#         """
#         rospy.spin()


# if __name__ == "__main__":
#     try:
#         controller = MasterController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass

#The above code is perfect
#Purpose:Log the time differnce of force timestamp waiting time

import rospy
import time
from omni_msgs.msg import OmniFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64 
from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
import csv
import os


class MasterController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('master', anonymous=True)
        
        # Sampling configuration - read from launch file parameters
        self.HAPTIC_DEVICE_RATE = rospy.get_param('~haptic_device_rate', 1000)  # Hz - default 1000
        self.DESIRED_SAMPLING_RATE = rospy.get_param('~sampling_rate', 500)  # Hz - default 500
        self.DOWNSAMPLE_FACTOR = self.HAPTIC_DEVICE_RATE // self.DESIRED_SAMPLING_RATE
        self.sample_counter = 0
        
        # Delay storage
        self.pose_delays = []
        self.pose_delay_entries = []
        self.force_delays = []
        self.force_delay_entries = []
        self.pose_pub_timestamps = []
        self.pose_gen_send_log = []
        self.force_receive_log = []
        self.latest_pose_msg = None
        self.latest_force_timestamp = 0.0  # Initialize to 0
        
        # ADDED: Track latest force data
        self.latest_force_data = None  # Store latest forceVector3

        # FIXED: Track force timestamp waiting time with dictionary
        self.force_timestamp_first_receive = {}  # Maps force_ts -> first receive time
        self.force_timestamp_wait_log = []  # Stores (force_ts, receive_time, send_time, wait_time)

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
        self.master_to_slave_pub = rospy.Publisher('master_to_slave_data', MasterToSlaveData, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('slave_to_master_data', SlaveToMasterData, self.slave_to_master_callback)
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_callback)

        # Shutdown flag
        self.shutdown_flag = False
        rospy.on_shutdown(self.shutdown_hook)
        
        rospy.loginfo(f"Master Controller initialized with {self.DESIRED_SAMPLING_RATE} Hz sampling rate (downsampling factor: {self.DOWNSAMPLE_FACTOR})")
        rospy.loginfo("Using bundled messaging (MasterToSlaveData <-> SlaveToMasterData)")

    def slave_to_master_callback(self, msg: SlaveToMasterData):
        """
        Receives bundled data from slave containing:
        - Force feedback (x, y, z)
        - Pose timestamp (echo)
        - Force timestamp
        """
        if self.shutdown_flag:
            return
        
        # 1. Extract and publish force to haptic device
        receive_time = rospy.get_time()
        
        force_msg = OmniFeedback()
        force_msg.force.x = msg.force.x
        force_msg.force.y = msg.force.y
        force_msg.force.z = msg.force.z
        
        self.force_pub.publish(force_msg)
        
        applied_time = rospy.get_time()
        diff = applied_time - receive_time
        self.force_receive_log.append((receive_time, applied_time, diff))
        
        # ADDED: Store latest force data (not used for bundling, just for local application)
        self.latest_force_data = msg.force
        
        # 2. Calculate pose RTD (Round Trip Delay)
        pose_timestamp = msg.pose_timestamp
        if pose_timestamp > 0:  # Valid timestamp
            current_time = rospy.get_time()
            pose_delay = current_time - pose_timestamp
            print(f"Delay for pose: {pose_delay}")
            self.pose_delays.append(pose_delay)
            self.pose_delay_entries.append((pose_timestamp, pose_delay))
        
        # 3. Echo back force timestamp for slave's RTD calculation
        force_timestamp = msg.force_timestamp
        if force_timestamp > 0:  # Valid timestamp
            # FIXED: Only record FIRST receive time for each unique force timestamp
            if force_timestamp not in self.force_timestamp_first_receive:
                self.force_timestamp_first_receive[force_timestamp] = rospy.get_time()
                rospy.loginfo_throttle(2.0, f"NEW force_timestamp received: {force_timestamp:.6f}")
            
            self.latest_force_timestamp = force_timestamp

    def pose_callback(self, msg: PoseStamped):
        """
        Callback for pose messages from haptic device.
        Implements downsampling by counter: only processes every Nth sample.
        Bundles pose + force_timestamp and sends together.
        """
        # Increment counter
        self.sample_counter += 1
        
        # Only process every DOWNSAMPLE_FACTOR sample (e.g., every 2nd sample for 500Hz)
        if self.sample_counter >= self.DOWNSAMPLE_FACTOR:
            self.latest_pose_msg = msg
            self.sample_counter = 0  # Reset counter
            
            # Create bundled message with ALL data
            bundled_msg = MasterToSlaveData()
            
            # 1. Add pose timestamp in header
            bundled_msg.header = msg.header  # Contains pose generation timestamp
            
            # 2. Add pose data (position + orientation = 7 values)
            bundled_msg.pose = msg.pose
            
            # 3. Add force timestamp (echo back the latest received from slave)
            # This ensures every pose packet carries the most recent force_timestamp
            bundled_msg.force_timestamp = self.latest_force_timestamp
            
            # Publish the bundled message
            gen_time = msg.header.stamp.to_sec()
            sent_time = rospy.get_time()
            
            self.master_to_slave_pub.publish(bundled_msg)
            
            # FIXED: Log force timestamp waiting time ONLY ONCE per unique timestamp
            if self.latest_force_timestamp > 0 and self.latest_force_timestamp in self.force_timestamp_first_receive:
                receive_time = self.force_timestamp_first_receive[self.latest_force_timestamp]
                wait_time = sent_time - receive_time
                
                self.force_timestamp_wait_log.append((
                    self.latest_force_timestamp,  # force_timestamp value
                    receive_time,                 # FIRST receive time at master
                    sent_time,                    # when sent in bundle
                    wait_time                     # waiting time
                ))
                
                rospy.loginfo_throttle(2.0, f"Force_ts {self.latest_force_timestamp:.6f} wait: {wait_time*1000:.3f}ms")
                
                # Remove from dictionary to avoid logging same timestamp again
                del self.force_timestamp_first_receive[self.latest_force_timestamp]
            
            # Log the data
            self.pose_pub_timestamps.append(sent_time)
            diff = sent_time - gen_time
            self.pose_gen_send_log.append((gen_time, sent_time, diff))
            
            rospy.loginfo_throttle(1.0, 
                f"Sent bundled @ {self.DESIRED_SAMPLING_RATE}Hz: "
                f"pose_time={gen_time:.6f}, force_ts_echo={self.latest_force_timestamp:.6f}")

    def save_delays_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/posed.csv"):
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Timestamp from Slave", "Pose Delay (s)"])
                for timestamp, delay in self.pose_delay_entries:
                    writer.writerow([f"{timestamp:.9f}", f"{delay:.9f}"])
            rospy.loginfo(f"Pose delay log saved to: {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to write pose delays to CSV: {e}")

    def save_pose_pub_timestamps_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/pose_pub_timestamps.csv"):
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["Pose Publish Timestamp (s)"])
                for ts in self.pose_pub_timestamps:
                    writer.writerow([f"{ts:.9f}"])
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

    def save_force_receive_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/force_processing.csv"):
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Receive Time', 'Applied Time', 'Difference (s)'])
                for recv, applied, diff in self.force_receive_log:
                    writer.writerow([f"{recv:.9f}", f"{applied:.9f}", f"{diff:.9f}"])
            rospy.loginfo(f"Force receive log saved to: {filepath}")
        except Exception as e:
            rospy.logerr(f"Failed to write force receive log to CSV: {e}")

    def save_force_timestamp_wait_to_csv(self, filepath="/home/autonomous-lab/Desktop/delay/force_timestamp_wait.csv"):
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'Force Timestamp', 
                    'Received at Master (s)', 
                    'Sent in Bundle (s)', 
                    'Wait Time (s)',
                    'Wait Time (ms)'
                ])
                for force_ts, recv_time, send_time, wait_time in self.force_timestamp_wait_log:
                    writer.writerow([
                        f"{force_ts:.9f}", 
                        f"{recv_time:.9f}", 
                        f"{send_time:.9f}", 
                        f"{wait_time:.9f}",
                        f"{wait_time*1000:.3f}"
                    ])
            rospy.loginfo(f"Force timestamp wait log saved to: {filepath}")
            
            # Print statistics
            if self.force_timestamp_wait_log:
                wait_times = [w[3] for w in self.force_timestamp_wait_log]
                rospy.loginfo("="*60)
                rospy.loginfo("Force Timestamp Wait Time Statistics:")
                rospy.loginfo(f"  Average: {sum(wait_times)/len(wait_times)*1000:.3f} ms")
                rospy.loginfo(f"  Min:     {min(wait_times)*1000:.3f} ms")
                rospy.loginfo(f"  Max:     {max(wait_times)*1000:.3f} ms")
                rospy.loginfo(f"  Total samples: {len(wait_times)}")
                rospy.loginfo("="*60)
        except Exception as e:
            rospy.logerr(f"Failed to write force timestamp wait log to CSV: {e}")

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
        self.save_force_receive_to_csv()
        self.save_force_timestamp_wait_to_csv()

    def main_loop(self):
        """
        Main loop - now just spins to keep node alive.
        All pose processing happens in the callback with downsampling.
        """
        rospy.spin()


if __name__ == "__main__":
    try:
        controller = MasterController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass