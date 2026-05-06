#!/usr/bin/env python3

# import rospy
# import numpy as np
# from geometry_msgs.msg import WrenchStamped
# from sensor_msgs.msg import JointState
# from omni_msgs.msg import OmniFeedback
# from std_msgs.msg import Float64
# from collections import deque
# import time
# import matplotlib.pyplot as plt 
# import csv

# class HapticForceController:

#     def __init__(self):

#         # Initializing node
#         rospy.init_node('haptic_force_controller', anonymous=True)

#         # Publishers
#         self.force_pub = rospy.Publisher('force_from_slave_to_master', OmniFeedback, queue_size=1)
#         self.time_pub = rospy.Publisher('time_from_slave_to_master_for_force', Float64, queue_size=1)
#         self.last_sent_force_timestamp = None

#         # Subscribers
#         rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
#         rospy.Subscriber('time_from_master_to_slave_for_force', Float64, self.time_from_master_to_slave_for_force_callback)

#         # For initialization parameter
#         self.start_time = time.time() 
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_force = np.zeros(3)
        
        
#         self.force_sent_timestamps = []   # just sent times
#         self.force_gen_send_log = []      # gen_time, sent_time, diff for force_gen_send.csv
#         self.sent_force_times = {}


#         # For moving average filter
#         self.haptic_window_size = 100
#         self.force_window = deque(maxlen=self.haptic_window_size)

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.robot_force_plot = []
#         self.haptic_force_plot = []
#         self.delay_log = []
#         self.delay_data_log = []  # Stores (timestamp, delay)


#     # def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#     #     received_time = msg.data
#     #     current_time = rospy.get_time()
#     #     #delay = (current_time - received_time) / 2  # One-way delay estimate
#     #     delay = current_time - received_time
#     #     print(f'[DEBUG] Received timestamp: {received_time}, Delay: {delay}')
    
#     #     self.delay_log.append(delay)
#     #     self.delay_data_log.append((received_time, delay))

#     # def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#     # # msg.data is the timestamp originally sent by the slave
#     #     if self.last_sent_force_timestamp is not None and msg.data == self.last_sent_force_timestamp:
#     #         round_trip_delay = rospy.get_time() - msg.data
#     #         print(f"[FORCE DELAY] Round-trip delay: {round_trip_delay:.6f} s")

#     #         self.delay_log.append(round_trip_delay)
#     #         self.delay_data_log.append((msg.data, round_trip_delay))

#     def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#         sent_ts = msg.data
#         if sent_ts in self.sent_force_times:
#             rtd = rospy.get_time() - sent_ts
#             print(f"[FORCE DELAY] Round-trip delay: {rtd:.6f} s")

#             self.delay_log.append(rtd)
#             self.delay_data_log.append((sent_ts, rtd))

#             # cleanup to avoid memory growth
#             del self.sent_force_times[sent_ts]

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))
#         self.haptic_force_plot.append(self.haptic_force)    

#     @staticmethod
#     def forcevector_conversion(joint_position_robot, robot_force):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         r11 = -(cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))
#         r12 = -(-sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6))
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
#         r21 = -(-cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6))
#         r22 = -(sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1))
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
#         r31 = -(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6))
#         r32 = -(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6))
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
#         R = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

#         z_axis_sensor_base = R[:, 2]
#         z_axis_base = np.array([0, 0, 1])
    
#         cross_product = np.cross(z_axis_base, z_axis_sensor_base)
#         dot_product = np.dot(z_axis_base, z_axis_sensor_base)

#         magnitude_of_cross_product_vector = np.linalg.norm(cross_product)

#         unit_cross_product_base = cross_product / magnitude_of_cross_product_vector
#         unit_cross_product_sensor = R.T @ unit_cross_product_base

#         angle = np.arctan2(magnitude_of_cross_product_vector, dot_product)

#         # correction_factor = (1.34 - 0.5)*sin(angle)*unit_cross_product_sensor
#         correction_factor = (1.34+0.15)*sin(angle)*unit_cross_product_sensor

#         # g_base = np.array([0, 0, -3.6335+1.5])
#         g_base = np.array([0, 0, -3.6335])
#         g_sensor = np.dot(R.T, g_base)

#         # can be added 0.2 in the z component
#         # F_offset = np.array([0, 0, -3.834+0.25+1.5])
#         F_offset = np.array([0, 0, -3.834+0.51-0.4+0.02+0.04])

#         robot_force = robot_force - g_sensor - F_offset - correction_factor
#         return (np.dot(R, robot_force))

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

#     def robot_force_callback(self, msg: WrenchStamped):
#         if time.time() - self.start_time < 1:
#             rospy.loginfo("booting")
#         robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
#         self.force_window.append(robot_force)
#         self.robot_force = np.mean(self.force_window, axis=0)

#     # def haptic_force_callback(self, event):
#     #     if self.shutdown_flag:
#     #         return
#     #     force_pub_msg = OmniFeedback()
#     #     robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#     #     force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
#     #     self.force_pub.publish(force_pub_msg)

#     #     time_msg = Float64()
#     #     time_msg.data = rospy.get_time()
#     #     self.time_pub.publish(time_msg)

#     #     self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])
#     #     self.update_list()

#     #     self.last_sent_force_timestamp = time_msg.data
#     #     self.time_pub.publish(time_msg)

#     # def haptic_force_callback(self, event):
#     #     if self.shutdown_flag:
#     #         return

#     # # Prepare and publish haptic force
#     #     force_pub_msg = OmniFeedback()
#     #     robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#     #     force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
#     #     self.force_pub.publish(force_pub_msg)

#     # # --- NEW LOGGING FOR TIMESTAMPS ---
#     #     gen_time = time.time()                # wall-clock generation time
#     #     time_msg = Float64()
#     #     time_msg.data = rospy.get_time()      # ROS time (sent timestamp)

#     #     self.last_sent_force_timestamp = time_msg.data
#     #     self.time_pub.publish(time_msg)
        
#     # # Store both for CSV logging
#     #     self.force_sent_timestamps.append(time_msg.data)  # clean float
#     #     diff = time_msg.data - gen_time
#     #     self.force_gen_send_log.append((gen_time, time_msg.data, diff))

#     # # Store haptic force for plotting/logging
#     #     self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])

#     # # Update plots and lists
#     #     self.update_list()

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#         # Prepare and publish haptic force
#         force_pub_msg = OmniFeedback()
#         robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
#         self.force_pub.publish(force_pub_msg)

#         # --- LOGGING FOR TIMESTAMPS ---
#         gen_time = time.time()                # wall-clock generation time
#         time_msg = Float64()
#         time_msg.data = rospy.get_time()      # ROS time (sent timestamp)

#         # Save mapping for RTD lookup
#         self.sent_force_times[time_msg.data] = gen_time  

#         self.time_pub.publish(time_msg)

#         # Store both for CSV logging
#         self.force_sent_timestamps.append(time_msg.data)
#         diff = time_msg.data - gen_time
#         self.force_gen_send_log.append((gen_time, time_msg.data, diff))

#         # Store haptic force for plotting/logging
#         self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])

#         # Update plots and lists
#         self.update_list()    

#     def plot_data(self):

#         times = np.array(self.time_stamps)
#         robot_force_plot = np.array(self.robot_force_plot)
#         haptic_force_plot = np.array(self.haptic_force_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, robot_force_plot[:, 0], label='robot force')
#         axs[0, 0].plot(times, haptic_force_plot[:, 0], label='haptic force')
#         axs[0, 0].set_title('Force in x')
#         axs[0, 0].set_ylabel('Newtons')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)
#         axs[0, 0].legend()

#         axs[1, 0].plot(times, robot_force_plot[:, 1], label='robot force')
#         axs[1, 0].plot(times, haptic_force_plot[:, 1], label='haptic force')
#         axs[1, 0].set_title('Force in y')
#         axs[1, 0].set_ylabel('Newtons')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)
#         axs[1, 0].legend()

#         axs[2, 0].plot(times, robot_force_plot[:, 2], label='robot force')
#         axs[2, 0].plot(times, haptic_force_plot[:, 2], label='haptic force')
#         axs[2, 0].set_title('Force in z')
#         axs[2, 0].set_ylabel('Newtons')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)
#         axs[2, 0].legend()

#         axs[0, 1].plot(times, robot_force_plot[:, 0] - haptic_force_plot[:, 0])
#         axs[0, 1].set_title('Error in x')
#         axs[0, 1].set_ylabel('Newtons')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)

#         axs[1, 1].plot(times, robot_force_plot[:, 1] - haptic_force_plot[:, 1])
#         axs[1, 1].set_title('Error in y')
#         axs[1, 1].set_ylabel('Newtons')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)

#         axs[2, 1].plot(times, robot_force_plot[:, 2] - haptic_force_plot[:, 2])
#         axs[2, 1].set_title('Error in z')
#         axs[2, 1].set_ylabel('Newtons')
#         axs[2, 1].set_xlabel('Seconds')
#         axs[2, 1].grid(True)

#         plt.tight_layout()
#         plt.show()

#     # def make_csv(self):
#     #     with open('/home/user/Desktop/force2.csv', 'w', newline='') as csvfile:
#     #         writer = csv.writer(csvfile)
#     #         writer.writerow(['Time (s)', 'Robot Force X (N)', 'Robot Force Y (N)', 'Robot Force Z (N)',
#     #                          'Haptic Force X (N)', 'Haptic Force Y (N)', 'Haptic Force Z (N)'])
#     #         for i in range(len(self.time_stamps)):
#     #             writer.writerow([self.time_stamps[i], 
#     #                              self.robot_force_plot[i][0], self.robot_force_plot[i][1], self.robot_force_plot[i][2],
#     #                              self.haptic_force_plot[i][0], self.haptic_force_plot[i][1], self.haptic_force_plot[i][2]])

#     def make_csv(self):
#         with open('/home/user/Desktop/delay/force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Timestamp Sent to Master', 'Time (s)',
#                 'Robot Force X (N)', 'Robot Force Y (N)', 'Robot Force Z (N)',
#                 'Haptic Force X (N)', 'Haptic Force Y (N)', 'Haptic Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 ts_sent = self.force_sent_timestamps[i] if i < len(self.force_sent_timestamps) else ''
#                 writer.writerow([
#                     f"{ts_sent:.9f}" if ts_sent != '' else '',
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot[i][0], self.robot_force_plot[i][1], self.robot_force_plot[i][2],
#                     self.haptic_force_plot[i][0], self.haptic_force_plot[i][1], self.haptic_force_plot[i][2]
#                 ])

                
#     def make_csv_for_delay(self):
#         with open('/home/user/Desktop/delay/forced.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Timestamp received from master', 'Delay'])
#             for entry in self.delay_data_log:
#                 writer.writerow([entry[0], entry[1]])

#     def make_csv_for_generation_and_send(self):
#         with open('/home/user/Desktop/delay/force_gen_send.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Generation Time (s)', 'Sent Timestamp (ROS Time)', 'Difference (s)'])
#             for gen_time, sent_time, diff in self.force_gen_send_log:
#                 writer.writerow([
#                     f"{gen_time:.9f}",
#                     f"{sent_time:.9f}",
#                     f"{diff:.9f}"
#                 ])
  


#     def main_loop(self):
#         rate = 500
#         rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
#         rospy.spin()

#     def shutdown_hook(self):
#         self.shutdown_flag = True
#         zero_force_msg = OmniFeedback()
#         zero_force_msg.force.x, zero_force_msg.force.y, zero_force_msg.force.z = (0, 0, 0)
#         rospy.loginfo("Sending zero force to haptic device.")
#         for _ in range(10):
#             self.force_pub.publish(zero_force_msg)
#             time.sleep(0.1)
#         rospy.loginfo("Shutting down, sent zero force to haptic device.")
#         self.make_csv()
#         #self.plot_data()
#         self.make_csv_for_delay()
#         self.make_csv_for_generation_and_send() 

# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass

#MESSAGE

#!/usr/bin/env python3

# import rospy
# import numpy as np
# from geometry_msgs.msg import WrenchStamped, Vector3
# from sensor_msgs.msg import JointState
# from omni_msgs.msg import OmniFeedback
# from std_msgs.msg import Float64
# from collections import deque
# import time
# import matplotlib.pyplot as plt 
# import csv



# class HapticForceController:

#     def __init__(self):

#         # Initializing node
#         rospy.init_node('haptic_force_controller', anonymous=True)

#         # Publishers - NOW PUBLISHING TO INTERNAL TOPICS (for pose code to bundle)
#         self.internal_force_pub = rospy.Publisher('internal_force_data', Vector3, queue_size=1)
#         self.internal_force_timestamp_pub = rospy.Publisher('internal_force_timestamp', Float64, queue_size=1)
        
#         # Keep these for logging/compatibility if needed
#         self.last_sent_force_timestamp = None

#         # Subscribers
#         rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
#         # Note: No longer subscribing to time_from_master_to_slave_for_force
#         # The pose code will handle timestamp echoing

#         # For initialization parameter
#         self.start_time = time.time() 
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_force = np.zeros(3)
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}

#         # For moving average filter
#         self.haptic_window_size = 100
#         self.force_window = deque(maxlen=self.haptic_window_size)

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.robot_force_plot = []
#         self.haptic_force_plot = []
#         self.delay_log = []
#         self.delay_data_log = []
        
#         rospy.loginfo("Force controller initialized - publishing to internal topics")

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))
#         self.haptic_force_plot.append(self.haptic_force)    

#     @staticmethod
#     def forcevector_conversion(joint_position_robot, robot_force):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         r11 = -(cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))
#         r12 = -(-sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6))
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
#         r21 = -(-cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6))
#         r22 = -(sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1))
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
#         r31 = -(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6))
#         r32 = -(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6))
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
#         R = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

#         z_axis_sensor_base = R[:, 2]
#         z_axis_base = np.array([0, 0, 1])
    
#         cross_product = np.cross(z_axis_base, z_axis_sensor_base)
#         dot_product = np.dot(z_axis_base, z_axis_sensor_base)

#         magnitude_of_cross_product_vector = np.linalg.norm(cross_product)

#         unit_cross_product_base = cross_product / magnitude_of_cross_product_vector
#         unit_cross_product_sensor = R.T @ unit_cross_product_base

#         angle = np.arctan2(magnitude_of_cross_product_vector, dot_product)

#         correction_factor = (1.34+0.15)*sin(angle)*unit_cross_product_sensor

#         g_base = np.array([0, 0, -3.6335])
#         g_sensor = np.dot(R.T, g_base)

#         F_offset = np.array([0, 0, -3.834+0.51-0.4+0.02+0.04])

#         robot_force = robot_force - g_sensor - F_offset - correction_factor
#         return (np.dot(R, robot_force))

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

#     def robot_force_callback(self, msg: WrenchStamped):
#         if time.time() - self.start_time < 1:
#             rospy.loginfo("booting")
#         robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
#         self.force_window.append(robot_force)
#         self.robot_force = np.mean(self.force_window, axis=0)

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#         # Calculate haptic force
#         robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        
#         # Create force message (Vector3 instead of OmniFeedback)
#         force_msg = Vector3()
#         force_msg.x = robot_force[0]
#         force_msg.y = robot_force[1]
#         force_msg.z = robot_force[2]
        
#         # Publish to internal topic (pose code will bundle this)
#         self.internal_force_pub.publish(force_msg)

#         # Publish force timestamp to internal topic
#         force_timestamp = rospy.get_time()
#         self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

#         # --- LOGGING ---
#         gen_time = time.time()
#         self.force_sent_timestamps.append(force_timestamp)
#         diff = force_timestamp - gen_time
#         self.force_gen_send_log.append((gen_time, force_timestamp, diff))

#         # Store haptic force for plotting/logging
#         self.haptic_force = robot_force

#         # Update plots and lists
#         self.update_list()    

#     def plot_data(self):
#         times = np.array(self.time_stamps)
#         robot_force_plot = np.array(self.robot_force_plot)
#         haptic_force_plot = np.array(self.haptic_force_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, robot_force_plot[:, 0], label='robot force')
#         axs[0, 0].plot(times, haptic_force_plot[:, 0], label='haptic force')
#         axs[0, 0].set_title('Force in x')
#         axs[0, 0].set_ylabel('Newtons')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)
#         axs[0, 0].legend()

#         axs[1, 0].plot(times, robot_force_plot[:, 1], label='robot force')
#         axs[1, 0].plot(times, haptic_force_plot[:, 1], label='haptic force')
#         axs[1, 0].set_title('Force in y')
#         axs[1, 0].set_ylabel('Newtons')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)
#         axs[1, 0].legend()

#         axs[2, 0].plot(times, robot_force_plot[:, 2], label='robot force')
#         axs[2, 0].plot(times, haptic_force_plot[:, 2], label='haptic force')
#         axs[2, 0].set_title('Force in z')
#         axs[2, 0].set_ylabel('Newtons')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)
#         axs[2, 0].legend()

#         axs[0, 1].plot(times, robot_force_plot[:, 0] - haptic_force_plot[:, 0])
#         axs[0, 1].set_title('Error in x')
#         axs[0, 1].set_ylabel('Newtons')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)

#         axs[1, 1].plot(times, robot_force_plot[:, 1] - haptic_force_plot[:, 1])
#         axs[1, 1].set_title('Error in y')
#         axs[1, 1].set_ylabel('Newtons')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)

#         axs[2, 1].plot(times, robot_force_plot[:, 2] - haptic_force_plot[:, 2])
#         axs[2, 1].set_title('Error in z')
#         axs[2, 1].set_ylabel('Newtons')
#         axs[2, 1].set_xlabel('Seconds')
#         axs[2, 1].grid(True)

#         plt.tight_layout()
#         plt.show()

#     def make_csv(self):
#         with open('/home/user/Desktop/delay/force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Timestamp Sent to Internal', 'Time (s)',
#                 'Robot Force X (N)', 'Robot Force Y (N)', 'Robot Force Z (N)',
#                 'Haptic Force X (N)', 'Haptic Force Y (N)', 'Haptic Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 ts_sent = self.force_sent_timestamps[i] if i < len(self.force_sent_timestamps) else ''
#                 writer.writerow([
#                     f"{ts_sent:.9f}" if ts_sent != '' else '',
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot[i][0], self.robot_force_plot[i][1], self.robot_force_plot[i][2],
#                     self.haptic_force_plot[i][0], self.haptic_force_plot[i][1], self.haptic_force_plot[i][2]
#                 ])

#     def make_csv_for_generation_and_send(self):
#         with open('/home/user/Desktop/delay/force_gen_send.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Generation Time (s)', 'Sent Timestamp (ROS Time)', 'Difference (s)'])
#             for gen_time, sent_time, diff in self.force_gen_send_log:
#                 writer.writerow([
#                     f"{gen_time:.9f}",
#                     f"{sent_time:.9f}",
#                     f"{diff:.9f}"
#                 ])

#     def main_loop(self):
#         rate = 500
#         rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
#         rospy.spin()

#     def shutdown_hook(self):
#         self.shutdown_flag = True
#         zero_force_msg = Vector3()
#         zero_force_msg.x, zero_force_msg.y, zero_force_msg.z = (0, 0, 0)
#         rospy.loginfo("Sending zero force to internal topic.")
#         for _ in range(10):
#             self.internal_force_pub.publish(zero_force_msg)
#             time.sleep(0.1)
#         rospy.loginfo("Shutting down, sent zero force.")
#         self.make_csv()
#         self.make_csv_for_generation_and_send() 

# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass


#The above force code works perfectly file 
#here we are logging the force round trip delay


# import rospy
# import numpy as np
# from geometry_msgs.msg import WrenchStamped, Vector3
# from sensor_msgs.msg import JointState
# from omni_msgs.msg import OmniFeedback
# from std_msgs.msg import Float64
# from teleop_msgs.msg import SlaveToMasterData  # ADDED: To receive RTD echoes
# from collections import deque
# import time
# import matplotlib.pyplot as plt 
# import csv

# class HapticForceController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('haptic_force_controller', anonymous=True)

#         # Publishers - Publishing to internal topics (for pose code to bundle) (SENDS ALL BUNDLED DATA TO INTERNAL POSE CODE)
#         self.internal_force_pub = rospy.Publisher('internal_force_data', Vector3, queue_size=1)
#         self.internal_force_timestamp_pub = rospy.Publisher('internal_force_timestamp', Float64, queue_size=1)
        
#         self.last_sent_force_timestamp = None

#         # Subscribers
#         rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # ADDED: Subscribe to bundled data from master (to get force timestamp echo)
#         rospy.Subscriber('slave_to_master_data', SlaveToMasterData, self.master_echo_callback)

#         # For initialization parameter
#         self.start_time = time.time() 
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_force = np.zeros(3)
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}
        
#         # ADDED: For RTD logging
#         self.force_rtd_log = []  # Stores (timestamp, RTD)

#         # For moving average filter
#         self.haptic_window_size = 100
#         self.force_window = deque(maxlen=self.haptic_window_size)

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.robot_force_plot = []
#         self.haptic_force_plot = []
        
#         rospy.loginfo("Force controller initialized - publishing to internal topics")
#         rospy.loginfo("Force RTD logging enabled")

#     # ADDED: Callback to receive echo from master and calculate RTD
#     def master_echo_callback(self, msg: SlaveToMasterData):
#         """
#         Receives bundled data that pose code receives from master.
#         This contains the echo of our force_timestamp.
#         """
#         force_timestamp_echo = msg.force_timestamp
        
#         if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
#             # Calculate RTD
#             current_time = rospy.get_time()
#             rtd = current_time - force_timestamp_echo
            
#             # Log it
#             self.force_rtd_log.append((force_timestamp_echo, rtd))
            
#             rospy.loginfo_throttle(1.0, f"Force RTD: {rtd:.6f}s")
            
#             # Cleanup
#             del self.sent_force_times[force_timestamp_echo]

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))
#         self.haptic_force_plot.append(self.haptic_force)    

#     @staticmethod
#     def forcevector_conversion(joint_position_robot, robot_force):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         r11 = -(cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))
#         r12 = -(-sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6))
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
#         r21 = -(-cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6))
#         r22 = -(sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1))
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
#         r31 = -(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6))
#         r32 = -(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6))
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
#         R = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

#         z_axis_sensor_base = R[:, 2]
#         z_axis_base = np.array([0, 0, 1])
    
#         cross_product = np.cross(z_axis_base, z_axis_sensor_base)
#         dot_product = np.dot(z_axis_base, z_axis_sensor_base)

#         magnitude_of_cross_product_vector = np.linalg.norm(cross_product)

#         unit_cross_product_base = cross_product / magnitude_of_cross_product_vector
#         unit_cross_product_sensor = R.T @ unit_cross_product_base

#         angle = np.arctan2(magnitude_of_cross_product_vector, dot_product)

#         correction_factor = (1.34+0.15)*sin(angle)*unit_cross_product_sensor

#         g_base = np.array([0, 0, -3.6335])
#         g_sensor = np.dot(R.T, g_base)

#         F_offset = np.array([0, 0, -3.834+0.51-0.4+0.02+0.04])

#         robot_force = robot_force - g_sensor - F_offset - correction_factor
#         return (np.dot(R, robot_force))

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

#     def robot_force_callback(self, msg: WrenchStamped):
#         if time.time() - self.start_time < 1:
#             rospy.loginfo("booting")
#         robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
#         self.force_window.append(robot_force)
#         self.robot_force = np.mean(self.force_window, axis=0)

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#         # Calculate haptic force
#         robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        
#         # Create force message (Vector3)
#         force_msg = Vector3()
#         force_msg.x = robot_force[0]
#         force_msg.y = robot_force[1]
#         force_msg.z = robot_force[2]
        
#         # Publish to internal topic (pose code will bundle this)
#         self.internal_force_pub.publish(force_msg)

#         # Publish force timestamp to internal topic
#         force_timestamp = rospy.get_time()
#         self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

#         # ADDED: Store timestamp for RTD calculation
#         self.sent_force_times[force_timestamp] = time.time()

#         # Logging
#         gen_time = time.time()
#         self.force_sent_timestamps.append(force_timestamp)
#         diff = force_timestamp - gen_time
#         self.force_gen_send_log.append((gen_time, force_timestamp, diff))

#         # Store haptic force for plotting/logging
#         self.haptic_force = robot_force

#         # Update plots and lists
#         self.update_list()    

#     def plot_data(self):
#         times = np.array(self.time_stamps)
#         robot_force_plot = np.array(self.robot_force_plot)
#         haptic_force_plot = np.array(self.haptic_force_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, robot_force_plot[:, 0], label='robot force')
#         axs[0, 0].plot(times, haptic_force_plot[:, 0], label='haptic force')
#         axs[0, 0].set_title('Force in x')
#         axs[0, 0].set_ylabel('Newtons')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)
#         axs[0, 0].legend()

#         axs[1, 0].plot(times, robot_force_plot[:, 1], label='robot force')
#         axs[1, 0].plot(times, haptic_force_plot[:, 1], label='haptic force')
#         axs[1, 0].set_title('Force in y')
#         axs[1, 0].set_ylabel('Newtons')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)
#         axs[1, 0].legend()

#         axs[2, 0].plot(times, robot_force_plot[:, 2], label='robot force')
#         axs[2, 0].plot(times, haptic_force_plot[:, 2], label='haptic force')
#         axs[2, 0].set_title('Force in z')
#         axs[2, 0].set_ylabel('Newtons')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)
#         axs[2, 0].legend()

#         axs[0, 1].plot(times, robot_force_plot[:, 0] - haptic_force_plot[:, 0])
#         axs[0, 1].set_title('Error in x')
#         axs[0, 1].set_ylabel('Newtons')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)

#         axs[1, 1].plot(times, robot_force_plot[:, 1] - haptic_force_plot[:, 1])
#         axs[1, 1].set_title('Error in y')
#         axs[1, 1].set_ylabel('Newtons')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)

#         axs[2, 1].plot(times, robot_force_plot[:, 2] - haptic_force_plot[:, 2])
#         axs[2, 1].set_title('Error in z')
#         axs[2, 1].set_ylabel('Newtons')
#         axs[2, 1].set_xlabel('Seconds')
#         axs[2, 1].grid(True)

#         plt.tight_layout()
#         plt.show()

#     def make_csv(self):
#         with open('/home/user/Desktop/delay/force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Timestamp Sent to Internal', 'Time (s)',
#                 'Robot Force X (N)', 'Robot Force Y (N)', 'Robot Force Z (N)',
#                 'Haptic Force X (N)', 'Haptic Force Y (N)', 'Haptic Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 ts_sent = self.force_sent_timestamps[i] if i < len(self.force_sent_timestamps) else ''
#                 writer.writerow([
#                     f"{ts_sent:.9f}" if ts_sent != '' else '',
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot[i][0], self.robot_force_plot[i][1], self.robot_force_plot[i][2],
#                     self.haptic_force_plot[i][0], self.haptic_force_plot[i][1], self.haptic_force_plot[i][2]
#                 ])

#     def make_csv_for_generation_and_send(self):
#         with open('/home/user/Desktop/delay/force_gen_send.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Generation Time (s)', 'Sent Timestamp (ROS Time)', 'Difference (s)'])
#             for gen_time, sent_time, diff in self.force_gen_send_log:
#                 writer.writerow([
#                     f"{gen_time:.9f}",
#                     f"{sent_time:.9f}",
#                     f"{diff:.9f}"
#                 ])

#     # ADDED: Save Force RTD to CSV
#     def make_csv_for_force_rtd(self):
#         """Save force round-trip delay to CSV"""
#         with open('/home/user/Desktop/delay/force_rtd.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Force Timestamp', 'Force RTD (s)'])
#             for timestamp, rtd in self.force_rtd_log:
#                 writer.writerow([f"{timestamp:.9f}", f"{rtd:.9f}"])
#         rospy.loginfo("Force RTD log saved to: /home/user/Desktop/delay/force_rtd.csv")

#     def main_loop(self):
#         rate = 500
#         rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
#         rospy.spin()

#     def shutdown_hook(self):
#         self.shutdown_flag = True
#         zero_force_msg = Vector3()
#         zero_force_msg.x, zero_force_msg.y, zero_force_msg.z = (0, 0, 0)
#         rospy.loginfo("Sending zero force to internal topic.")
#         for _ in range(10):
#             self.internal_force_pub.publish(zero_force_msg)
#             time.sleep(0.1)
#         rospy.loginfo("Shutting down, sent zero force.")
        
#         # Save all CSVs
#         self.make_csv()
#         self.make_csv_for_generation_and_send()
#         self.make_csv_for_force_rtd()  # ADDED

# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass

#purpose :To rectify the issue of force rtd

#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Vector3
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback
from std_msgs.msg import Float64
from teleop_msgs.msg import MasterToSlaveData  # FIXED: Changed from SlaveToMasterData
from collections import deque
import time
import matplotlib.pyplot as plt 
import csv

class HapticForceController:

    def __init__(self):
        # Initializing node
        rospy.init_node('haptic_force_controller', anonymous=True)

        # Publishers - Publishing to internal topics (for pose code to bundle)
        self.internal_force_pub = rospy.Publisher('internal_force_data', Vector3, queue_size=1)
        self.internal_force_timestamp_pub = rospy.Publisher('internal_force_timestamp', Float64, queue_size=1)
        
        self.last_sent_force_timestamp = None

        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
        # FIXED: Subscribe to data coming FROM master (master_to_slave_data, not slave_to_master_data)
        rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_echo_callback)

        # For initialization parameter
        self.start_time = time.time() 
        self.robot_force = np.zeros(3)
        self.robot_force_initial = np.zeros(3)
        self.joint_position_robot = np.zeros(6)
        self.haptic_force = np.zeros(3)
        
        self.force_sent_timestamps = []
        self.force_gen_send_log = []
        self.sent_force_times = {}
        
        # For RTD logging
        self.force_rtd_log = []  # Stores (timestamp, RTD)

        # For moving average filter #Original it was 100
        self.haptic_window_size = 100
        self.force_window = deque(maxlen=self.haptic_window_size)

        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.shutdown_flag = False

        # For plotting
        self.start_time = None
        self.time_stamps = []
        self.robot_force_plot = []
        self.haptic_force_plot = []
        
        rospy.loginfo("Force controller initialized - publishing to internal topics")
        rospy.loginfo("Force RTD logging enabled - subscribing to 'master_to_slave_data'")

    # FIXED: Receives data FROM master (MasterToSlaveData type)
    def master_echo_callback(self, msg: MasterToSlaveData):
        """
        Receives bundled data from master containing:
        - Pose data (for robot)
        - Pose timestamp (for pose RTD)
        - Force timestamp (echo for force RTD)
        """
        force_timestamp_echo = msg.force_timestamp
        
        if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
            # Calculate RTD
            current_time = rospy.get_time()
            rtd = current_time - force_timestamp_echo
            
            # Log it
            self.force_rtd_log.append((force_timestamp_echo, rtd))
            
            rospy.loginfo_throttle(1.0, f"Force RTD: {rtd*1000:.3f}ms")
            
            # Cleanup
            del self.sent_force_times[force_timestamp_echo]

    def update_list(self):
        if self.shutdown_flag:
            return
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))
        self.haptic_force_plot.append(self.haptic_force)    

    @staticmethod
    def forcevector_conversion(joint_position_robot, robot_force):
        q1, q2, q3, q4, q5, q6 = joint_position_robot
        cos = np.cos
        sin = np.sin
        r11 = -(cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6))
        r12 = -(-sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6))
        r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        r21 = -(-cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6))
        r22 = -(sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1))
        r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        r31 = -(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6))
        r32 = -(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6))
        r33 = -sin(q2 + q3 + q4)*sin(q5)
        R = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])

        z_axis_sensor_base = R[:, 2]
        z_axis_base = np.array([0, 0, 1])
    
        cross_product = np.cross(z_axis_base, z_axis_sensor_base)
        dot_product = np.dot(z_axis_base, z_axis_sensor_base)

        magnitude_of_cross_product_vector = np.linalg.norm(cross_product)

        unit_cross_product_base = cross_product / magnitude_of_cross_product_vector
        unit_cross_product_sensor = R.T @ unit_cross_product_base

        angle = np.arctan2(magnitude_of_cross_product_vector, dot_product)

        correction_factor = (1.34+0.15)*sin(angle)*unit_cross_product_sensor

        g_base = np.array([0, 0, -3.6335])
        g_sensor = np.dot(R.T, g_base)

        F_offset = np.array([0, 0, -3.834+0.51-0.4+0.02+0.04])

        robot_force = robot_force - g_sensor - F_offset - correction_factor
        return (np.dot(R, robot_force))

    def joint_callback_robot(self, msg: JointState):
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

    def robot_force_callback(self, msg: WrenchStamped):
        if time.time() - self.start_time < 1:
            rospy.loginfo("booting")
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

    def haptic_force_callback(self, event):
        if self.shutdown_flag:
            return

        # Calculate haptic force
        robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        
        # Create force message (Vector3)
        force_msg = Vector3()
        force_msg.x = robot_force[0]
        force_msg.y = robot_force[1]
        force_msg.z = robot_force[2]
        
        # Publish to internal topic (pose code will bundle this)
        self.internal_force_pub.publish(force_msg)

        # Publish force timestamp to internal topic
        force_timestamp = rospy.get_time()
        self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

        # Store timestamp for RTD calculation
        self.sent_force_times[force_timestamp] = time.time()

        # Logging
        gen_time = time.time()
        self.force_sent_timestamps.append(force_timestamp)
        diff = force_timestamp - gen_time
        self.force_gen_send_log.append((gen_time, force_timestamp, diff))

        # Store haptic force for plotting/logging
        self.haptic_force = robot_force

        # Update plots and lists
        self.update_list()    

    def plot_data(self):
        times = np.array(self.time_stamps)
        robot_force_plot = np.array(self.robot_force_plot)
        haptic_force_plot = np.array(self.haptic_force_plot)

        fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
        axs[0, 0].plot(times, robot_force_plot[:, 0], label='robot force')
        axs[0, 0].plot(times, haptic_force_plot[:, 0], label='haptic force')
        axs[0, 0].set_title('Force in x')
        axs[0, 0].set_ylabel('Newtons')
        axs[0, 0].set_xlabel('Seconds')
        axs[0, 0].grid(True)
        axs[0, 0].legend()

        axs[1, 0].plot(times, robot_force_plot[:, 1], label='robot force')
        axs[1, 0].plot(times, haptic_force_plot[:, 1], label='haptic force')
        axs[1, 0].set_title('Force in y')
        axs[1, 0].set_ylabel('Newtons')
        axs[1, 0].set_xlabel('Seconds')
        axs[1, 0].grid(True)
        axs[1, 0].legend()

        axs[2, 0].plot(times, robot_force_plot[:, 2], label='robot force')
        axs[2, 0].plot(times, haptic_force_plot[:, 2], label='haptic force')
        axs[2, 0].set_title('Force in z')
        axs[2, 0].set_ylabel('Newtons')
        axs[2, 0].set_xlabel('Seconds')
        axs[2, 0].grid(True)
        axs[2, 0].legend()

        axs[0, 1].plot(times, robot_force_plot[:, 0] - haptic_force_plot[:, 0])
        axs[0, 1].set_title('Error in x')
        axs[0, 1].set_ylabel('Newtons')
        axs[0, 1].set_xlabel('Seconds')
        axs[0, 1].grid(True)

        axs[1, 1].plot(times, robot_force_plot[:, 1] - haptic_force_plot[:, 1])
        axs[1, 1].set_title('Error in y')
        axs[1, 1].set_ylabel('Newtons')
        axs[1, 1].set_xlabel('Seconds')
        axs[1, 1].grid(True)

        axs[2, 1].plot(times, robot_force_plot[:, 2] - haptic_force_plot[:, 2])
        axs[2, 1].set_title('Error in z')
        axs[2, 1].set_ylabel('Newtons')
        axs[2, 1].set_xlabel('Seconds')
        axs[2, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def make_csv(self):
        with open('/home/user/Desktop/delay/force.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Timestamp Sent to Internal', 'Time (s)',
                'Robot Force X (N)', 'Robot Force Y (N)', 'Robot Force Z (N)',
                'Haptic Force X (N)', 'Haptic Force Y (N)', 'Haptic Force Z (N)'
            ])
            for i in range(len(self.time_stamps)):
                ts_sent = self.force_sent_timestamps[i] if i < len(self.force_sent_timestamps) else ''
                writer.writerow([
                    f"{ts_sent:.9f}" if ts_sent != '' else '',
                    f"{self.time_stamps[i]:.9f}",
                    self.robot_force_plot[i][0], self.robot_force_plot[i][1], self.robot_force_plot[i][2],
                    self.haptic_force_plot[i][0], self.haptic_force_plot[i][1], self.haptic_force_plot[i][2]
                ])

    def make_csv_for_generation_and_send(self):
        with open('/home/user/Desktop/delay/force_gen_send.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Generation Time (s)', 'Sent Timestamp (ROS Time)', 'Difference (s)'])
            for gen_time, sent_time, diff in self.force_gen_send_log:
                writer.writerow([
                    f"{gen_time:.9f}",
                    f"{sent_time:.9f}",
                    f"{diff:.9f}"
                ])

    def make_csv_for_force_rtd(self):
        """Save force round-trip delay to CSV with statistics"""
        with open('/home/user/Desktop/delay/force_rtd.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Force Timestamp', 'Force RTD (s)'])
            for timestamp, rtd in self.force_rtd_log:
                writer.writerow([f"{timestamp:.9f}", f"{rtd:.9f}"])
        
        # Calculate and print statistics
        if self.force_rtd_log:
            rtds = [rtd for _, rtd in self.force_rtd_log]
            rospy.loginfo("="*60)
            rospy.loginfo("Force RTD Statistics:")
            rospy.loginfo(f"  Average: {np.mean(rtds)*1000:.3f} ms")
            rospy.loginfo(f"  Min:     {np.min(rtds)*1000:.3f} ms")
            rospy.loginfo(f"  Max:     {np.max(rtds)*1000:.3f} ms")
            rospy.loginfo(f"  Std Dev: {np.std(rtds)*1000:.3f} ms")
            rospy.loginfo(f"  Total samples: {len(rtds)}")
            rospy.loginfo("="*60)
        
        rospy.loginfo("Force RTD log saved to: /home/user/Desktop/delay/force_rtd.csv")

    def main_loop(self):
        rate = 500
        rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
        rospy.spin()

    def shutdown_hook(self):
        self.shutdown_flag = True
        zero_force_msg = Vector3()
        zero_force_msg.x, zero_force_msg.y, zero_force_msg.z = (0, 0, 0)
        rospy.loginfo("Sending zero force to internal topic.")
        for _ in range(10):
            self.internal_force_pub.publish(zero_force_msg)
            time.sleep(0.1)
        rospy.loginfo("Shutting down, sent zero force.")
        
        # Save all CSVs
        self.make_csv()
        self.make_csv_for_generation_and_send()
        self.make_csv_for_force_rtd()

if __name__ == "__main__":
    try:
        controller = HapticForceController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass