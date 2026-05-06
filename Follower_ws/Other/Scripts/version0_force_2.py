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
#         self.robot_force_corrected_raw = np.zeros(3)


#         self.robot_force_raw = np.zeros(3)     # raw FT sensor values
#         self.robot_force_plot_raw = []         # store raw values for plotting/logging

        
        
#         self.force_sent_timestamps = []   # just sent times
#         self.force_gen_send_log = []      # gen_time, sent_time, diff for force_gen_send.csv


#         # For moving average filter
#         # self.haptic_window_size = 100
#         # self.force_window = deque(maxlen=self.haptic_window_size)

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
#         self.robot_force_corrected_raw_plot = []

#           # Stores (timestamp, delay)


#     # def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#     #     received_time = msg.data
#     #     current_time = rospy.get_time()
#     #     #delay = (current_time - received_time) / 2  # One-way delay estimate
#     #     delay = current_time - received_time
#     #     print(f'[DEBUG] Received timestamp: {received_time}, Delay: {delay}')
    
#     #     self.delay_log.append(delay)
#     #     self.delay_data_log.append((received_time, delay))

#     def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#     # msg.data is the timestamp originally sent by the slave
#         if self.last_sent_force_timestamp is not None and msg.data == self.last_sent_force_timestamp:
#             round_trip_delay = rospy.get_time() - msg.data
#             print(f"[FORCE DELAY] Round-trip delay: {round_trip_delay:.6f} s")

#             self.delay_log.append(round_trip_delay)
#             self.delay_data_log.append((msg.data, round_trip_delay))

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         # log raw (unfiltered) and filtered values
#         self.robot_force_plot_raw.append(self.robot_force_raw)
#         # self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))
#         # self.haptic_force_plot.append(self.haptic_force)
#         self.robot_force_plot.append(
#             self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         )
#         self.robot_force_corrected_raw_plot.append(
#             self.robot_force_corrected_raw
#         )

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
#         self.robot_force_raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
    
#     # push raw into deque for filtering
#         self.force_window.append(self.robot_force_raw)
#         self.robot_force = np.mean(self.force_window, axis=0)

#             # ---- NEW: CORRECTED BUT UNFILTERED FORCE ----
#         self.robot_force_corrected_raw = self.forcevector_conversion(
#             self.joint_position_robot,
#             self.robot_force_raw
#         )


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

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#     # Prepare and publish haptic force
#         force_pub_msg = OmniFeedback()
#         robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
#         self.force_pub.publish(force_pub_msg)

#     # --- NEW LOGGING FOR TIMESTAMPS ---
#         gen_time = time.time()                # wall-clock generation time
#         time_msg = Float64()
#         time_msg.data = rospy.get_time()      # ROS time (sent timestamp)

#         self.last_sent_force_timestamp = time_msg.data
#         self.time_pub.publish(time_msg)
        
#     # Store both for CSV logging
#         self.force_sent_timestamps.append(time_msg.data)  # clean float
#         diff = time_msg.data - gen_time
#         self.force_gen_send_log.append((gen_time, time_msg.data, diff))

#     # Store haptic force for plotting/logging
#         self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])

#     # Update plots and lists
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


#     def make_csv_for_raw_and_filtered(self):
#         with open('/home/user/Desktop/delay/raw_vs_filtered_force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Raw Force X (N)', 'Raw Force Y (N)', 'Raw Force Z (N)',
#                 'Filtered Force X (N)', 'Filtered Force Y (N)', 'Filtered Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot_raw[i][0], self.robot_force_plot_raw[i][1], self.robot_force_plot_raw[i][2],
#                     self.robot_force_plot[i][0], self.robot_force_plot[i][1], self.robot_force_plot[i][2]
#                 ])

#     def make_csv_for_corrected_unfiltered_force(self):
#         with open('/home/user/Desktop/delay/corrected_unfiltered_force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Corrected Raw Force X (N)',
#                 'Corrected Raw Force Y (N)',
#                 'Corrected Raw Force Z (N)'
#             ])

#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_corrected_raw_plot[i][0],
#                     self.robot_force_corrected_raw_plot[i][1],
#                     self.robot_force_corrected_raw_plot[i][2]
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
#         self.make_csv_for_raw_and_filtered()
#         self.make_csv_for_corrected_unfiltered_force()


        
# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass





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
#         self.robot_force_corrected_raw = np.zeros(3)


#         self.robot_force_raw = np.zeros(3)     # raw FT sensor values
#         self.robot_force_plot_raw = []         # store raw values for plotting/logging

        
        
#         self.force_sent_timestamps = []   # just sent times
#         self.force_gen_send_log = []      # gen_time, sent_time, diff for force_gen_send.csv


#         # For moving average filter
#         # self.haptic_window_size = 100
#         # self.force_window = deque(maxlen=self.haptic_window_size)

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
#         self.robot_force_corrected_raw_plot = []

#           # Stores (timestamp, delay)


#     # def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#     #     received_time = msg.data
#     #     current_time = rospy.get_time()
#     #     #delay = (current_time - received_time) / 2  # One-way delay estimate
#     #     delay = current_time - received_time
#     #     print(f'[DEBUG] Received timestamp: {received_time}, Delay: {delay}')
    
#     #     self.delay_log.append(delay)
#     #     self.delay_data_log.append((received_time, delay))

#     def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#     # msg.data is the timestamp originally sent by the slave
#         if self.last_sent_force_timestamp is not None and msg.data == self.last_sent_force_timestamp:
#             round_trip_delay = rospy.get_time() - msg.data
#             print(f"[FORCE DELAY] Round-trip delay: {round_trip_delay:.6f} s")

#             self.delay_log.append(round_trip_delay)
#             self.delay_data_log.append((msg.data, round_trip_delay))

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         # log raw (unfiltered) and filtered values
#         self.robot_force_plot_raw.append(self.robot_force_raw)
#         # self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))
#         # self.haptic_force_plot.append(self.haptic_force)
#         self.robot_force_plot.append(
#             self.forcevector_conversion(self.joint_position_robot, self.robot_force_raw)
#         )
#         self.robot_force_corrected_raw_plot.append(
#             self.robot_force_corrected_raw
#         )

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
#         self.robot_force_raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
     
#         self.robot_force = self.robot_force_raw.copy()
#     # push raw into deque for filtering
#         # self.force_window.append(self.robot_force_raw)
#         # self.robot_force = np.mean(self.force_window, axis=0)

#             # ---- NEW: CORRECTED BUT UNFILTERED FORCE ----
#         self.robot_force_corrected_raw = self.forcevector_conversion(
#             self.joint_position_robot,
#             self.robot_force_raw
#         )


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

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#     # Prepare and publish haptic force
#         force_pub_msg = OmniFeedback()
#         robot_force = np.array([0.0, 0.0, 0.0])
#         force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
#         self.force_pub.publish(force_pub_msg)

#     # --- NEW LOGGING FOR TIMESTAMPS ---
#         gen_time = time.time()                # wall-clock generation time
#         time_msg = Float64()
#         time_msg.data = rospy.get_time()      # ROS time (sent timestamp)

#         self.last_sent_force_timestamp = time_msg.data
#         self.time_pub.publish(time_msg)
        
#     # Store both for CSV logging
#         self.force_sent_timestamps.append(time_msg.data)  # clean float
#         diff = time_msg.data - gen_time
#         self.force_gen_send_log.append((gen_time, time_msg.data, diff))

#     # Store haptic force for plotting/logging
#         self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])

#     # Update plots and lists
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


#     def make_csv_for_raw_and_filtered(self):
#         with open('/home/user/Desktop/delay/raw_vs_filtered_force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Raw Force X (N)', 'Raw Force Y (N)', 'Raw Force Z (N)',
#                 'Filtered Force X (N)', 'Filtered Force Y (N)', 'Filtered Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot_raw[i][0], self.robot_force_plot_raw[i][1], self.robot_force_plot_raw[i][2],
#                     self.robot_force_plot[i][0], self.robot_force_plot[i][1], self.robot_force_plot[i][2]
#                 ])

#     def make_csv_for_corrected_unfiltered_force(self):
#         with open('/home/user/Desktop/delay/corrected_unfiltered_force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Corrected Raw Force X (N)',
#                 'Corrected Raw Force Y (N)',
#                 'Corrected Raw Force Z (N)'
#             ])

#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_corrected_raw_plot[i][0],
#                     self.robot_force_corrected_raw_plot[i][1],
#                     self.robot_force_corrected_raw_plot[i][2]
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
#         self.make_csv_for_raw_and_filtered()
#         self.make_csv_for_corrected_unfiltered_force()


        
# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass



###
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

#         self.robot_force_raw_sensor = np.zeros(3)     # raw FT sensor values (sensor frame)
#         self.robot_force_raw_base = np.zeros(3)       # raw force in base frame (no filtering)
#         self.robot_force_plot_raw_sensor = []         # store raw sensor frame values
#         self.robot_force_plot_raw_base = []           # store raw base frame values
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []

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

#     def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#         if self.last_sent_force_timestamp is not None and msg.data == self.last_sent_force_timestamp:
#             round_trip_delay = rospy.get_time() - msg.data
#             print(f"[FORCE DELAY] Round-trip delay: {round_trip_delay:.6f} s")
#             self.delay_log.append(round_trip_delay)
#             self.delay_data_log.append((msg.data, round_trip_delay))

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
        
#         # Log raw sensor frame values
#         self.robot_force_plot_raw_sensor.append(self.robot_force_raw_sensor.copy())
        
#         # Log raw base frame values (no filtering)
#         self.robot_force_plot_raw_base.append(self.robot_force_raw_base.copy())
        
#         # Log filtered base frame values
#         self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))
        
#         # Log haptic force
#         self.haptic_force_plot.append(self.haptic_force.copy())

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
        
#         # Store raw sensor values
#         self.robot_force_raw_sensor = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        
#         # Convert raw to base frame (no filtering)
#         self.robot_force_raw_base = self.forcevector_conversion(self.joint_position_robot, self.robot_force_raw_sensor)
        
#         # Apply moving average filter for control
#         self.force_window.append(self.robot_force_raw_sensor)
#         self.robot_force = np.mean(self.force_window, axis=0)

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#         # Prepare and publish haptic force (using filtered force)
#         force_pub_msg = OmniFeedback()
#         robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
#         self.force_pub.publish(force_pub_msg)

#         # Timestamp logging
#         gen_time = time.time()
#         time_msg = Float64()
#         time_msg.data = rospy.get_time()

#         self.last_sent_force_timestamp = time_msg.data
#         self.time_pub.publish(time_msg)
        
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

#     def make_csv(self):
#         """Main force CSV with filtered forces in base frame"""
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

#     def make_csv_for_raw_sensor_frame(self):
#         """Raw force in sensor frame (directly from FT sensor)"""
#         with open('/home/user/Desktop/delay/raw_force_sensor_frame.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Raw Sensor Force X (N)', 'Raw Sensor Force Y (N)', 'Raw Sensor Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot_raw_sensor[i][0],
#                     self.robot_force_plot_raw_sensor[i][1],
#                     self.robot_force_plot_raw_sensor[i][2]
#                 ])

#     def make_csv_for_raw_vs_filtered_base_frame(self):
#         """Raw vs filtered force, both in base frame"""
#         with open('/home/user/Desktop/delay/raw_vs_filtered_base_frame.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Raw Base Force X (N)', 'Raw Base Force Y (N)', 'Raw Base Force Z (N)',
#                 'Filtered Base Force X (N)', 'Filtered Base Force Y (N)', 'Filtered Base Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot_raw_base[i][0],
#                     self.robot_force_plot_raw_base[i][1],
#                     self.robot_force_plot_raw_base[i][2],
#                     self.robot_force_plot[i][0],
#                     self.robot_force_plot[i][1],
#                     self.robot_force_plot[i][2]
#                 ])

#     def make_csv_for_all_forces(self):
#         """Complete CSV with all force types"""
#         with open('/home/user/Desktop/delay/all_forces_complete.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Raw Sensor X (N)', 'Raw Sensor Y (N)', 'Raw Sensor Z (N)',
#                 'Raw Base X (N)', 'Raw Base Y (N)', 'Raw Base Z (N)',
#                 'Filtered Base X (N)', 'Filtered Base Y (N)', 'Filtered Base Z (N)',
#                 'Haptic X (N)', 'Haptic Y (N)', 'Haptic Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot_raw_sensor[i][0],
#                     self.robot_force_plot_raw_sensor[i][1],
#                     self.robot_force_plot_raw_sensor[i][2],
#                     self.robot_force_plot_raw_base[i][0],
#                     self.robot_force_plot_raw_base[i][1],
#                     self.robot_force_plot_raw_base[i][2],
#                     self.robot_force_plot[i][0],
#                     self.robot_force_plot[i][1],
#                     self.robot_force_plot[i][2],
#                     self.haptic_force_plot[i][0],
#                     self.haptic_force_plot[i][1],
#                     self.haptic_force_plot[i][2]
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
        
#         # Generate all CSV files
#         self.make_csv()
#         self.make_csv_for_delay()
#         self.make_csv_for_generation_and_send()
#         self.make_csv_for_raw_sensor_frame()
#         self.make_csv_for_raw_vs_filtered_base_frame()
#         self.make_csv_for_all_forces()
        
#         rospy.loginfo("All CSV files saved.")

        
# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass


####

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

#         self.robot_force_raw_sensor = np.zeros(3)     # raw FT sensor values (sensor frame)
#         self.robot_force_raw_base = np.zeros(3)       # raw force in base frame (no filtering)
#         self.robot_force_plot_raw_sensor = []         # store raw sensor frame values
#         self.robot_force_plot_raw_base = []           # store raw base frame values
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []

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

#     def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#         if self.last_sent_force_timestamp is not None and msg.data == self.last_sent_force_timestamp:
#             round_trip_delay = rospy.get_time() - msg.data
#             print(f"[FORCE DELAY] Round-trip delay: {round_trip_delay:.6f} s")
#             self.delay_log.append(round_trip_delay)
#             self.delay_data_log.append((msg.data, round_trip_delay))

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
        
#         # Log raw sensor frame values
#         self.robot_force_plot_raw_sensor.append(self.robot_force_raw_sensor.copy())
        
#         # Log raw base frame values (no filtering)
#         self.robot_force_plot_raw_base.append(self.robot_force_raw_base.copy())
        
#         # Log filtered base frame values
#         self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))
        
#         # Log haptic force
#         self.haptic_force_plot.append(self.haptic_force.copy())

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
        
#         # Store raw sensor values (DIRECTLY from /ft_wrench topic - NO processing)
#         self.robot_force_raw_sensor = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        
#         # Convert raw to base frame (no filtering, but with frame transformation)
#         self.robot_force_raw_base = self.forcevector_conversion(self.joint_position_robot, self.robot_force_raw_sensor)
        
#         # Apply moving average filter for control
#         self.force_window.append(self.robot_force_raw_sensor)
#         self.robot_force = np.mean(self.force_window, axis=0)

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#         # Prepare and publish haptic force (using filtered force)
#         force_pub_msg = OmniFeedback()
#         robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
#         self.force_pub.publish(force_pub_msg)

#         # Timestamp logging
#         gen_time = time.time()
#         time_msg = Float64()
#         time_msg.data = rospy.get_time()

#         self.last_sent_force_timestamp = time_msg.data
#         self.time_pub.publish(time_msg)
        
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

#     def make_csv(self):
#         """Main force CSV with filtered forces in base frame"""
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

#     def make_csv_for_raw_sensor_frame(self):
#         """Raw force in sensor frame (directly from FT sensor)"""
#         with open('/home/user/Desktop/delay/raw_force_sensor_frame.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Raw Sensor Force X (N)', 'Raw Sensor Force Y (N)', 'Raw Sensor Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot_raw_sensor[i][0],
#                     self.robot_force_plot_raw_sensor[i][1],
#                     self.robot_force_plot_raw_sensor[i][2]
#                 ])

#     def make_csv_for_raw_vs_filtered_base_frame(self):
#         """Raw vs filtered force, both in base frame"""
#         with open('/home/user/Desktop/delay/raw_vs_filtered_base_frame.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Raw Base Force X (N)', 'Raw Base Force Y (N)', 'Raw Base Force Z (N)',
#                 'Filtered Base Force X (N)', 'Filtered Base Force Y (N)', 'Filtered Base Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot_raw_base[i][0],
#                     self.robot_force_plot_raw_base[i][1],
#                     self.robot_force_plot_raw_base[i][2],
#                     self.robot_force_plot[i][0],
#                     self.robot_force_plot[i][1],
#                     self.robot_force_plot[i][2]
#                 ])

#     def make_csv_for_all_forces(self):
#         """Complete CSV with all force types"""
#         with open('/home/user/Desktop/delay/all_forces_complete.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Raw Sensor X (N)', 'Raw Sensor Y (N)', 'Raw Sensor Z (N)',
#                 'Raw Base X (N)', 'Raw Base Y (N)', 'Raw Base Z (N)',
#                 'Filtered Base X (N)', 'Filtered Base Y (N)', 'Filtered Base Z (N)',
#                 'Haptic X (N)', 'Haptic Y (N)', 'Haptic Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot_raw_sensor[i][0],
#                     self.robot_force_plot_raw_sensor[i][1],
#                     self.robot_force_plot_raw_sensor[i][2],
#                     self.robot_force_plot_raw_base[i][0],
#                     self.robot_force_plot_raw_base[i][1],
#                     self.robot_force_plot_raw_base[i][2],
#                     self.robot_force_plot[i][0],
#                     self.robot_force_plot[i][1],
#                     self.robot_force_plot[i][2],
#                     self.haptic_force_plot[i][0],
#                     self.haptic_force_plot[i][1],
#                     self.haptic_force_plot[i][2]
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
        
#         # Generate all CSV files
#         self.make_csv()
#         self.make_csv_for_delay()
#         self.make_csv_for_generation_and_send()
#         self.make_csv_for_raw_sensor_frame()
#         self.make_csv_for_raw_vs_filtered_base_frame()
#         self.make_csv_for_all_forces()
        
#         rospy.loginfo("All CSV files saved.")

        
# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass



#####
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback
from std_msgs.msg import Float64
from collections import deque
import time
import matplotlib.pyplot as plt 
import csv

class HapticForceController:

    def __init__(self):

        # Initializing node
        rospy.init_node('haptic_force_controller', anonymous=True)

        # Publishers
        self.force_pub = rospy.Publisher('force_from_slave_to_master', OmniFeedback, queue_size=1)
        self.time_pub = rospy.Publisher('time_from_slave_to_master_for_force', Float64, queue_size=1)
        self.last_sent_force_timestamp = None

        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        rospy.Subscriber('time_from_master_to_slave_for_force', Float64, self.time_from_master_to_slave_for_force_callback)

        # For initialization parameter
        self.start_time = time.time() 
        self.robot_force = np.zeros(3)
        self.robot_force_initial = np.zeros(3)
        self.joint_position_robot = np.zeros(6)
        self.haptic_force = np.zeros(3)

        self.robot_force_raw_sensor = np.zeros(3)     # raw FT sensor values (sensor frame)
        self.robot_force_raw_base = np.zeros(3)       # raw force in base frame (no filtering)
        self.robot_force_compensated = np.zeros(3)   # after all compensation, before moving average
        self.robot_force_plot_raw_sensor = []         # store raw sensor frame values
        self.robot_force_plot_raw_base = []           # store raw base frame values
        self.robot_force_plot_compensated = []        # store compensated (no filter) values
        
        self.force_sent_timestamps = []
        self.force_gen_send_log = []

        # For moving average filter
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
        self.delay_log = []
        self.delay_data_log = []

    def time_from_master_to_slave_for_force_callback(self, msg: Float64):
        if self.last_sent_force_timestamp is not None and msg.data == self.last_sent_force_timestamp:
            round_trip_delay = rospy.get_time() - msg.data
            print(f"[FORCE DELAY] Round-trip delay: {round_trip_delay:.6f} s")
            self.delay_log.append(round_trip_delay)
            self.delay_data_log.append((msg.data, round_trip_delay))

    def update_list(self):
        if self.shutdown_flag:
            return
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        
        # Log raw sensor frame values
        self.robot_force_plot_raw_sensor.append(self.robot_force_raw_sensor.copy())
        
        # Log compensated force (after all compensation steps, NO moving average)
        self.robot_force_plot_compensated.append(self.robot_force_compensated.copy())
        
        # Log filtered base frame values (with moving average)
        filtered_force, _ = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        self.robot_force_plot.append(filtered_force)
        
        # Log haptic force
        self.haptic_force_plot.append(self.haptic_force.copy())

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
        return (np.dot(R, robot_force)), R  # Return both force and rotation matrix

    def joint_callback_robot(self, msg: JointState):
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

    def robot_force_callback(self, msg: WrenchStamped):
        if time.time() - self.start_time < 1:
            rospy.loginfo("booting")
        
        # Store raw sensor values (DIRECTLY from /ft_wrench topic - NO processing)
        self.robot_force_raw_sensor = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        
        # Convert raw to base frame (after all compensation, but NO moving average)
        self.robot_force_compensated, _ = self.forcevector_conversion(self.joint_position_robot, self.robot_force_raw_sensor)
        
        # Apply moving average filter for control
        self.force_window.append(self.robot_force_raw_sensor)
        self.robot_force = np.mean(self.force_window, axis=0)

    def haptic_force_callback(self, event):
        if self.shutdown_flag:
            return

        # Prepare and publish haptic force (using filtered force)
        force_pub_msg = OmniFeedback()
        robot_force, _ = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
        self.force_pub.publish(force_pub_msg)

        # Timestamp logging
        gen_time = time.time()
        time_msg = Float64()
        time_msg.data = rospy.get_time()

        self.last_sent_force_timestamp = time_msg.data
        self.time_pub.publish(time_msg)
        
        self.force_sent_timestamps.append(time_msg.data)
        diff = time_msg.data - gen_time
        self.force_gen_send_log.append((gen_time, time_msg.data, diff))

        # Store haptic force for plotting/logging
        self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])

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
        """Main force CSV with filtered forces in base frame"""
        with open('/home/user/Desktop/delay/force.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Timestamp Sent to Master', 'Time (s)',
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

    def make_csv_for_delay(self):
        with open('/home/user/Desktop/delay/forced.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp received from master', 'Delay'])
            for entry in self.delay_data_log:
                writer.writerow([entry[0], entry[1]])

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

    def make_csv_for_raw_sensor_frame(self):
        """Raw force in sensor frame (directly from FT sensor)"""
        with open('/home/user/Desktop/delay/raw_force_sensor_frame.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Time (s)',
                'Raw Sensor Force X (N)', 'Raw Sensor Force Y (N)', 'Raw Sensor Force Z (N)'
            ])
            for i in range(len(self.time_stamps)):
                writer.writerow([
                    f"{self.time_stamps[i]:.9f}",
                    self.robot_force_plot_raw_sensor[i][0],
                    self.robot_force_plot_raw_sensor[i][1],
                    self.robot_force_plot_raw_sensor[i][2]
                ])

    def make_csv_for_raw_vs_filtered_base_frame(self):
        """Compensated (no filter) vs filtered force, both in base frame"""
        with open('/home/user/Desktop/delay/compensated_vs_filtered.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Time (s)',
                'Compensated Force X (N)', 'Compensated Force Y (N)', 'Compensated Force Z (N)',
                'Filtered Force X (N)', 'Filtered Force Y (N)', 'Filtered Force Z (N)'
            ])
            for i in range(len(self.time_stamps)):
                writer.writerow([
                    f"{self.time_stamps[i]:.9f}",
                    self.robot_force_plot_compensated[i][0],
                    self.robot_force_plot_compensated[i][1],
                    self.robot_force_plot_compensated[i][2],
                    self.robot_force_plot[i][0],
                    self.robot_force_plot[i][1],
                    self.robot_force_plot[i][2]
                ])

    def make_csv_for_all_forces(self):
        """Complete CSV with all force types"""
        with open('/home/user/Desktop/delay/all_forces_complete.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Time (s)',
                'Raw Sensor X (N)', 'Raw Sensor Y (N)', 'Raw Sensor Z (N)',
                'Compensated X (N)', 'Compensated Y (N)', 'Compensated Z (N)',
                'Filtered X (N)', 'Filtered Y (N)', 'Filtered Z (N)',
                'Haptic X (N)', 'Haptic Y (N)', 'Haptic Z (N)'
            ])
            for i in range(len(self.time_stamps)):
                writer.writerow([
                    f"{self.time_stamps[i]:.9f}",
                    self.robot_force_plot_raw_sensor[i][0],
                    self.robot_force_plot_raw_sensor[i][1],
                    self.robot_force_plot_raw_sensor[i][2],
                    self.robot_force_plot_compensated[i][0],
                    self.robot_force_plot_compensated[i][1],
                    self.robot_force_plot_compensated[i][2],
                    self.robot_force_plot[i][0],
                    self.robot_force_plot[i][1],
                    self.robot_force_plot[i][2],
                    self.haptic_force_plot[i][0],
                    self.haptic_force_plot[i][1],
                    self.haptic_force_plot[i][2]
                ])

    def main_loop(self):
        rate = 500
        rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
        rospy.spin()

    def shutdown_hook(self):
        self.shutdown_flag = True
        zero_force_msg = OmniFeedback()
        zero_force_msg.force.x, zero_force_msg.force.y, zero_force_msg.force.z = (0, 0, 0)
        rospy.loginfo("Sending zero force to haptic device.")
        for _ in range(10):
            self.force_pub.publish(zero_force_msg)
            time.sleep(0.1)
        rospy.loginfo("Shutting down, sent zero force to haptic device.")
        
        # Generate all CSV files
        self.make_csv()
        self.make_csv_for_delay()
        self.make_csv_for_generation_and_send()
        self.make_csv_for_raw_sensor_frame()
        self.make_csv_for_raw_vs_filtered_base_frame()
        self.make_csv_for_all_forces()
        
        rospy.loginfo("All CSV files saved.")

        
if __name__ == "__main__":
    try:
        controller = HapticForceController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass



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

#         self.robot_force_raw = np.zeros(3)     # raw FT sensor values
#         self.robot_force_plot_raw = []         # store raw values for plotting/logging

        
        
#         self.force_sent_timestamps = []   # just sent times
#         self.force_gen_send_log = []      # gen_time, sent_time, diff for force_gen_send.csv

#         # CSV file for end-effector velocities
#         self.csv_file_vel = open('/home/user/Desktop/delay/velocity_data.csv', 'w', newline='')
#         self.csv_writer_vel = csv.writer(self.csv_file_vel)
#         self.csv_writer_vel.writerow(["time", "vx", "vy", "vz"])



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
#           # Stores (timestamp, delay)


#     # def time_from_master_to_slave_for_force_callback(self, msg: Float64):
#     #     received_time = msg.data
#     #     current_time = rospy.get_time()
#     #     #delay = (current_time - received_time) / 2  # One-way delay estimate
#     #     delay = current_time - received_time
#     #     print(f'[DEBUG] Received timestamp: {received_time}, Delay: {delay}')
    
#     #     self.delay_log.append(delay)
#     #     self.delay_data_log.append((received_time, delay))

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
#         # log raw (unfiltered) and filtered values
#         self.robot_force_plot_raw.append(self.robot_force_raw)
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
#         self.joint_velocity_robot = np.array([msg.velocity[2], msg.velocity[1], msg.velocity[0], msg.velocity[3], msg.velocity[4], msg.velocity[5]])


#     def compute_ee_velocity(self):
#         if self.joint_position_robot is None or self.joint_velocity_robot is None:
#             return None
#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)
#         ee_velocity = J_geometrical @ self.joint_velocity_robot  # 6x1 vector
#         return ee_velocity[:3]  # vx, vy, vz

#     def robot_force_callback(self, msg: WrenchStamped):
#         if time.time() - self.start_time < 1:
#             rospy.loginfo("booting")
#         self.robot_force_raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
    
#     # push raw into deque for filtering
#         self.force_window.append(self.robot_force_raw)
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

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#     # Prepare and publish haptic force
#         force_pub_msg = OmniFeedback()
#         robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
#         self.force_pub.publish(force_pub_msg)

#     # --- NEW LOGGING FOR TIMESTAMPS ---
#         gen_time = time.time()                # wall-clock generation time
#         time_msg = Float64()
#         time_msg.data = rospy.get_time()      # ROS time (sent timestamp)

#         self.last_sent_force_timestamp = time_msg.data
#         self.time_pub.publish(time_msg)
        
#     # Store both for CSV logging
#         self.force_sent_timestamps.append(time_msg.data)  # clean float
#         diff = time_msg.data - gen_time
#         self.force_gen_send_log.append((gen_time, time_msg.data, diff))

#     # Store haptic force for plotting/logging
#         self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])

#     # Update plots and lists
#         self.update_list()

#     # --- velocity logging here ---
#         ee_vel = self.compute_ee_velocity()
#         if ee_vel is not None:
#             applied_time = rospy.get_time()
#             self.csv_writer_vel.writerow([applied_time] + list(ee_vel))
#             self.csv_file_vel.flush()

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


#     def make_csv_for_raw_and_filtered(self):
#         with open('/home/user/Desktop/delay/raw_vs_filtered_force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (s)',
#                 'Raw Force X (N)', 'Raw Force Y (N)', 'Raw Force Z (N)',
#                 'Filtered Force X (N)', 'Filtered Force Y (N)', 'Filtered Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 writer.writerow([
#                     f"{self.time_stamps[i]:.9f}",
#                     self.robot_force_plot_raw[i][0], self.robot_force_plot_raw[i][1], self.robot_force_plot_raw[i][2],
#                     self.robot_force_plot[i][0], self.robot_force_plot[i][1], self.robot_force_plot[i][2]
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
#         self.make_csv_for_raw_and_filtered()
#         self.csv_file_vel.close()

        
# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass