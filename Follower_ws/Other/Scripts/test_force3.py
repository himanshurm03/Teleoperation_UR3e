#!/usr/bin/env python3

#Purpose:Implementation of Moving Average and Weber fraction
#working fine
# #moving average(15-30) + Weber Fraction (7%)
#Normal(works perfect)
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

        #Moving Average(15 -30)
        self.haptic_window_size = 30
        self.force_window = deque(maxlen=self.haptic_window_size)

        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.shutdown_flag = False

        # For plotting
        self.start_time = None
        self.time_stamps = []
        self.robot_force_plot = []
        self.haptic_force_plot = []

        #Weber sampling
        self.weber_delta = 0.05
        self.prev_weber_force = None
        
        rospy.loginfo("Force controller initialized - publishing to internal topics")
        rospy.loginfo("Force RTD logging enabled - subscribing to 'master_to_slave_data'")

    # FIXED: Receives data FROM master (MasterToSlaveData type)
    def master_echo_callback(self, msg: MasterToSlaveData):
        
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

    def apply_weber_sampling(self, current_force):
        """
        Weber sampling:
        (Fn - Fn-1) / Fn-1 >= delta
        """
        if self.prev_weber_force is None:
            self.prev_weber_force = current_force
            return current_force

        Fn = current_force
        Fp = self.prev_weber_force

    # Avoid division by zero
        eps = 1e-6
        ratio = np.linalg.norm(Fn - Fp) / (np.linalg.norm(Fp) + eps)

        if ratio >= self.weber_delta:
            self.prev_weber_force = Fn
            return Fn
        else:
            return Fp


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
        #robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        robot_force_ma = self.forcevector_conversion(
            self.joint_position_robot,
            self.robot_force
        )

        # Weber sampling AFTER moving average
        robot_force = self.apply_weber_sampling(robot_force_ma)
        
        # Create force message (Vector3) * 0.5
        force_msg = Vector3()
        force_msg.x = robot_force[0] *0
        force_msg.y = robot_force[1] *0
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



################################################################################################
#fixing the issue with the delay 

# import rospy
# import numpy as np
# from geometry_msgs.msg import WrenchStamped, Vector3
# from sensor_msgs.msg import JointState
# from omni_msgs.msg import OmniFeedback
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData  # FIXED: Changed from SlaveToMasterData
# from collections import deque
# import time
# import matplotlib.pyplot as plt 
# import csv

# class HapticForceController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('haptic_force_controller', anonymous=True)

#         # Publishers - Publishing to internal topics (for pose code to bundle)
#         self.internal_force_pub = rospy.Publisher('internal_force_data', Vector3, queue_size=1)
#         self.internal_force_timestamp_pub = rospy.Publisher('internal_force_timestamp', Float64, queue_size=1)
        
#         self.last_sent_force_timestamp = None

#         # Subscribers
#         rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # FIXED: Subscribe to data coming FROM master (master_to_slave_data, not slave_to_master_data)
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_echo_callback)

#         # For initialization parameter
#         self.boot_time = time.time()  
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_force = np.zeros(3)
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}
        
#         # For RTD logging
#         self.force_rtd_log = []  # Stores (timestamp, RTD)

#         #Moving Average(15 -30)
#         self.haptic_window_size = 30
#         self.force_window = deque(maxlen=self.haptic_window_size)

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.robot_force_plot = []
#         self.haptic_force_plot = []

#         #Weber sampling
#         self.weber_delta = 0.05
#         self.prev_weber_force = None
        
#         rospy.loginfo("Force controller initialized - publishing to internal topics")
#         rospy.loginfo("Force RTD logging enabled - subscribing to 'master_to_slave_data'")

#     # FIXED: Receives data FROM master (MasterToSlaveData type)
#     def master_echo_callback(self, msg: MasterToSlaveData):
        
#         force_timestamp_echo = msg.force_timestamp
        
#         if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
#             # Calculate RTD
#             current_time = rospy.get_time()
#             rtd = current_time - force_timestamp_echo
            
#             # Log it
#             self.force_rtd_log.append((force_timestamp_echo, rtd))
            
#             rospy.loginfo_throttle(1.0, f"Force RTD: {rtd*1000:.3f}ms")
            
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

#     def apply_weber_sampling(self, current_force):
#         """
#         Weber sampling:
#         (Fn - Fn-1) / Fn-1 >= delta
#         """
#         if self.prev_weber_force is None:
#             self.prev_weber_force = current_force
#             return current_force

#         Fn = current_force
#         Fp = self.prev_weber_force

#     # Avoid division by zero
#         eps = 1e-6
#         ratio = np.linalg.norm(Fn - Fp) / (np.linalg.norm(Fp) + eps)

#         if ratio >= self.weber_delta:
#             self.prev_weber_force = Fn
#             return Fn
#         else:
#             return Fp


#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

#     def robot_force_callback(self, msg: WrenchStamped):
#         if time.time() - self.boot_time < 1:
#             rospy.loginfo("booting")
#         robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
#         self.force_window.append(robot_force)
#         self.robot_force = np.mean(self.force_window, axis=0)

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#         # Calculate haptic force
#         #robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         robot_force_ma = self.forcevector_conversion(
#             self.joint_position_robot,
#             self.robot_force
#         )

#         # Weber sampling AFTER moving average
#         robot_force = self.apply_weber_sampling(robot_force_ma)
        
#         # Create force message (Vector3) * 0.5
#         force_msg = Vector3()
#         force_msg.x = robot_force[0] *0
#         force_msg.y = robot_force[1] *0
#         force_msg.z = robot_force[2] 
        
#         # Publish to internal topic (pose code will bundle this)
#         self.internal_force_pub.publish(force_msg)

#         # Publish force timestamp to internal topic
#         force_timestamp = rospy.get_time()
#         self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

#         # Store timestamp for RTD calculation
#         self.sent_force_times[force_timestamp] = force_timestamp  # ROS time
#         gen_time = rospy.get_time()
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

#     def make_csv_for_force_rtd(self):
#         """Save force round-trip delay to CSV with statistics"""
#         with open('/home/user/Desktop/delay/force_rtd.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Force Timestamp', 'Force RTD (s)'])
#             for timestamp, rtd in self.force_rtd_log:
#                 writer.writerow([f"{timestamp:.9f}", f"{rtd:.9f}"])
        
#         # Calculate and print statistics
#         if self.force_rtd_log:
#             rtds = [rtd for _, rtd in self.force_rtd_log]
#             rospy.loginfo("="*60)
#             rospy.loginfo("Force RTD Statistics:")
#             rospy.loginfo(f"  Average: {np.mean(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Min:     {np.min(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Max:     {np.max(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Std Dev: {np.std(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Total samples: {len(rtds)}")
#             rospy.loginfo("="*60)
        
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
#         self.make_csv_for_force_rtd()

# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass



############################################################################################################
#The purpose of this below code is too check The force is smooth or not
#And we have implemented this by creating an plane at height of 0.08m and above the plane we are getting
#ON of force and on the surface and below thesurface we are getting 2N of force implemented using:F=k*delta x
#F=k*delta x
#Spring

# import rospy
# import numpy as np
# from geometry_msgs.msg import Vector3
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData
# from scipy.spatial.transform import Rotation as R
# import time
# import csv

# class VirtualPlaneHapticController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('virtual_plane_force_controller', anonymous=True)

#         # Publishers - Publishing to internal topics (for pose code to bundle)
#         self.internal_force_pub = rospy.Publisher('internal_force_data', Vector3, queue_size=1)
#         self.internal_force_timestamp_pub = rospy.Publisher('internal_force_timestamp', Float64, queue_size=1)
        
#         # Subscribers
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot, queue_size=1)
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_echo_callback, queue_size=1)

#         # Virtual plane parameters
#         self.plane_z_height = 0.1628  # Height of the virtual plane in meters (ADJUST THIS!)
#         self.stiffness = 100.0  # Spring stiffness k (N/m) - for calculating penetration limit
#         self.constant_force = 2.0  # Constant force when in contact (N)
#         self.penetration_threshold = self.constant_force / self.stiffness  # delta_x at which force reaches 2N
        
#         # Robot state
#         self.start_time = time.time() 
#         self.joint_position_robot = np.zeros(6)
#         self.current_ee_pose = np.zeros((6, 1))
#         self.haptic_force = np.zeros(3)
#         self.end_effector_position = np.zeros(3)
        
#         # For unwrapping angles (matching pose code)
#         self.robot_previous_angles = None
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}
        
#         # For RTD logging
#         self.force_rtd_log = []

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For CSV logging
#         self.csv_log = []  # Stores (time, x, y, z, fx, fy, fz, penetration, is_contact)
        
#         rospy.loginfo("="*60)
#         rospy.loginfo("Virtual Plane Haptic Controller initialized")
#         rospy.loginfo(f"Plane height: {self.plane_z_height}m")
#         rospy.loginfo(f"Stiffness k: {self.stiffness} N/m")
#         rospy.loginfo(f"Constant force: {self.constant_force}N")
#         rospy.loginfo(f"Penetration threshold (F=2N): {self.penetration_threshold*1000:.2f}mm")
#         rospy.loginfo("Force model: F = k*Δx until 2N, then constant 2N")
#         rospy.loginfo("Publishing to: internal_force_data, internal_force_timestamp")
#         rospy.loginfo("="*60)

#     def master_echo_callback(self, msg: MasterToSlaveData):
#         """RTD calculation for force data"""
#         force_timestamp_echo = msg.force_timestamp
        
#         if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
#             current_time = rospy.get_time()
#             rtd = current_time - force_timestamp_echo
#             self.force_rtd_log.append((force_timestamp_echo, rtd))
#             rospy.loginfo_throttle(2.0, f"Force RTD: {rtd*1000:.3f}ms")
#             del self.sent_force_times[force_timestamp_echo]

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         """Angle unwrapping - matching pose code"""
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     def pose_and_rotation(self, joint_position_robot):
#         """
#         EXACT COPY of forward kinematics from your pose code
#         This ensures both codes use identical position calculations
#         """
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles
        
#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])

#     def calculate_virtual_plane_force(self, position):
#         """
#         Hybrid force model:
#         - If z > plane_z_height: F = 0N (no force above plane)
#         - If z <= plane_z_height:
#             * Use F = k * delta_x while F < 2N (proportional region)
#             * Use F = 2N constant once penetration reaches threshold (constant region)
        
#         Where:
#         - k = stiffness (N/m)
#         - delta_x = penetration depth = (plane_z_height - z_position)
#         - threshold = 2N / k (penetration at which force reaches 2N)
#         """
#         z_pos = position[2]
        
#         # Calculate penetration depth (delta_x)
#         penetration = self.plane_z_height - z_pos
        
#         if penetration <= 0:
#             # Above the plane - no force
#             force = np.zeros(3)
#             is_contact = False
#             force_magnitude = 0.0
#         else:
#             # Below the plane - apply force model
#             if penetration < self.penetration_threshold:
#                 # Proportional region: F = k * delta_x
#                 force_magnitude = self.stiffness * penetration
#             else:
#                 # Constant region: F = 2N
#                 force_magnitude = self.constant_force
            
#             # Force is upward (opposing penetration)
#             force = np.array([0, 0, force_magnitude])
#             is_contact = True
        
#         return force, is_contact, penetration

#     def joint_callback_robot(self, msg: JointState):
#         """Update joint positions and calculate end effector pose"""
#         self.joint_position_robot = np.array([
#             msg.position[2], msg.position[1], msg.position[0], 
#             msg.position[3], msg.position[4], msg.position[5]
#         ])
        
#         # Calculate full pose using exact FK from pose code
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
        
#         # Extract position (first 3 elements) - direct, no filtering
#         self.end_effector_position = self.current_ee_pose[0:3, 0]

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#         current_time = rospy.get_time()
        
#         # Calculate force based on simple virtual plane model
#         virtual_force, is_contact, penetration = self.calculate_virtual_plane_force(
#             self.end_effector_position
#         )
        
#         # Create force message
#         force_msg = Vector3()
#         force_msg.x = virtual_force[0]
#         force_msg.y = virtual_force[1]
#         force_msg.z = virtual_force[2]
        
#         # Publish to internal topic (pose code will pick this up)
#         self.internal_force_pub.publish(force_msg)

#         # Publish force timestamp
#         force_timestamp = current_time
#         self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

#         # Store timestamp for RTD calculation
#         self.sent_force_times[force_timestamp] = time.time()

#         # Logging for generation timing
#         gen_time = time.time()
#         self.force_sent_timestamps.append(force_timestamp)
#         diff = force_timestamp - gen_time
#         self.force_gen_send_log.append((gen_time, force_timestamp, diff))

#         # Store haptic force
#         self.haptic_force = virtual_force
        
#         # Log to CSV buffer
#         self.csv_log.append((
#             current_time,
#             self.end_effector_position[0],
#             self.end_effector_position[1],
#             self.end_effector_position[2],
#             virtual_force[0],
#             virtual_force[1],
#             virtual_force[2],
#             penetration if is_contact else 0.0,
#             1 if is_contact else 0
#         ))
        
#         # Log status periodically
#         if len(self.csv_log) % 500 == 0:  # Every ~1 second at 500Hz
#             z_pos = self.end_effector_position[2]
#             state = "CONTACT" if is_contact else "FREE"
#             force_mag = np.linalg.norm(virtual_force)
#             pen_val = penetration if is_contact else 0.0
            
#             # Determine which region we're in
#             if is_contact:
#                 if pen_val < self.penetration_threshold:
#                     region = f"SPRING (F=k*Δx={self.stiffness}*{pen_val:.4f})"
#                 else:
#                     region = "CONSTANT (F=2N)"
#             else:
#                 region = "FREE"
            
#             rospy.loginfo(
#                 f"Z: {z_pos:.4f}m | {state} | Force: {force_mag:.3f}N | "
#                 f"Pen: {pen_val*1000:.2f}mm | {region}"
#             )

#     def make_csv_force_interaction(self):
#         """Save complete force interaction data to CSV"""
#         with open('/home/user/Desktop/delay/virtual_plane_interaction.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (ROS)', 
#                 'Position X (m)', 'Position Y (m)', 'Position Z (m)',
#                 'Force X (N)', 'Force Y (N)', 'Force Z (N)',
#                 'Penetration (m)', 'Contact (0/1)'
#             ])
            
#             for row in self.csv_log:
#                 writer.writerow([
#                     f"{row[0]:.9f}",  # Time
#                     f"{row[1]:.6f}", f"{row[2]:.6f}", f"{row[3]:.6f}",  # Position
#                     f"{row[4]:.6f}", f"{row[5]:.6f}", f"{row[6]:.6f}",  # Force
#                     f"{row[7]:.6f}",  # Penetration
#                     int(row[8])       # Contact
#                 ])
        
#         rospy.loginfo("="*60)
#         rospy.loginfo("Interaction CSV saved to: /home/user/Desktop/delay/virtual_plane_interaction.csv")
        
#         # Calculate and print statistics
#         if self.csv_log:
#             contact_events = sum(1 for row in self.csv_log if row[8] == 1)
#             max_penetration = max(row[7] for row in self.csv_log)
#             max_force = max(np.linalg.norm([row[4], row[5], row[6]]) for row in self.csv_log)
            
#             rospy.loginfo(f"Total samples: {len(self.csv_log)}")
#             rospy.loginfo(f"Contact samples: {contact_events} ({100*contact_events/len(self.csv_log):.1f}%)")
#             rospy.loginfo(f"Max penetration: {max_penetration*1000:.2f} mm")
#             rospy.loginfo(f"Max force: {max_force:.2f} N")
#         rospy.loginfo("="*60)

#     def make_csv_for_generation_and_send(self):
#         """Save force generation timing data"""
#         with open('/home/user/Desktop/delay/force_gen_send.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Generation Time (s)', 'Sent Timestamp (ROS Time)', 'Difference (s)'])
#             for gen_time, sent_time, diff in self.force_gen_send_log:
#                 writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
        
#         rospy.loginfo("Force timing CSV saved to: /home/user/Desktop/delay/force_gen_send.csv")

#     def make_csv_for_force_rtd(self):
#         """Save force round-trip delay statistics"""
#         with open('/home/user/Desktop/delay/force_rtd.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Force Timestamp', 'Force RTD (s)', 'Force RTD (ms)'])
#             for timestamp, rtd in self.force_rtd_log:
#                 writer.writerow([f"{timestamp:.9f}", f"{rtd:.9f}", f"{rtd*1000:.3f}"])
        
#         if self.force_rtd_log:
#             rtds = [rtd for _, rtd in self.force_rtd_log]
#             rospy.loginfo("="*60)
#             rospy.loginfo("Force RTD Statistics:")
#             rospy.loginfo(f"  Average: {np.mean(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Min:     {np.min(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Max:     {np.max(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Std Dev: {np.std(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Total samples: {len(rtds)}")
#             rospy.loginfo("="*60)
        
#         rospy.loginfo("Force RTD CSV saved to: /home/user/Desktop/delay/force_rtd.csv")

#     def main_loop(self):
#         rate = 500  # Match your system's control rate
#         rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
#         rospy.loginfo("Force control loop started at 500Hz")
#         rospy.loginfo(f"Force model: F = k*Δx (k={self.stiffness}N/m) until {self.constant_force}N, then constant {self.constant_force}N")
#         rospy.spin()

#     def shutdown_hook(self):
#         self.shutdown_flag = True
#         zero_force_msg = Vector3()
#         zero_force_msg.x, zero_force_msg.y, zero_force_msg.z = (0, 0, 0)
#         rospy.loginfo("Sending zero force to internal topic.")
#         for _ in range(10):
#             self.internal_force_pub.publish(zero_force_msg)
#             time.sleep(0.01)
#         rospy.loginfo("Shutting down, sent zero force.")
        
#         # Save all data
#         try:
#             self.make_csv_force_interaction()
#             self.make_csv_for_generation_and_send()
#             self.make_csv_for_force_rtd()
#         except Exception as e:
#             rospy.logerr(f"Error saving data: {e}")

# if __name__ == "__main__":
#     try:
#         controller = VirtualPlaneHapticController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass


##########################################################################################
#This is just a trial code
# import rospy
# import numpy as np
# from geometry_msgs.msg import WrenchStamped, Vector3
# from sensor_msgs.msg import JointState
# from omni_msgs.msg import OmniFeedback
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData
# from collections import deque
# import time
# import matplotlib.pyplot as plt 
# import csv

# class HapticForceController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('haptic_force_controller', anonymous=True)

#         # Publishers - Publishing to internal topics (for pose code to bundle)
#         self.internal_force_pub = rospy.Publisher('internal_force_data', Vector3, queue_size=1)
#         self.internal_force_timestamp_pub = rospy.Publisher('internal_force_timestamp', Float64, queue_size=1)
        
#         self.last_sent_force_timestamp = None

#         # Subscribers
#         rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_echo_callback)

#         # For initialization parameter
#         self.start_time = time.time() 
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_force = np.zeros(3)
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}
        
#         # For RTD logging
#         self.force_rtd_log = []

#         # DUAL-STAGE FILTERING
#         # Stage 1: Moving average for initial noise reduction
#         self.haptic_window_size = 20  # Balanced between 15 and 30
#         self.force_window = deque(maxlen=self.haptic_window_size)
        
#         # Stage 2: Low-pass filter for smoothing without lag
#         self.use_lowpass_filter = True
#         self.lowpass_alpha = 0.15  # Lower = smoother (was 0.3, too harsh)
#         self.filtered_force = None

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.robot_force_plot = []
#         self.haptic_force_plot = []

#         # MODIFIED Weber sampling - less aggressive
#         self.use_weber_sampling = True
#         self.weber_delta = 0.05  # Moderate threshold (between 0.04 and 0.07)
#         self.prev_weber_force = None
        
#         # Force magnitude threshold - ignore tiny forces (sensor noise)
#         self.force_deadband = 0.15  # Newtons - ignore forces below this
        
#         # Force rate limiter - prevent sudden spikes
#         self.use_rate_limiter = True
#         self.max_force_change_rate = 5.0  # N/s max change (tune this!)
#         self.prev_limited_force = None
#         self.prev_time = None
        
#         rospy.loginfo("Force controller initialized - SMOOTH FORCE MODE")
#         rospy.loginfo(f"Moving average: {self.haptic_window_size} samples")
#         rospy.loginfo(f"Low-pass alpha: {self.lowpass_alpha}")
#         rospy.loginfo(f"Weber delta: {self.weber_delta}")
#         rospy.loginfo(f"Force deadband: {self.force_deadband} N")
#         rospy.loginfo(f"Max force rate: {self.max_force_change_rate} N/s")

#     def master_echo_callback(self, msg: MasterToSlaveData):
#         force_timestamp_echo = msg.force_timestamp
        
#         if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
#             current_time = rospy.get_time()
#             rtd = current_time - force_timestamp_echo
#             self.force_rtd_log.append((force_timestamp_echo, rtd))
#             rospy.loginfo_throttle(1.0, f"Force RTD: {rtd*1000:.3f}ms")
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

#     def apply_deadband(self, force):
#         """
#         Apply deadband to remove sensor noise at low forces
#         """
#         force_magnitude = np.linalg.norm(force)
        
#         if force_magnitude < self.force_deadband:
#             return np.zeros(3)
        
#         return force

#     def apply_rate_limiter(self, current_force, dt):
#         """
#         Limit how fast force can change to prevent sudden spikes
#         This is KEY for preventing hard/harsh forces
#         """
#         if not self.use_rate_limiter:
#             return current_force
            
#         if self.prev_limited_force is None:
#             self.prev_limited_force = current_force
#             return current_force
        
#         # Calculate desired change
#         force_diff = current_force - self.prev_limited_force
#         force_diff_magnitude = np.linalg.norm(force_diff)
        
#         # Maximum allowed change in this timestep
#         max_change = self.max_force_change_rate * dt
        
#         # Limit the change if necessary
#         if force_diff_magnitude > max_change:
#             # Scale down the change
#             limited_diff = force_diff * (max_change / force_diff_magnitude)
#             limited_force = self.prev_limited_force + limited_diff
#         else:
#             limited_force = current_force
        
#         self.prev_limited_force = limited_force
#         return limited_force

#     def lowpass_filter(self, new_force):
#         """
#         Low-pass filter using exponential moving average
#         Lower alpha = smoother but slower response
#         """
#         if self.filtered_force is None:
#             self.filtered_force = new_force
#             return new_force
        
#         # Exponential moving average
#         self.filtered_force = self.lowpass_alpha * new_force + (1 - self.lowpass_alpha) * self.filtered_force
#         return self.filtered_force

#     def apply_weber_sampling(self, current_force):
#         """
#         Standard Weber sampling with moderate threshold
#         """
#         if not self.use_weber_sampling:
#             return current_force
            
#         if self.prev_weber_force is None:
#             self.prev_weber_force = current_force
#             return current_force

#         Fn = current_force
#         Fp = self.prev_weber_force

#         eps = 1e-6
#         ratio = np.linalg.norm(Fn - Fp) / (np.linalg.norm(Fp) + eps)

#         if ratio >= self.weber_delta:
#             self.prev_weber_force = Fn
#             return Fn
#         else:
#             return Fp

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], 
#                                                msg.position[3], msg.position[4], msg.position[5]])

#     def robot_force_callback(self, msg: WrenchStamped):
#         if time.time() - self.start_time < 1:
#             rospy.loginfo("booting")
#             return
            
#         robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        
#         # Stage 1: Moving average (reduces high-frequency noise)
#         self.force_window.append(robot_force)
#         self.robot_force = np.mean(self.force_window, axis=0)

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#         # Calculate time step for rate limiter
#         current_time = rospy.get_time()
#         if self.prev_time is None:
#             dt = 1.0 / 500.0  # Assume 500Hz
#         else:
#             dt = max(current_time - self.prev_time, 1e-6)  # Prevent division by zero
#         self.prev_time = current_time

#         # Transform to base frame
#         robot_force_transformed = self.forcevector_conversion(
#             self.joint_position_robot,
#             self.robot_force
#         )

#         # FILTERING PIPELINE:
#         # 1. Apply deadband (remove sensor noise)
#         force_step1 = self.apply_deadband(robot_force_transformed)
        
#         # 2. Apply low-pass filter (smooth out remaining noise)
#         if self.use_lowpass_filter:
#             force_step2 = self.lowpass_filter(force_step1)
#         else:
#             force_step2 = force_step1
        
#         # 3. Apply rate limiter (prevent sudden spikes) - THIS IS KEY!
#         force_step3 = self.apply_rate_limiter(force_step2, dt)
        
#         # 4. Apply Weber sampling (reduce update rate)
#         final_force = self.apply_weber_sampling(force_step3)
        
#         # Create force message
#         force_msg = Vector3()
#         force_msg.x = final_force[0]
#         force_msg.y = final_force[1]
#         force_msg.z = final_force[2]
        
#         # Publish to internal topic
#         self.internal_force_pub.publish(force_msg)

#         # Publish force timestamp
#         force_timestamp = rospy.get_time()
#         self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

#         # Store timestamp for RTD calculation
#         self.sent_force_times[force_timestamp] = time.time()

#         # Logging
#         gen_time = time.time()
#         self.force_sent_timestamps.append(force_timestamp)
#         diff = force_timestamp - gen_time
#         self.force_gen_send_log.append((gen_time, force_timestamp, diff))

#         # Store haptic force for plotting/logging
#         self.haptic_force = final_force

#         # Update plots and lists
#         self.update_list()    

#     def plot_data(self):
#         times = np.array(self.time_stamps)
#         robot_force_plot = np.array(self.robot_force_plot)
#         haptic_force_plot = np.array(self.haptic_force_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, robot_force_plot[:, 0], label='robot force', alpha=0.7)
#         axs[0, 0].plot(times, haptic_force_plot[:, 0], label='haptic force', linewidth=2)
#         axs[0, 0].set_title('Force in x')
#         axs[0, 0].set_ylabel('Newtons')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)
#         axs[0, 0].legend()

#         axs[1, 0].plot(times, robot_force_plot[:, 1], label='robot force', alpha=0.7)
#         axs[1, 0].plot(times, haptic_force_plot[:, 1], label='haptic force', linewidth=2)
#         axs[1, 0].set_title('Force in y')
#         axs[1, 0].set_ylabel('Newtons')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)
#         axs[1, 0].legend()

#         axs[2, 0].plot(times, robot_force_plot[:, 2], label='robot force', alpha=0.7)
#         axs[2, 0].plot(times, haptic_force_plot[:, 2], label='haptic force', linewidth=2)
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

#     def make_csv_for_force_rtd(self):
#         """Save force round-trip delay to CSV with statistics"""
#         with open('/home/user/Desktop/delay/force_rtd.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Force Timestamp', 'Force RTD (s)'])
#             for timestamp, rtd in self.force_rtd_log:
#                 writer.writerow([f"{timestamp:.9f}", f"{rtd:.9f}"])
        
#         if self.force_rtd_log:
#             rtds = [rtd for _, rtd in self.force_rtd_log]
#             rospy.loginfo("="*60)
#             rospy.loginfo("Force RTD Statistics:")
#             rospy.loginfo(f"  Average: {np.mean(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Min:     {np.min(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Max:     {np.max(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Std Dev: {np.std(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Total samples: {len(rtds)}")
#             rospy.loginfo("="*60)
        
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
        
#         self.make_csv()
#         self.make_csv_for_generation_and_send()
#         self.make_csv_for_force_rtd()

# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass


###############################################################################################

#interaction detection then apply F=kx for 10 sec.


# import rospy
# import numpy as np
# from geometry_msgs.msg import WrenchStamped, Vector3
# from sensor_msgs.msg import JointState
# from omni_msgs.msg import OmniFeedback
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData  # FIXED: Changed from SlaveToMasterData
# from collections import deque
# import time
# import matplotlib.pyplot as plt 
# import csv

# class HapticForceController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('haptic_force_controller', anonymous=True)

#         # Publishers - Publishing to internal topics (for pose code to bundle)
#         self.internal_force_pub = rospy.Publisher('internal_force_data', Vector3, queue_size=1)
#         self.internal_force_timestamp_pub = rospy.Publisher('internal_force_timestamp', Float64, queue_size=1)
        
#         self.last_sent_force_timestamp = None

#         # Subscribers
#         rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # FIXED: Subscribe to data coming FROM master (master_to_slave_data, not slave_to_master_data)
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_echo_callback)

#         # For initialization parameter
#         self.init_start_time = time.time()  # Renamed to avoid conflict
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_force = np.zeros(3)
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}
        
#         # For RTD logging
#         self.force_rtd_log = []  # Stores (timestamp, RTD)
        
#         # For spring interaction logging
#         self.spring_interaction_log = []  # Stores (start_time, end_time, start_pos, trigger_force)

#         #Moving Average(15 -30)
#         self.haptic_window_size = 30
#         self.force_window = deque(maxlen=self.haptic_window_size)

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For plotting
#         self.start_time = None  # Used for relative time plotting
#         self.time_stamps = []
#         self.robot_force_plot = []
#         self.haptic_force_plot = []

#         #Weber sampling
#         self.weber_delta = 0.05
#         self.prev_weber_force = None
        
#         # NEW: Spring interaction mode
#         self.interaction_threshold = 1.0  # 1N threshold in any direction
#         self.spring_constant = 150.0  # Spring constant k (N/m) 
#         self.spring_duration = 10.0  # Maximum duration in seconds
        
#         self.in_spring_mode = False
#         self.spring_mode_start_time = None
#         self.spring_start_position = None

#         self.spring_start_force = None
#         self.baseline_force = None
        
#         rospy.loginfo("Force controller initialized - publishing to internal topics")
#         rospy.loginfo("Force RTD logging enabled - subscribing to 'master_to_slave_data'")
#         rospy.loginfo(f"Spring interaction mode: k={self.spring_constant} N/m, duration={self.spring_duration}s, threshold={self.interaction_threshold}N")

#     # FIXED: Receives data FROM master (MasterToSlaveData type)
#     def master_echo_callback(self, msg: MasterToSlaveData):
        
#         force_timestamp_echo = msg.force_timestamp
        
#         if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
#             # Calculate RTD
#             current_time = rospy.get_time()
#             rtd = current_time - force_timestamp_echo
            
#             # Log it
#             self.force_rtd_log.append((force_timestamp_echo, rtd))
            
#             rospy.loginfo_throttle(1.0, f"Force RTD: {rtd*1000:.3f}ms")
            
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

#     def apply_weber_sampling(self, current_force):
#         """
#         Weber sampling:
#         (Fn - Fn-1) / Fn-1 >= delta
#         """
#         if self.prev_weber_force is None:
#             self.prev_weber_force = current_force
#             return current_force

#         Fn = current_force
#         Fp = self.prev_weber_force

#         # Avoid division by zero
#         eps = 1e-6
#         ratio = np.linalg.norm(Fn - Fp) / (np.linalg.norm(Fp) + eps)

#         if ratio >= self.weber_delta:
#             self.prev_weber_force = Fn
#             return Fn
#         else:
#             return Fp

#     def check_interaction(self, robot_force_base):
#         """
#         Check if there is ACTIVE interaction (contact with object).
#         Returns True if currently in contact (force exceeds threshold).
#         """
#         # Establish baseline force (average of first few readings when in free air)
#         if self.baseline_force is None:
#             if time.time() - self.init_start_time > 2.0:  # After 2 seconds of initialization
#                 self.baseline_force = robot_force_base.copy()
#                 rospy.loginfo(f"Baseline force established: {self.baseline_force}")
#             return False
        
#         # Calculate force deviation from baseline
#         force_deviation = robot_force_base - self.baseline_force
        
#         # Check if any component exceeds threshold (currently in contact)
#         if np.any(np.abs(force_deviation) > self.interaction_threshold):
#             return True  # Active contact detected
        
#         return False  # No contact (in air)

#     def calculate_spring_force(self, current_position):
#         """
#         Calculate F = k*delta_x spring force based on displacement from spring start position.
#         delta_x = current_position - spring_start_position
#         """
#         delta_x = current_position - self.spring_start_position
#         spring_force = -self.spring_constant * delta_x  # F = k * delta_x (positive displacement = positive force)
#         return spring_force

#     def get_end_effector_position(self):
#         """
#         Get the current end-effector position from joint states using forward kinematics.
#         Returns only XYZ position (not orientation).
#         """
#         q1, q2, q3, q4, q5, q6 = self.joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         return np.array([x, y, z])

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

#     def robot_force_callback(self, msg: WrenchStamped):
#         if time.time() - self.init_start_time < 1:
#             rospy.loginfo("booting")
#         robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
#         self.force_window.append(robot_force)
#         self.robot_force = np.mean(self.force_window, axis=0)

#     def haptic_force_callback(self, event):
#         if self.shutdown_flag:
#             return

#         # Calculate robot force in base frame
#         robot_force_ma = self.forcevector_conversion(
#             self.joint_position_robot,
#             self.robot_force
#         )

#         # ===== NEW: Interaction detection and spring mode =====
#         current_time = rospy.get_time()
        
#         # Check if currently in contact with object
#         is_in_contact = self.check_interaction(robot_force_ma)
        
#         if is_in_contact:
#             # ACTIVE CONTACT - Apply spring mode
            
#             if not self.in_spring_mode:
#                 # First contact - enter spring mode
#                 self.in_spring_mode = True
#                 self.spring_mode_start_time = current_time
#                 self.spring_start_position = self.get_end_effector_position()
#                 self.spring_start_force = robot_force_ma.copy()
                
#                 rospy.logwarn(f"CONTACT DETECTED! Entering spring mode F=k*delta_x (k={self.spring_constant} N/m)")
#                 rospy.loginfo(f"Start position: {self.spring_start_position}")
#                 rospy.loginfo(f"Contact force: {robot_force_ma}")
                
#                 # Log the interaction start
#                 self.spring_interaction_log.append({
#                     'start_time': current_time,
#                     'start_position': self.spring_start_position.copy(),
#                     'trigger_force': robot_force_ma.copy()
#                 })
            
#             # Check if we've exceeded max duration
#             elapsed_time = current_time - self.spring_mode_start_time
#             if elapsed_time >= self.spring_duration:
#                 # Max duration reached - exit spring mode and return to normal
#                 rospy.logwarn(f"Spring mode MAX DURATION ({self.spring_duration}s) reached - returning to normal")
                
#                 # Log the interaction end
#                 if self.spring_interaction_log:
#                     self.spring_interaction_log[-1]['end_time'] = current_time
#                     self.spring_interaction_log[-1]['end_position'] = self.get_end_effector_position().copy()
#                     self.spring_interaction_log[-1]['reason'] = 'max_duration'
                
#                 self.in_spring_mode = False
#                 self.spring_mode_start_time = None
#                 self.spring_start_position = None
#                 self.prev_weber_force = None  #added
#                 # Use normal force feedback
#                 robot_force = self.apply_weber_sampling(robot_force_ma)
#             else:
#                 # Still in contact and within time limit - apply F = k*delta_x
#                 current_position = self.get_end_effector_position()
#                 spring_force = self.calculate_spring_force(current_position)
                
#                 robot_force = spring_force
                
#                 # Debug: Calculate displacement
#                 displacement = current_position - self.spring_start_position
#                 disp_magnitude = np.linalg.norm(displacement)
#                 force_magnitude = np.linalg.norm(spring_force)
                
#                 rospy.loginfo_throttle(0.5, 
#                     f"Spring mode - Time: {elapsed_time:.1f}s/{self.spring_duration}s | "
#                     f"Disp: {disp_magnitude*1000:.1f}mm | "
#                     f"Force: {force_magnitude:.2f}N [{spring_force[0]:.2f}, {spring_force[1]:.2f}, {spring_force[2]:.2f}]")
        
#         else:
#             # NO CONTACT (in air) - Normal mode
            
#             if self.in_spring_mode:
#                 # Was in spring mode but contact lost - exit spring mode
#                 elapsed_time = current_time - self.spring_mode_start_time
#                 rospy.loginfo(f"Contact LOST after {elapsed_time:.1f}s - returning to normal mode")
                
#                 # Log the interaction end
#                 if self.spring_interaction_log:
#                     self.spring_interaction_log[-1]['end_time'] = current_time
#                     self.spring_interaction_log[-1]['end_position'] = self.get_end_effector_position().copy()
#                     self.spring_interaction_log[-1]['reason'] = 'contact_lost'
                
#                 self.in_spring_mode = False
#                 self.spring_mode_start_time = None
#                 self.spring_start_position = None
#                 self.prev_weber_force = None #added
            
#             # Normal force feedback with Weber sampling (for when in air)
#             robot_force = self.apply_weber_sampling(robot_force_ma)
        
#         # Create force message - NO SCALING (as requested)
#         force_msg = Vector3()
#         force_msg.x = robot_force[0]
#         force_msg.y = robot_force[1]
#         force_msg.z = robot_force[2]
        
#         # Debug: Log what's being sent
#         sent_magnitude = np.linalg.norm([force_msg.x, force_msg.y, force_msg.z])
#         if self.in_spring_mode:
#             rospy.loginfo_throttle(1.0, 
#                 f"SENDING to haptic: [{force_msg.x:.3f}, {force_msg.y:.3f}, {force_msg.z:.3f}]N (mag: {sent_magnitude:.3f}N)")
        
#         # Publish to internal topic (pose code will bundle this)
#         self.internal_force_pub.publish(force_msg)

#         # Publish force timestamp to internal topic
#         force_timestamp = rospy.get_time()
#         self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

#         # Store timestamp for RTD calculation
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

#     def make_csv_for_spring_interactions(self):
#         """Save spring interaction events to CSV"""
#         with open('/home/user/Desktop/delay/spring_interactions.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Start Time (s)', 'End Time (s)', 'Duration (s)', 'Exit Reason',
#                 'Start X (m)', 'Start Y (m)', 'Start Z (m)',
#                 'End X (m)', 'End Y (m)', 'End Z (m)',
#                 'Displacement X (m)', 'Displacement Y (m)', 'Displacement Z (m)',
#                 'Total Displacement (m)',
#                 'Trigger Force X (N)', 'Trigger Force Y (N)', 'Trigger Force Z (N)'
#             ])
            
#             for interaction in self.spring_interaction_log:
#                 start_time = interaction['start_time']
#                 end_time = interaction.get('end_time', start_time)  # In case shutdown before end
#                 duration = end_time - start_time
#                 reason = interaction.get('reason', 'incomplete')
                
#                 start_pos = interaction['start_position']
#                 end_pos = interaction.get('end_position', start_pos)
#                 displacement = end_pos - start_pos
#                 total_disp = np.linalg.norm(displacement)
                
#                 trigger_force = interaction['trigger_force']
                
#                 writer.writerow([
#                     f"{start_time:.9f}",
#                     f"{end_time:.9f}",
#                     f"{duration:.9f}",
#                     reason,
#                     f"{start_pos[0]:.6f}", f"{start_pos[1]:.6f}", f"{start_pos[2]:.6f}",
#                     f"{end_pos[0]:.6f}", f"{end_pos[1]:.6f}", f"{end_pos[2]:.6f}",
#                     f"{displacement[0]:.6f}", f"{displacement[1]:.6f}", f"{displacement[2]:.6f}",
#                     f"{total_disp:.6f}",
#                     f"{trigger_force[0]:.3f}", f"{trigger_force[1]:.3f}", f"{trigger_force[2]:.3f}"
#                 ])
        
#         rospy.loginfo(f"Spring interaction log saved: {len(self.spring_interaction_log)} events")
#         rospy.loginfo("File: /home/user/Desktop/delay/spring_interactions.csv")

#     def make_csv_for_force_rtd(self):
#         """Save force round-trip delay to CSV with statistics"""
#         with open('/home/user/Desktop/delay/force_rtd.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Force Timestamp', 'Force RTD (s)'])
#             for timestamp, rtd in self.force_rtd_log:
#                 writer.writerow([f"{timestamp:.9f}", f"{rtd:.9f}"])
        
#         # Calculate and print statistics
#         if self.force_rtd_log:
#             rtds = [rtd for _, rtd in self.force_rtd_log]
#             rospy.loginfo("="*60)
#             rospy.loginfo("Force RTD Statistics:")
#             rospy.loginfo(f"  Average: {np.mean(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Min:     {np.min(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Max:     {np.max(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Std Dev: {np.std(rtds)*1000:.3f} ms")
#             rospy.loginfo(f"  Total samples: {len(rtds)}")
#             rospy.loginfo("="*60)
        
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
#         self.make_csv_for_force_rtd()
#         self.make_csv_for_spring_interactions()

# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass