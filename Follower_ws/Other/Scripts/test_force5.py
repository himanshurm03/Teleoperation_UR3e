#!/usr/bin/env python3

# #Single time

# import rospy
# import numpy as np
# from geometry_msgs.msg import WrenchStamped, Vector3
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData
# from scipy.spatial.transform import Rotation as R
# from collections import deque
# import time
# import csv

# class HybridForceController:
    

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('hybrid_force_controller', anonymous=True)

#         # Publishers - Publishing to internal topics (for pose code to bundle)
#         self.internal_force_pub = rospy.Publisher('internal_force_data', Vector3, queue_size=1)
#         self.internal_force_timestamp_pub = rospy.Publisher('internal_force_timestamp', Float64, queue_size=1)
        
#         # Subscribers
#         rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback, queue_size=1)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot, queue_size=1)
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_echo_callback, queue_size=1)


#         self.plane_z_height = 0.1628 
#         self.stiffness = 100.0  
#         self.constant_force = 2.0  
#         self.penetration_threshold = self.constant_force / self.stiffness  
        

#         self.mode = "VIRTUAL_PLANE"  
#         self.first_contact_time = None  
#         self.virtual_plane_duration = 5.0  
#         self.contact_detected = False  
      
#         self.start_time = time.time() 
#         self.joint_position_robot = np.zeros(6)
#         self.current_ee_pose = np.zeros((6, 1))
#         self.end_effector_position = np.zeros(3)
        
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.haptic_force = np.zeros(3)
        
#         # Moving Average (from normal force code)
#         self.haptic_window_size = 30
#         self.force_window = deque(maxlen=self.haptic_window_size)
        
#         # Weber sampling (from normal force code)
#         self.weber_delta = 0.05
#         self.prev_weber_force = None
        
        
#         self.robot_previous_angles = None
        
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}
#         self.force_rtd_log = []
#         self.csv_log = []  
        
#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False
        
#         rospy.loginfo("="*60)
#         rospy.loginfo("Hybrid Force Controller initialized")
#         rospy.loginfo("Mode: VIRTUAL_PLANE → FORCE_SENSOR after 2s contact")
#         rospy.loginfo(f"Plane height: {self.plane_z_height}m")
#         rospy.loginfo(f"Stiffness k: {self.stiffness} N/m")
#         rospy.loginfo(f"Constant force: {self.constant_force}N")
#         rospy.loginfo(f"Penetration threshold: {self.penetration_threshold*1000:.2f}mm")
#         rospy.loginfo(f"Virtual plane duration: {self.virtual_plane_duration}s")
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
#         """Angle unwrapping - from virtual plane code"""
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     def pose_and_rotation(self, joint_position_robot):
#         """
#         Forward kinematics from virtual plane code
#         Ensures consistent position calculations
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
#         Virtual plane force model (from virtual plane code)
#         Returns force, contact status, and penetration depth
#         """
#         z_pos = position[2]
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

#     @staticmethod
#     def forcevector_conversion(joint_position_robot, robot_force):
#         """
#         Force vector conversion from sensor frame to base frame (from normal force code)
#         """
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
#         Weber sampling (from normal force code)
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

#     def joint_callback_robot(self, msg: JointState):
#         """Update joint positions and calculate end effector pose"""
#         self.joint_position_robot = np.array([
#             msg.position[2], msg.position[1], msg.position[0], 
#             msg.position[3], msg.position[4], msg.position[5]
#         ])
        
#         # Calculate full pose using exact FK from virtual plane code
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
        
#         # Extract position (first 3 elements)
#         self.end_effector_position = self.current_ee_pose[0:3, 0]

#     def robot_force_callback(self, msg: WrenchStamped):
#         """Update force sensor readings (from normal force code)"""
#         if time.time() - self.start_time < 1:
#             return  # Booting
        
#         robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
#         self.force_window.append(robot_force)
#         self.robot_force = np.mean(self.force_window, axis=0)

#     def check_and_update_mode(self, is_virtual_contact):
#         """
#         Check if mode should switch from VIRTUAL_PLANE to FORCE_SENSOR
        
#         Logic:
#         1. If in VIRTUAL_PLANE mode and contact detected for first time → record contact time
#         2. If 2 seconds have elapsed since first contact → switch to FORCE_SENSOR mode
#         """
#         if self.mode == "VIRTUAL_PLANE":
#             if is_virtual_contact and not self.contact_detected:
#                 # First contact detected
#                 self.first_contact_time = rospy.get_time()
#                 self.contact_detected = True
#                 rospy.loginfo("="*60)
#                 rospy.loginfo("FIRST CONTACT DETECTED - Starting 2s timer")
#                 rospy.loginfo(f"Contact time: {self.first_contact_time:.3f}s")
#                 rospy.loginfo("="*60)
            
#             if self.contact_detected:
#                 # Check if 2 seconds have elapsed
#                 elapsed_time = rospy.get_time() - self.first_contact_time
                
#                 if elapsed_time >= self.virtual_plane_duration:
#                     # Switch to force sensor mode
#                     self.mode = "FORCE_SENSOR"
#                     rospy.loginfo("="*60)
#                     rospy.loginfo("SWITCHING TO FORCE_SENSOR MODE")
#                     rospy.loginfo(f"Elapsed time: {elapsed_time:.3f}s")
#                     rospy.loginfo("Now using real force sensor data")
#                     rospy.loginfo("="*60)

#     def haptic_force_callback(self, event):
#         """Main force calculation and publishing loop"""
#         if self.shutdown_flag:
#             return

#         current_time = rospy.get_time()
        
   
        
#         # 1. Virtual plane force
#         virtual_force, is_virtual_contact, penetration = self.calculate_virtual_plane_force(
#             self.end_effector_position
#         )
        
#         # 2. Real force sensor (with moving average and Weber sampling)
#         robot_force_ma = self.forcevector_conversion(
#             self.joint_position_robot,
#             self.robot_force
#         )
#         sensor_force = self.apply_weber_sampling(robot_force_ma)
        
       
#         self.check_and_update_mode(is_virtual_contact)
        
        
#         if self.mode == "VIRTUAL_PLANE":
            
#             output_force = virtual_force
#             mode_str = "VIRTUAL_PLANE"
#         else:
#             # APPLY CHANGES HERE
#             output_force = np.array([
#                 sensor_force[0] * 0, 
#                 sensor_force[1] * 0,  
#                 sensor_force[2]       
#             ])
#             mode_str = "FORCE_SENSOR"
        
        
#         force_msg = Vector3()
#         force_msg.x = output_force[0]
#         force_msg.y = output_force[1]
#         force_msg.z = output_force[2]
        
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
#         self.haptic_force = output_force
        
#         # ============ LOG TO CSV BUFFER ============
#         self.csv_log.append((
#             current_time,
#             mode_str,
#             self.end_effector_position[0],
#             self.end_effector_position[1],
#             self.end_effector_position[2],
#             output_force[0],
#             output_force[1],
#             output_force[2],
#             penetration if is_virtual_contact else 0.0,
#             1 if is_virtual_contact else 0
#         ))
        
#         # ============ PERIODIC STATUS LOGGING ============
#         if len(self.csv_log) % 500 == 0:  # Every ~1 second at 500Hz
#             z_pos = self.end_effector_position[2]
#             force_mag = np.linalg.norm(output_force)
            
#             if self.mode == "VIRTUAL_PLANE":
#                 state = "CONTACT" if is_virtual_contact else "FREE"
#                 pen_val = penetration if is_virtual_contact else 0.0
                
#                 if is_virtual_contact:
#                     if pen_val < self.penetration_threshold:
#                         region = f"SPRING (F={self.stiffness}*{pen_val:.4f})"
#                     else:
#                         region = "CONSTANT (F=2N)"
#                 else:
#                     region = "FREE"
                
#                 time_info = ""
#                 if self.contact_detected:
#                     elapsed = rospy.get_time() - self.first_contact_time
#                     remaining = self.virtual_plane_duration - elapsed
#                     time_info = f"| Switch in: {remaining:.2f}s"
                
#                 rospy.loginfo(
#                     f"[{self.mode}] Z: {z_pos:.4f}m | {state} | Force: {force_mag:.3f}N | "
#                     f"Pen: {pen_val*1000:.2f}mm | {region} {time_info}"
#                 )
#             else:
#                 rospy.loginfo(
#                     f"[{self.mode}] Z: {z_pos:.4f}m | Force: {force_mag:.3f}N | "
#                     f"Sensor: [{sensor_force[0]:.2f}, {sensor_force[1]:.2f}, {sensor_force[2]:.2f}]N"
#                 )

#     def make_csv_force_interaction(self):
#         """Save complete force interaction data to CSV"""
#         with open('/home/user/Desktop/delay/hybrid_force_interaction.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow([
#                 'Time (ROS)', 'Mode',
#                 'Position X (m)', 'Position Y (m)', 'Position Z (m)',
#                 'Force X (N)', 'Force Y (N)', 'Force Z (N)',
#                 'Penetration (m)', 'Contact (0/1)'
#             ])
            
#             for row in self.csv_log:
#                 writer.writerow([
#                     f"{row[0]:.9f}",  # Time
#                     row[1],           # Mode
#                     f"{row[2]:.6f}", f"{row[3]:.6f}", f"{row[4]:.6f}",  # Position
#                     f"{row[5]:.6f}", f"{row[6]:.6f}", f"{row[7]:.6f}",  # Force
#                     f"{row[8]:.6f}",  # Penetration
#                     int(row[9])       # Contact
#                 ])
        
#         rospy.loginfo("="*60)
#         rospy.loginfo("Interaction CSV saved to: /home/user/Desktop/delay/hybrid_force_interaction.csv")
        
#         # Calculate and print statistics
#         if self.csv_log:
#             virtual_samples = sum(1 for row in self.csv_log if row[1] == "VIRTUAL_PLANE")
#             sensor_samples = sum(1 for row in self.csv_log if row[1] == "FORCE_SENSOR")
#             contact_events = sum(1 for row in self.csv_log if row[9] == 1)
#             max_penetration = max(row[8] for row in self.csv_log)
#             max_force = max(np.linalg.norm([row[5], row[6], row[7]]) for row in self.csv_log)
            
#             rospy.loginfo(f"Total samples: {len(self.csv_log)}")
#             rospy.loginfo(f"Virtual plane samples: {virtual_samples} ({100*virtual_samples/len(self.csv_log):.1f}%)")
#             rospy.loginfo(f"Force sensor samples: {sensor_samples} ({100*sensor_samples/len(self.csv_log):.1f}%)")
#             rospy.loginfo(f"Contact samples: {contact_events} ({100*contact_events/len(self.csv_log):.1f}%)")
#             rospy.loginfo(f"Max penetration: {max_penetration*1000:.2f} mm")
#             rospy.loginfo(f"Max force: {max_force:.2f} N")
#         rospy.loginfo("="*60)

#     def make_csv_for_generation_and_send(self):
#         """Save force generation timing data"""
#         with open('/home/user/Desktop/delay/hybrid_force_gen_send.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Generation Time (s)', 'Sent Timestamp (ROS Time)', 'Difference (s)'])
#             for gen_time, sent_time, diff in self.force_gen_send_log:
#                 writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
        
#         rospy.loginfo("Force timing CSV saved to: /home/user/Desktop/delay/hybrid_force_gen_send.csv")

#     def make_csv_for_force_rtd(self):
#         """Save force round-trip delay statistics"""
#         with open('/home/user/Desktop/delay/hybrid_force_rtd.csv', 'w', newline='') as csvfile:
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
        
#         rospy.loginfo("Force RTD CSV saved to: /home/user/Desktop/delay/hybrid_force_rtd.csv")

#     def main_loop(self):
#         rate = 500  # Match your system's control rate
#         rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
#         rospy.loginfo("Hybrid force control loop started at 500Hz")
#         rospy.loginfo("Initial mode: VIRTUAL_PLANE")
#         rospy.loginfo("Will switch to FORCE_SENSOR after 2s contact")
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
#         controller = HybridForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass




#####################################################################################


#Every Time
#Every Time: first z plane spring then normal FT sensor force values


import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from teleop_msgs.msg import MasterToSlaveData
from scipy.spatial.transform import Rotation as R
from collections import deque
import time
import csv

class RepeatingHybridForceController:
    

    def __init__(self):
        # Initializing node
        rospy.init_node('repeating_hybrid_force_controller', anonymous=True)
        
        # Suppress "OLD MESSAGE" warnings
        import logging
        for handler in logging.getLogger('rosout').handlers:
            handler.addFilter(lambda record: 'OLD MESSAGE' not in record.getMessage())

        # Publishers - Publishing to internal topics (for pose code to bundle)
        self.internal_force_pub = rospy.Publisher('internal_force_data', Vector3, queue_size=1)
        self.internal_force_timestamp_pub = rospy.Publisher('internal_force_timestamp', Float64, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_echo_callback, queue_size=1, tcp_nodelay=True)

        #0.1628 , 0.1764  
        self.plane_z_height = 0.1628  
        self.stiffness = 200.0  # Spring stiffness k (N/m)
        self.constant_force = 2.0  
        self.penetration_threshold = self.constant_force / self.stiffness  
        
        
        self.mode = "FORCE_SENSOR"  
        self.contact_start_time = None  
        self.virtual_plane_duration = 5.0  
        self.in_contact = False  
        self.previous_in_contact = False  
        self.contact_count = 0  
        
       
        self.start_time = time.time() 
        self.joint_position_robot = np.zeros(6)
        self.current_ee_pose = np.zeros((6, 1))
        self.end_effector_position = np.zeros(3)
        
        
        self.robot_force = np.zeros(3)
        self.robot_force_initial = np.zeros(3)
        self.haptic_force = np.zeros(3)
        
        
        self.haptic_window_size = 30
        self.force_window = deque(maxlen=self.haptic_window_size)
        
        
        self.weber_delta = 0.05
        self.prev_weber_force = None
        
     
        self.robot_previous_angles = None
        
        self.force_sent_timestamps = []
        self.force_gen_send_log = []
        self.sent_force_times = {}
        self.force_rtd_log = []
        self.csv_log = [] 
        
        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.shutdown_flag = False
        
        rospy.loginfo("="*60)
        rospy.loginfo("Repeating Hybrid Force Controller initialized")
        rospy.loginfo("Mode: Cycles between VIRTUAL_PLANE and FORCE_SENSOR")
        rospy.loginfo("Every contact → F=kx for 2s → Normal force until next contact")
        rospy.loginfo(f"Plane height: {self.plane_z_height}m")
        rospy.loginfo(f"Stiffness k: {self.stiffness} N/m")
        rospy.loginfo(f"Constant force: {self.constant_force}N")
        rospy.loginfo(f"Penetration threshold: {self.penetration_threshold*1000:.2f}mm")
        rospy.loginfo(f"Virtual plane duration: {self.virtual_plane_duration}s per contact")
        rospy.loginfo("Publishing to: internal_force_data, internal_force_timestamp")
        rospy.loginfo("="*60)

    def master_echo_callback(self, msg: MasterToSlaveData):
        """RTD calculation for force data"""
        force_timestamp_echo = msg.force_timestamp
        
        if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
            current_time = rospy.get_time()
            rtd = current_time - force_timestamp_echo
            self.force_rtd_log.append((force_timestamp_echo, rtd))
            rospy.loginfo_throttle(2.0, f"Force RTD: {rtd*1000:.3f}ms")
            del self.sent_force_times[force_timestamp_echo]

    @staticmethod
    def unwrap_angle(angle, previous_angle):
      
        if previous_angle is None:
            return angle
        delta = angle - previous_angle
        delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
        delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
        delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
        return previous_angle + delta

    def pose_and_rotation(self, joint_position_robot):
      
        q1, q2, q3, q4, q5, q6 = joint_position_robot
        cos = np.cos
        sin = np.sin
        a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
        x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
            (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
            (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
            (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

        y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
            (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
            (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
            (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

        z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
            (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

        r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
        r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
        r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
        r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
        r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
        r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
        r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
        r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
        r33 = -sin(q2 + q3 + q4)*sin(q5)
        
        R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
        rotation = R.from_matrix(R_matrix)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        euler_angles = np.array(euler_angles)
        if self.robot_previous_angles is not None:
            euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
        self.robot_previous_angles = euler_angles
        
        return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])

    def calculate_virtual_plane_force(self, position):
        
        z_pos = position[2]
        penetration = self.plane_z_height - z_pos
        
        if penetration <= 0:
            # Above the plane - no force
            force = np.zeros(3)
            is_contact = False
            force_magnitude = 0.0
        else:
            # Below the plane - apply force model
            if penetration < self.penetration_threshold:
                # Proportional region: F = k * delta_x
                force_magnitude = self.stiffness * penetration
            else:
                # Constant region: F = 2N
                force_magnitude = self.constant_force
            
            # Force is upward (opposing penetration)
            force = np.array([0, 0, force_magnitude])
            is_contact = True
        
        return force, is_contact, penetration

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

    def check_and_update_mode(self, is_virtual_contact):
        
        current_time = rospy.get_time()
        
        # Update contact state
        self.previous_in_contact = self.in_contact
        self.in_contact = is_virtual_contact
        
        # Detect NEW contact (rising edge)
        new_contact_detected = self.in_contact and not self.previous_in_contact
        
        if new_contact_detected:
            # NEW CONTACT EVENT - Switch to VIRTUAL_PLANE mode
            self.mode = "VIRTUAL_PLANE"
            self.contact_start_time = current_time
            self.contact_count += 1
            
            rospy.loginfo("="*60)
            rospy.loginfo(f"NEW CONTACT #{self.contact_count} DETECTED - Switching to VIRTUAL_PLANE")
            rospy.loginfo(f"Contact time: {self.contact_start_time:.3f}s")
            rospy.loginfo("F=kx mode active for 2 seconds")
            rospy.loginfo("="*60)
        
        # Check if we should switch from VIRTUAL_PLANE to FORCE_SENSOR
        if self.mode == "VIRTUAL_PLANE" and self.contact_start_time is not None:
            elapsed_time = current_time - self.contact_start_time
            
            if elapsed_time >= self.virtual_plane_duration:
                # Switch to force sensor mode
                self.mode = "FORCE_SENSOR"
                
                rospy.loginfo("="*60)
                rospy.loginfo(f"SWITCHING TO FORCE_SENSOR MODE (Contact #{self.contact_count})")
                rospy.loginfo(f"Elapsed time: {elapsed_time:.3f}s")
                rospy.loginfo("Now using real force sensor data")
                rospy.loginfo("Waiting for next contact...")
                rospy.loginfo("="*60)

    def joint_callback_robot(self, msg: JointState):
        """Update joint positions and calculate end effector pose"""
        self.joint_position_robot = np.array([
            msg.position[2], msg.position[1], msg.position[0], 
            msg.position[3], msg.position[4], msg.position[5]
        ])
        
        # Calculate full pose using exact FK from virtual plane code
        self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
        
        # Extract position (first 3 elements)
        self.end_effector_position = self.current_ee_pose[0:3, 0]

    def robot_force_callback(self, msg: WrenchStamped):
        """Update force sensor readings (from normal force code)"""
        if time.time() - self.start_time < 1:
            return  # Booting
        
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

    def haptic_force_callback(self, event):
        """Main force calculation and publishing loop"""
        if self.shutdown_flag:
            return

        current_time = rospy.get_time()
        
        # ============ CALCULATE BOTH FORCE SOURCES ============
        
        # 1. Virtual plane force
        virtual_force, is_virtual_contact, penetration = self.calculate_virtual_plane_force(
            self.end_effector_position
        )
        
        # 2. Real force sensor (with moving average and Weber sampling)
        robot_force_ma = self.forcevector_conversion(
            self.joint_position_robot,
            self.robot_force
        )
        sensor_force = self.apply_weber_sampling(robot_force_ma)
        
        # ============ CHECK AND UPDATE MODE (REPEATING LOGIC) ============
        self.check_and_update_mode(is_virtual_contact)
        
        # ============ SELECT FORCE BASED ON MODE ============
        if self.mode == "VIRTUAL_PLANE":
            # Use virtual plane force
            output_force = virtual_force
            mode_str = "VIRTUAL_PLANE"
        else:
            # Use real sensor force (with scaling from normal force code)
            output_force = np.array([
                sensor_force[0] * 0,  # X scaled to 0
                sensor_force[1] * 0,  # Y scaled to 0
                sensor_force[2]       # Z unchanged
            ])
            mode_str = "FORCE_SENSOR"
        
        # ============ PUBLISH FORCE ============
        force_msg = Vector3()
        force_msg.x = output_force[0]
        force_msg.y = output_force[1]
        force_msg.z = output_force[2]
        
        self.internal_force_pub.publish(force_msg)

        # Publish force timestamp
        force_timestamp = current_time
        self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

        # Store timestamp for RTD calculation
        self.sent_force_times[force_timestamp] = time.time()

        # Logging for generation timing
        gen_time = time.time()
        self.force_sent_timestamps.append(force_timestamp)
        diff = force_timestamp - gen_time
        self.force_gen_send_log.append((gen_time, force_timestamp, diff))

        # Store haptic force
        self.haptic_force = output_force
        
        # ============ LOG TO CSV BUFFER ============
        self.csv_log.append((
            current_time,
            mode_str,
            self.contact_count,
            self.end_effector_position[0],
            self.end_effector_position[1],
            self.end_effector_position[2],
            output_force[0],
            output_force[1],
            output_force[2],
            penetration if is_virtual_contact else 0.0,
            1 if is_virtual_contact else 0
        ))
        
        # ============ PERIODIC STATUS LOGGING ============
        if len(self.csv_log) % 500 == 0:  # Every ~1 second at 500Hz
            z_pos = self.end_effector_position[2]
            force_mag = np.linalg.norm(output_force)
            
            if self.mode == "VIRTUAL_PLANE":
                state = "CONTACT" if is_virtual_contact else "FREE"
                pen_val = penetration if is_virtual_contact else 0.0
                
                if is_virtual_contact:
                    if pen_val < self.penetration_threshold:
                        region = f"SPRING (F={self.stiffness}*{pen_val:.4f})"
                    else:
                        region = "CONSTANT (F=2N)"
                else:
                    region = "FREE"
                
                time_info = ""
                if self.contact_start_time is not None:
                    elapsed = current_time - self.contact_start_time
                    remaining = self.virtual_plane_duration - elapsed
                    time_info = f"| Switch in: {max(0, remaining):.2f}s"
                
                rospy.loginfo(
                    f"[{self.mode}] Contact#{self.contact_count} | Z: {z_pos:.4f}m | {state} | "
                    f"Force: {force_mag:.3f}N | Pen: {pen_val*1000:.2f}mm | {region} {time_info}"
                )
            else:
                rospy.loginfo(
                    f"[{self.mode}] Contact#{self.contact_count} | Z: {z_pos:.4f}m | "
                    f"Force: {force_mag:.3f}N | Sensor: [{sensor_force[0]:.2f}, {sensor_force[1]:.2f}, {sensor_force[2]:.2f}]N"
                )

    def make_csv_force_interaction(self):
        """Save complete force interaction data to CSV"""
        with open('/home/user/Desktop/delay/interaction.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Time (ROS)', 'Mode', 'Contact Event #',
                'Position X (m)', 'Position Y (m)', 'Position Z (m)',
                'Force X (N)', 'Force Y (N)', 'Force Z (N)',
                'Penetration (m)', 'Contact (0/1)'
            ])
            
            for row in self.csv_log:
                writer.writerow([
                    f"{row[0]:.9f}",  # Time
                    row[1],           # Mode
                    row[2],           # Contact event number
                    f"{row[3]:.6f}", f"{row[4]:.6f}", f"{row[5]:.6f}",  # Position
                    f"{row[6]:.6f}", f"{row[7]:.6f}", f"{row[8]:.6f}",  # Force
                    f"{row[9]:.6f}",  # Penetration
                    int(row[10])      # Contact
                ])
        
        rospy.loginfo("="*60)
        rospy.loginfo("Interaction CSV saved to: /home/user/Desktop/delay/repeating_hybrid_interaction.csv")
        
        # Calculate and print statistics
        if self.csv_log:
            virtual_samples = sum(1 for row in self.csv_log if row[1] == "VIRTUAL_PLANE")
            sensor_samples = sum(1 for row in self.csv_log if row[1] == "FORCE_SENSOR")
            contact_events = sum(1 for row in self.csv_log if row[10] == 1)
            max_penetration = max(row[9] for row in self.csv_log)
            max_force = max(np.linalg.norm([row[6], row[7], row[8]]) for row in self.csv_log)
            
            rospy.loginfo(f"Total samples: {len(self.csv_log)}")
            rospy.loginfo(f"Total contact events: {self.contact_count}")
            rospy.loginfo(f"Virtual plane samples: {virtual_samples} ({100*virtual_samples/len(self.csv_log):.1f}%)")
            rospy.loginfo(f"Force sensor samples: {sensor_samples} ({100*sensor_samples/len(self.csv_log):.1f}%)")
            rospy.loginfo(f"Contact samples: {contact_events} ({100*contact_events/len(self.csv_log):.1f}%)")
            rospy.loginfo(f"Max penetration: {max_penetration*1000:.2f} mm")
            rospy.loginfo(f"Max force: {max_force:.2f} N")
        rospy.loginfo("="*60)

    def make_csv_for_generation_and_send(self):
        """Save force generation timing data"""
        with open('/home/user/Desktop/delay/repeating_hybrid_gen_send.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Generation Time (s)', 'Sent Timestamp (ROS Time)', 'Difference (s)'])
            for gen_time, sent_time, diff in self.force_gen_send_log:
                writer.writerow([f"{gen_time:.9f}", f"{sent_time:.9f}", f"{diff:.9f}"])
        
        rospy.loginfo("Force timing CSV saved to: /home/user/Desktop/delay/repeating_hybrid_gen_send.csv")

    def make_csv_for_force_rtd(self):
        """Save force round-trip delay statistics"""
        with open('/home/user/Desktop/delay/forcertd.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Force Timestamp', 'Force RTD (s)', 'Force RTD (ms)'])
            for timestamp, rtd in self.force_rtd_log:
                writer.writerow([f"{timestamp:.9f}", f"{rtd:.9f}", f"{rtd*1000:.3f}"])
        
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
        
        rospy.loginfo("Force RTD CSV saved to: /home/user/Desktop/delay/repeating_hybrid_rtd.csv")

    def main_loop(self):
        rate = 500  # Match your system's control rate
        rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
        rospy.loginfo("Repeating hybrid force control loop started at 500Hz")
        rospy.loginfo("Initial mode: FORCE_SENSOR (waiting for first contact)")
        rospy.loginfo("Every contact → VIRTUAL_PLANE (2s) → FORCE_SENSOR (until next contact)")
        rospy.spin()

    def shutdown_hook(self):
        self.shutdown_flag = True
        zero_force_msg = Vector3()
        zero_force_msg.x, zero_force_msg.y, zero_force_msg.z = (0, 0, 0)
        rospy.loginfo("Sending zero force to internal topic.")
        for _ in range(10):
            self.internal_force_pub.publish(zero_force_msg)
            time.sleep(0.01)
        rospy.loginfo("Shutting down, sent zero force.")
        
        # Save all data
        try:
            self.make_csv_force_interaction()
            self.make_csv_for_generation_and_send()
            self.make_csv_for_force_rtd()
        except Exception as e:
            rospy.logerr(f"Error saving data: {e}")

if __name__ == "__main__":
    try:
        controller = RepeatingHybridForceController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
