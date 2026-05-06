#!/usr/bin/env python3
#Purpose:Implementation of Impedance Control and weber Fraction on Force

#this is working perfectly fine,just small issue 

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Vector3
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback
from std_msgs.msg import Float64
from teleop_msgs.msg import MasterToSlaveData
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
        rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_echo_callback)

        # For initialization parameter - USE ROS TIME
        self.start_time = rospy.get_time()
        self.robot_force = np.zeros(3)
        self.robot_force_initial = np.zeros(3)
        self.joint_position_robot = np.zeros(6)
        self.haptic_force = np.zeros(3)
        
        self.force_sent_timestamps = []
        self.force_gen_send_log = []
        self.sent_force_times = {}

        self.force_rtd_log = []


        
        self.target_force = np.zeros(3)  
        self.current_smooth_force = np.zeros(3) 
        
        # Impedance parameters(not final force)
        self.force_damping = 1.0  
        self.force_mass = 0.5  
        self.force_velocity = np.zeros(3)  
        
        self.last_time = None
        
        
        self.weber_fraction = 0.05  
        self.previous_sent_force = np.zeros(3)  
        
        # HAPTIC DEVICE LIMITS
        self.max_haptic_force = 3.0 
        self.force_safety_margin = 0.95  

        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.shutdown_flag = False

        # For plotting
        self.plot_start_time = None
        self.time_stamps = []
        self.robot_force_plot = []
        self.haptic_force_plot = []
        self.raw_force_plot = []
        
        rospy.loginfo("="*60)
        rospy.loginfo("Force controller initialized with:")
        rospy.loginfo("  - Impedance Control (transition smoothing)")
        rospy.loginfo(f"  - Force damping: {self.force_damping} N·s/m")
        rospy.loginfo(f"  - Force mass: {self.force_mass} kg")
        rospy.loginfo(f"  - Weber fraction: {self.weber_fraction*100}% (delta = {self.weber_fraction})")
        rospy.loginfo(f"  - Max haptic force: {self.max_haptic_force} N")
        rospy.loginfo("  - Processing: FT Sensor → Impedance → Weber → Haptic Device")
        rospy.loginfo("="*60)

    def master_echo_callback(self, msg: MasterToSlaveData):
     
        force_timestamp_echo = msg.force_timestamp
        
        if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
            # Use ROS time consistently for accurate RTD
            current_time = rospy.get_time()
            sent_time = self.sent_force_times[force_timestamp_echo]
            
            # Calculate RTD
            rtd = current_time - force_timestamp_echo
            
            # Sanity check
            if rtd < 0:
                rospy.logwarn(f"Negative RTD detected: {rtd*1000:.3f}ms - clock issue!")
            elif rtd > 1.0:
                rospy.logwarn(f"Very large RTD: {rtd*1000:.3f}ms - network issue!")
            
            self.force_rtd_log.append((force_timestamp_echo, rtd))
            rospy.loginfo_throttle(1.0, f"Force RTD: {rtd*1000:.3f}ms")
            del self.sent_force_times[force_timestamp_echo]

    def apply_transition_smoothing(self, target_force, dt):

        if dt <= 0 or dt > 0.1:  
            return target_force
        
        #
        force_error = target_force - self.current_smooth_force
        
        
        # Calculate desired acceleration toward target
        # Using: a = (error - D*v) / M
        acceleration = (force_error - self.force_damping * self.force_velocity) / self.force_mass
        
        # Update velocity (rate of force change)
        self.force_velocity += acceleration * dt
        
        # Update current force
        self.current_smooth_force += self.force_velocity * dt
        
        
        error_magnitude = np.linalg.norm(force_error)
        if error_magnitude < 0.01:  
            self.current_smooth_force = target_force.copy()
            self.force_velocity = np.zeros(3)
        
        
        if error_magnitude < 0.1: 
            self.force_velocity *= 0.98
        
        return self.current_smooth_force

    def apply_weber_fraction(self, new_force):
 
        prev_magnitude = np.linalg.norm(self.previous_sent_force)
        
        
        force_diff = new_force - self.previous_sent_force
        diff_magnitude = np.linalg.norm(force_diff)
        
        
        if prev_magnitude > 0.01:  # Normal case (force > 10mN)
            jnd_threshold = self.weber_fraction * prev_magnitude  # 5% of current force
        else:
            
            jnd_threshold = 0.05  
        
        
        if diff_magnitude > jnd_threshold:
            self.previous_sent_force = new_force.copy()
            rospy.logdebug(f"Weber: Force updated - change {diff_magnitude:.4f}N > threshold {jnd_threshold:.4f}N")
            return new_force
        else:
            
            rospy.logdebug(f"Weber: Force held - change {diff_magnitude:.4f}N < threshold {jnd_threshold:.4f}N")
            return self.previous_sent_force

    def saturate_force(self, force):

        force_magnitude = np.linalg.norm(force)
        max_allowed = self.max_haptic_force * self.force_safety_margin
        
        if force_magnitude > max_allowed:
            
            saturated_force = force * (max_allowed / force_magnitude)
            rospy.logwarn_throttle(1.0, f"Force saturated from {force_magnitude:.3f}N to {max_allowed:.3f}N")
            return saturated_force
        
        return force

    def update_list(self):
        if self.shutdown_flag:
            return
        current_time = rospy.get_time()
        if self.plot_start_time is None:
            self.plot_start_time = current_time
        self.time_stamps.append(current_time - self.plot_start_time)
        raw_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        self.robot_force_plot.append(raw_force)
        self.raw_force_plot.append(raw_force.copy())
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
        if rospy.get_time() - self.start_time < 1:
            rospy.loginfo("booting")
       
        self.robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def haptic_force_callback(self, event):
        
        if self.shutdown_flag:
            return

        # Get current time and calculate dt
        current_time = rospy.get_time()
        if self.last_time is None:
            self.last_time = current_time
            dt = 0.001  # Initial dt
        else:
            dt = current_time - self.last_time
            self.last_time = current_time
            
        # Sanity check on dt
        if dt > 0.1:  # More than 100ms is suspicious
            rospy.logwarn(f"Large dt detected: {dt*1000:.1f}ms")
            dt = 0.001

       
        raw_robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        

        smoothed_force = self.apply_transition_smoothing(raw_robot_force, dt)
        
       
        perceptually_filtered_force = self.apply_weber_fraction(smoothed_force)
        
        
        final_force = self.saturate_force(perceptually_filtered_force)
        
        # Create force message (Vector3)
        force_msg = Vector3()
        force_msg.x = final_force[0]
        force_msg.y = final_force[1]
        force_msg.z = final_force[2]
        
        # Publish to internal topic (pose code will bundle this)
        self.internal_force_pub.publish(force_msg)

        # Publish force timestamp to internal topic (ROS time)
        force_timestamp = rospy.get_time()
        self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

        # Store ROS time for RTD calculation
        self.sent_force_times[force_timestamp] = rospy.get_time()

        # Logging - use ROS time for consistency
        gen_time = rospy.get_time()
        self.force_sent_timestamps.append(force_timestamp)
        diff = force_timestamp - gen_time
        self.force_gen_send_log.append((gen_time, force_timestamp, diff))

        # Store haptic force for plotting/logging
        self.haptic_force = final_force

        # Update plots and lists
        self.update_list()

    def plot_data(self):
        times = np.array(self.time_stamps)
        raw_force_plot = np.array(self.raw_force_plot)
        haptic_force_plot = np.array(self.haptic_force_plot)

        fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
        axs[0, 0].plot(times, raw_force_plot[:, 0], label='raw force', alpha=0.5, linewidth=1)
        axs[0, 0].plot(times, haptic_force_plot[:, 0], label='smoothed force', linewidth=2)
        axs[0, 0].set_title('Force in X (Transition Smoothing)')
        axs[0, 0].set_ylabel('Newtons')
        axs[0, 0].set_xlabel('Seconds')
        axs[0, 0].grid(True)
        axs[0, 0].legend()

        axs[1, 0].plot(times, raw_force_plot[:, 1], label='raw force', alpha=0.5, linewidth=1)
        axs[1, 0].plot(times, haptic_force_plot[:, 1], label='smoothed force', linewidth=2)
        axs[1, 0].set_title('Force in Y (Transition Smoothing)')
        axs[1, 0].set_ylabel('Newtons')
        axs[1, 0].set_xlabel('Seconds')
        axs[1, 0].grid(True)
        axs[1, 0].legend()

        axs[2, 0].plot(times, raw_force_plot[:, 2], label='raw force', alpha=0.5, linewidth=1)
        axs[2, 0].plot(times, haptic_force_plot[:, 2], label='smoothed force', linewidth=2)
        axs[2, 0].set_title('Force in Z (Transition Smoothing)')
        axs[2, 0].set_ylabel('Newtons')
        axs[2, 0].set_xlabel('Seconds')
        axs[2, 0].grid(True)
        axs[2, 0].legend()

        axs[0, 1].plot(times, raw_force_plot[:, 0] - haptic_force_plot[:, 0])
        axs[0, 1].set_title('Smoothing Effect in X')
        axs[0, 1].set_ylabel('Newtons')
        axs[0, 1].set_xlabel('Seconds')
        axs[0, 1].grid(True)

        axs[1, 1].plot(times, raw_force_plot[:, 1] - haptic_force_plot[:, 1])
        axs[1, 1].set_title('Smoothing Effect in Y')
        axs[1, 1].set_ylabel('Newtons')
        axs[1, 1].set_xlabel('Seconds')
        axs[1, 1].grid(True)

        axs[2, 1].plot(times, raw_force_plot[:, 2] - haptic_force_plot[:, 2])
        axs[2, 1].set_title('Smoothing Effect in Z')
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
                'Raw Force X (N)', 'Raw Force Y (N)', 'Raw Force Z (N)',
                'Smoothed Haptic Force X (N)', 'Smoothed Haptic Force Y (N)', 'Smoothed Haptic Force Z (N)'
            ])
            for i in range(len(self.time_stamps)):
                ts_sent = self.force_sent_timestamps[i] if i < len(self.force_sent_timestamps) else ''
                writer.writerow([
                    f"{ts_sent:.9f}" if ts_sent != '' else '',
                    f"{self.time_stamps[i]:.9f}",
                    self.raw_force_plot[i][0], self.raw_force_plot[i][1], self.raw_force_plot[i][2],
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



#purpose:Contact Release Detection

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

#         # For initialization parameter - USE ROS TIME
#         self.start_time = rospy.get_time()
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_force = np.zeros(3)
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}
        
#         # For RTD logging
#         self.force_rtd_log = []

#         # IMPEDANCE CONTROL PARAMETERS
#         self.target_force = np.zeros(3)  # The force we're smoothly transitioning TO
#         self.current_smooth_force = np.zeros(3)  # Current smoothed force value
        
#         # Impedance parameters (not final force)
#         self.force_damping = 2.0  # N·s/m - Higher = slower, smoother transitions
#         self.force_mass = 0.8  # kg - Virtual mass for force transitions
#         self.force_velocity = np.zeros(3)  # Rate of change of force
        
#         self.last_time = None
        
#         # WEBER FRACTION PARAMETERS 
#         self.weber_fraction = 0.05  # 5% Weber fraction (delta = 0.05)
#         self.previous_sent_force = np.zeros(3)  # Last force sent to haptic device
        
#         # HAPTIC DEVICE LIMITS
#         self.max_haptic_force = 3.0  # Maximum force haptic device can handle (N)
#         self.force_safety_margin = 0.95  # Use 95% of max to stay safe

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For plotting
#         self.plot_start_time = None
#         self.time_stamps = []
#         self.robot_force_plot = []
#         self.haptic_force_plot = []
#         self.raw_force_plot = []
        
#         rospy.loginfo("="*60)
#         rospy.loginfo("Force controller initialized with:")
#         rospy.loginfo("  - Impedance Control (transition smoothing)")
#         rospy.loginfo(f"  - Force damping: {self.force_damping} N·s/m")
#         rospy.loginfo(f"  - Force mass: {self.force_mass} kg")
#         rospy.loginfo(f"  - Weber fraction: {self.weber_fraction*100}% (delta = {self.weber_fraction})")
#         rospy.loginfo(f"  - Max haptic force: {self.max_haptic_force} N")
#         rospy.loginfo("  - Processing: FT Sensor → Impedance → Weber → Haptic Device")
#         rospy.loginfo("  - Contact release detection: ENABLED")
#         rospy.loginfo("="*60)

#     def master_echo_callback(self, msg: MasterToSlaveData):
#         """Receives data FROM master for RTD calculation"""
#         force_timestamp_echo = msg.force_timestamp
        
#         if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
#             # Use ROS time consistently for accurate RTD
#             current_time = rospy.get_time()
#             sent_time = self.sent_force_times[force_timestamp_echo]
            
#             # Calculate RTD
#             rtd = current_time - force_timestamp_echo
            
#             # Sanity check
#             if rtd < 0:
#                 rospy.logwarn(f"Negative RTD detected: {rtd*1000:.3f}ms - clock issue!")
#             elif rtd > 1.0:
#                 rospy.logwarn(f"Very large RTD: {rtd*1000:.3f}ms - network issue!")
            
#             self.force_rtd_log.append((force_timestamp_echo, rtd))
#             rospy.loginfo_throttle(1.0, f"Force RTD: {rtd*1000:.3f}ms")
#             del self.sent_force_times[force_timestamp_echo]

#     def apply_transition_smoothing(self, target_force, dt):
#         """
#         Apply impedance-based smoothing to TRANSITIONS only.
#         The final force value equals target_force (preserving hardness).
#         Only the RATE OF CHANGE is controlled (eliminating bounce).
        
#         Special handling: Fast decay when contact is released (force drops to ~0)
        
#         Args:
#             target_force: The actual force from the robot (what we want to reach)
#             dt: Time step
            
#         Returns:
#             Smoothly transitioning force that will eventually equal target_force
#         """
#         if dt <= 0 or dt > 0.1:  # Sanity check on dt
#             return target_force
        
#         # Calculate the force ERROR (how far we are from target)
#         force_error = target_force - self.current_smooth_force
        
#         # CONTACT RELEASE DETECTION
#         # Detect when contact is released (force drops to near zero)
#         target_magnitude = np.linalg.norm(target_force)
#         current_magnitude = np.linalg.norm(self.current_smooth_force)
        
#         # If target force is very small BUT current force is significant
#         # This means contact was just released → fast decay needed!
#         if target_magnitude < 0.1 and current_magnitude > 0.2:
#             # Aggressive decay when releasing contact
#             decay_factor = 0.8  # Decay 20% per step (very fast!)
#             self.current_smooth_force *= decay_factor
#             self.force_velocity *= decay_factor
            
#             rospy.logdebug(f"Contact release detected! Fast decay: {current_magnitude:.3f}N → {np.linalg.norm(self.current_smooth_force):.3f}N")
            
#             # If close enough to zero, snap to zero immediately
#             if np.linalg.norm(self.current_smooth_force) < 0.05:  # Within 50mN
#                 self.current_smooth_force = np.zeros(3)
#                 self.force_velocity = np.zeros(3)
#                 rospy.logdebug("Snapped to zero force")
            
#             return self.current_smooth_force
        
#         # ========== NORMAL IMPEDANCE DYNAMICS ==========
#         # For all other cases (pressing in, holding, gradual changes)
        
#         # Calculate desired acceleration toward target
#         # Using: a = (error - D*v) / M
#         acceleration = (force_error - self.force_damping * self.force_velocity) / self.force_mass
        
#         # Update velocity (rate of force change)
#         self.force_velocity += acceleration * dt
        
#         # Update current force
#         self.current_smooth_force += self.force_velocity * dt
        
#         # Prevent overshoot - snap to target when very close
#         error_magnitude = np.linalg.norm(force_error)
#         if error_magnitude < 0.01:  # Within 10mN
#             self.current_smooth_force = target_force.copy()
#             self.force_velocity = np.zeros(3)
        
#         # Decay velocity when close to target (prevents oscillation)
#         if error_magnitude < 0.1:  # Within 100mN
#             self.force_velocity *= 0.98
        
#         return self.current_smooth_force

#     def apply_weber_fraction(self, new_force):
#         """
#         Apply Weber fraction to determine if force change is perceptible.
#         Only update haptic device if change exceeds Just Noticeable Difference (JND).
        
#         Weber's Law: ΔI/I = constant (where delta = 0.05 or 5%)
        
#         Args:
#             new_force: New force vector after impedance smoothing
            
#         Returns:
#             Force to send to haptic device (either new or previous)
#         """
#         # Calculate magnitude of previous force
#         prev_magnitude = np.linalg.norm(self.previous_sent_force)
        
#         # Calculate the change in force
#         force_diff = new_force - self.previous_sent_force
#         diff_magnitude = np.linalg.norm(force_diff)
        
#         # Calculate Just Noticeable Difference (JND) threshold
#         # JND = weber_fraction × reference_stimulus
#         if prev_magnitude > 0.01:  # Normal case (force > 10mN)
#             jnd_threshold = self.weber_fraction * prev_magnitude  # 5% of current force
#         else:
#             # For very small forces, use absolute threshold
#             jnd_threshold = 0.05  # 50mN absolute threshold
        
#         # Only update if change exceeds JND (perceptible change)
#         if diff_magnitude > jnd_threshold:
#             self.previous_sent_force = new_force.copy()
#             rospy.logdebug(f"Weber: Force updated - change {diff_magnitude:.4f}N > threshold {jnd_threshold:.4f}N")
#             return new_force
#         else:
#             # Change not perceptible, keep sending previous force
#             rospy.logdebug(f"Weber: Force held - change {diff_magnitude:.4f}N < threshold {jnd_threshold:.4f}N")
#             return self.previous_sent_force

#     def saturate_force(self, force):
#         """
#         Saturate force to stay within haptic device limits.
        
#         Args:
#             force: Input force vector
            
#         Returns:
#             Saturated force vector
#         """
#         force_magnitude = np.linalg.norm(force)
#         max_allowed = self.max_haptic_force * self.force_safety_margin
        
#         if force_magnitude > max_allowed:
#             # Scale down to maximum allowed
#             saturated_force = force * (max_allowed / force_magnitude)
#             rospy.logwarn_throttle(1.0, f"Force saturated from {force_magnitude:.3f}N to {max_allowed:.3f}N")
#             return saturated_force
        
#         return force

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.plot_start_time is None:
#             self.plot_start_time = current_time
#         self.time_stamps.append(current_time - self.plot_start_time)
#         raw_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         self.robot_force_plot.append(raw_force)
#         self.raw_force_plot.append(raw_force.copy())
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
#         if rospy.get_time() - self.start_time < 1:
#             rospy.loginfo("booting")
#         # Direct force reading - impedance control handles smoothing
#         self.robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

#     def haptic_force_callback(self, event):
#         """
#         Main force processing pipeline:
        
#         FT Sensor (Raw Force)
#                ↓
#         [Coordinate Transform] - Convert to base frame
#                ↓
#         [Impedance Control] - Smooth transitions, eliminate bounce
#                ↓
#         [Weber Fraction] - Filter imperceptible changes (5% JND)
#                ↓
#         [Saturation] - Limit to 3N max
#                ↓
#         Internal Topics → Pose Code → Master Side → Haptic Device
#         """
#         if self.shutdown_flag:
#             return

#         # Get current time and calculate dt
#         current_time = rospy.get_time()
#         if self.last_time is None:
#             self.last_time = current_time
#             dt = 0.001  # Initial dt
#         else:
#             dt = current_time - self.last_time
#             self.last_time = current_time
            
#         # Sanity check on dt
#         if dt > 0.1:  # More than 100ms is suspicious
#             rospy.logwarn(f"Large dt detected: {dt*1000:.1f}ms")
#             dt = 0.001

#         # Step 1: Get raw force from robot (with coordinate transformation)
#         raw_robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        
#         # Step 2: Apply impedance control for smooth transitions
#         # This removes bounce while preserving object characteristics
#         smoothed_force = self.apply_transition_smoothing(raw_robot_force, dt)
        
#         # Step 3: Apply Weber fraction (perceptual filtering)
#         # Only send updates if change is perceptible (> 5% JND)
#         perceptually_filtered_force = self.apply_weber_fraction(smoothed_force)
        
#         # Step 4: Saturate to haptic device limits (3N max)
#         final_force = self.saturate_force(perceptually_filtered_force)
        
#         # Create force message (Vector3)
#         force_msg = Vector3()
#         force_msg.x = final_force[0]
#         force_msg.y = final_force[1]
#         force_msg.z = final_force[2]
        
#         # Publish to internal topic (pose code will bundle this)
#         self.internal_force_pub.publish(force_msg)

#         # Publish force timestamp to internal topic (ROS time)
#         force_timestamp = rospy.get_time()
#         self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

#         # Store ROS time for RTD calculation
#         self.sent_force_times[force_timestamp] = rospy.get_time()

#         # Logging - use ROS time for consistency
#         gen_time = rospy.get_time()
#         self.force_sent_timestamps.append(force_timestamp)
#         diff = force_timestamp - gen_time
#         self.force_gen_send_log.append((gen_time, force_timestamp, diff))

#         # Store haptic force for plotting/logging
#         self.haptic_force = final_force

#         # Update plots and lists
#         self.update_list()

#     def plot_data(self):
#         times = np.array(self.time_stamps)
#         raw_force_plot = np.array(self.raw_force_plot)
#         haptic_force_plot = np.array(self.haptic_force_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, raw_force_plot[:, 0], label='raw force', alpha=0.5, linewidth=1)
#         axs[0, 0].plot(times, haptic_force_plot[:, 0], label='smoothed force', linewidth=2)
#         axs[0, 0].set_title('Force in X (Transition Smoothing)')
#         axs[0, 0].set_ylabel('Newtons')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)
#         axs[0, 0].legend()

#         axs[1, 0].plot(times, raw_force_plot[:, 1], label='raw force', alpha=0.5, linewidth=1)
#         axs[1, 0].plot(times, haptic_force_plot[:, 1], label='smoothed force', linewidth=2)
#         axs[1, 0].set_title('Force in Y (Transition Smoothing)')
#         axs[1, 0].set_ylabel('Newtons')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)
#         axs[1, 0].legend()

#         axs[2, 0].plot(times, raw_force_plot[:, 2], label='raw force', alpha=0.5, linewidth=1)
#         axs[2, 0].plot(times, haptic_force_plot[:, 2], label='smoothed force', linewidth=2)
#         axs[2, 0].set_title('Force in Z (Transition Smoothing)')
#         axs[2, 0].set_ylabel('Newtons')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)
#         axs[2, 0].legend()

#         axs[0, 1].plot(times, raw_force_plot[:, 0] - haptic_force_plot[:, 0])
#         axs[0, 1].set_title('Smoothing Effect in X')
#         axs[0, 1].set_ylabel('Newtons')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)

#         axs[1, 1].plot(times, raw_force_plot[:, 1] - haptic_force_plot[:, 1])
#         axs[1, 1].set_title('Smoothing Effect in Y')
#         axs[1, 1].set_ylabel('Newtons')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)

#         axs[2, 1].plot(times, raw_force_plot[:, 2] - haptic_force_plot[:, 2])
#         axs[2, 1].set_title('Smoothing Effect in Z')
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
#                 'Raw Force X (N)', 'Raw Force Y (N)', 'Raw Force Z (N)',
#                 'Smoothed Haptic Force X (N)', 'Smoothed Haptic Force Y (N)', 'Smoothed Haptic Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 ts_sent = self.force_sent_timestamps[i] if i < len(self.force_sent_timestamps) else ''
#                 writer.writerow([
#                     f"{ts_sent:.9f}" if ts_sent != '' else '',
#                     f"{self.time_stamps[i]:.9f}",
#                     self.raw_force_plot[i][0], self.raw_force_plot[i][1], self.raw_force_plot[i][2],
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




#log the raw force and the final force from the impedance control and weber fraction.

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

#         # For initialization parameter - USE ROS TIME
#         self.start_time = rospy.get_time()
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_force = np.zeros(3)
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}
        
#         # For RTD logging
#         self.force_rtd_log = []

#         # ========== NEW: LOGGING FOR RAW AND FINAL FORCES ==========
#         # For logging raw force (after coordinate transform, before impedance)
#         self.raw_force_log = []  # Stores (timestamp, raw_force_x, raw_force_y, raw_force_z)
        
#         # For logging final force (after impedance + weber, sent to haptic)
#         self.final_force_log = []  # Stores (timestamp, final_force_x, final_force_y, final_force_z)

#         # IMPEDANCE CONTROL PARAMETERS

#         self.target_force = np.zeros(3)  # The force we're smoothly transitioning TO
#         self.current_smooth_force = np.zeros(3)  # Current smoothed force value
        
#         # Impedance parameters (not final force)
#         #D=3
#         #M=0.8
#         self.force_damping = 3.0  # N·s/m - Higher = slower, smoother transitions 
#         self.force_mass = 0.8  # kg - Virtual mass for force transitions
#         self.force_velocity = np.zeros(3)  # Rate of change of force
        
#         self.last_time = None
        
#         # WEBER FRACTION PARAMETERS 
#         self.weber_fraction = 0.05  # 5% Weber fraction (delta = 0.05)
#         self.previous_sent_force = np.zeros(3)  # Last force sent to haptic device
        
#         # HAPTIC DEVICE LIMITS
#         self.max_haptic_force = 3.0  # Maximum force haptic device can handle (N)
#         self.force_safety_margin = 0.95  # Use 95% of max to stay safe

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For plotting
#         self.plot_start_time = None
#         self.time_stamps = []
#         self.robot_force_plot = []
#         self.haptic_force_plot = []
#         self.raw_force_plot = []
        
#         rospy.loginfo("="*60)
#         rospy.loginfo("Force controller initialized with:")
#         rospy.loginfo("  - Impedance Control (transition smoothing)")
#         rospy.loginfo(f"  - Force damping: {self.force_damping} N·s/m")
#         rospy.loginfo(f"  - Force mass: {self.force_mass} kg")
#         rospy.loginfo(f"  - Weber fraction: {self.weber_fraction*100}% (delta = {self.weber_fraction})")
#         rospy.loginfo(f"  - Max haptic force: {self.max_haptic_force} N")
#         rospy.loginfo("  - Processing: FT Sensor → Impedance → Weber → Haptic Device")
#         rospy.loginfo("  - Contact release detection: ENABLED")
#         rospy.loginfo("="*60)

#     def master_echo_callback(self, msg: MasterToSlaveData):
#         """Receives data FROM master for RTD calculation"""
#         force_timestamp_echo = msg.force_timestamp
        
#         if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
#             # Use ROS time consistently for accurate RTD
#             current_time = rospy.get_time()
#             sent_time = self.sent_force_times[force_timestamp_echo]
            
#             # Calculate RTD
#             rtd = current_time - force_timestamp_echo
            
#             # Sanity check
#             if rtd < 0:
#                 rospy.logwarn(f"Negative RTD detected: {rtd*1000:.3f}ms - clock issue!")
#             elif rtd > 1.0:
#                 rospy.logwarn(f"Very large RTD: {rtd*1000:.3f}ms - network issue!")
            
#             self.force_rtd_log.append((force_timestamp_echo, rtd))
#             rospy.loginfo_throttle(1.0, f"Force RTD: {rtd*1000:.3f}ms")
#             del self.sent_force_times[force_timestamp_echo]

#     def apply_transition_smoothing(self, target_force, dt):
#         """
#         Apply impedance-based smoothing to TRANSITIONS only.
#         The final force value equals target_force (preserving hardness).
#         Only the RATE OF CHANGE is controlled (eliminating bounce).
        
#         Special handling: Fast decay when contact is released (force drops to ~0)
        
#         Args:
#             target_force: The actual force from the robot (what we want to reach)
#             dt: Time step
            
#         Returns:
#             Smoothly transitioning force that will eventually equal target_force
#         """
#         if dt <= 0 or dt > 0.1:  # Sanity check on dt
#             return target_force
        
#         # Calculate the force ERROR (how far we are from target)
#         force_error = target_force - self.current_smooth_force
        
#         # ========== CONTACT RELEASE DETECTION ==========
#         # Detect when contact is released (force drops to near zero)
#         target_magnitude = np.linalg.norm(target_force)
#         current_magnitude = np.linalg.norm(self.current_smooth_force)
        
#         # If target force is very small BUT current force is significant
#         # This means contact was just released → fast decay needed!
#         if target_magnitude < 0.1 and current_magnitude > 0.2:
#             # Aggressive decay when releasing contact
#             decay_factor = 0.8  # Decay 20% per step (very fast!)
#             self.current_smooth_force *= decay_factor
#             self.force_velocity *= decay_factor
            
#             rospy.logdebug(f"Contact release detected! Fast decay: {current_magnitude:.3f}N → {np.linalg.norm(self.current_smooth_force):.3f}N")
            
#             # If close enough to zero, snap to zero immediately
#             if np.linalg.norm(self.current_smooth_force) < 0.05:  # Within 50mN
#                 self.current_smooth_force = np.zeros(3)
#                 self.force_velocity = np.zeros(3)
#                 rospy.logdebug("Snapped to zero force")
            
#             return self.current_smooth_force
        
#         # ========== NORMAL IMPEDANCE DYNAMICS ==========
#         # For all other cases (pressing in, holding, gradual changes)
        
#         # Calculate desired acceleration toward target
#         # Using: a = (error - D*v) / M
#         acceleration = (force_error - self.force_damping * self.force_velocity) / self.force_mass
        
#         # Update velocity (rate of force change)
#         self.force_velocity += acceleration * dt
        
#         # Update current force
#         self.current_smooth_force += self.force_velocity * dt
        
#         # Prevent overshoot - snap to target when very close
#         error_magnitude = np.linalg.norm(force_error)
#         if error_magnitude < 0.01:  # Within 10mN
#             self.current_smooth_force = target_force.copy()
#             self.force_velocity = np.zeros(3)
        
#         # Decay velocity when close to target (prevents oscillation)
#         if error_magnitude < 0.1:  # Within 100mN
#             self.force_velocity *= 0.98
        
#         return self.current_smooth_force

#     def apply_weber_fraction(self, new_force):
#         """
#         Apply Weber fraction to determine if force change is perceptible.
#         Only update haptic device if change exceeds Just Noticeable Difference (JND).
        
#         Weber's Law: ΔI/I = constant (where delta = 0.05 or 5%)
        
#         Args:
#             new_force: New force vector after impedance smoothing
            
#         Returns:
#             Force to send to haptic device (either new or previous)
#         """
#         # Calculate magnitude of previous force
#         prev_magnitude = np.linalg.norm(self.previous_sent_force)
        
#         # Calculate the change in force
#         force_diff = new_force - self.previous_sent_force
#         diff_magnitude = np.linalg.norm(force_diff)
        
#         # Calculate Just Noticeable Difference (JND) threshold
#         # JND = weber_fraction × reference_stimulus
#         if prev_magnitude > 0.01:  # Normal case (force > 10mN)
#             jnd_threshold = self.weber_fraction * prev_magnitude  # 5% of current force
#         else:
#             # For very small forces, use absolute threshold
#             jnd_threshold = 0.05  # 50mN absolute threshold
        
#         # Only update if change exceeds JND (perceptible change)
#         if diff_magnitude > jnd_threshold:
#             self.previous_sent_force = new_force.copy()
#             rospy.logdebug(f"Weber: Force updated - change {diff_magnitude:.4f}N > threshold {jnd_threshold:.4f}N")
#             return new_force
#         else:
#             # Change not perceptible, keep sending previous force
#             rospy.logdebug(f"Weber: Force held - change {diff_magnitude:.4f}N < threshold {jnd_threshold:.4f}N")
#             return self.previous_sent_force

#     def saturate_force(self, force):
#         """
#         Saturate force to stay within haptic device limits.
        
#         Args:
#             force: Input force vector
            
#         Returns:
#             Saturated force vector
#         """
#         force_magnitude = np.linalg.norm(force)
#         max_allowed = self.max_haptic_force * self.force_safety_margin
        
#         if force_magnitude > max_allowed:
#             # Scale down to maximum allowed
#             saturated_force = force * (max_allowed / force_magnitude)
#             rospy.logwarn_throttle(1.0, f"Force saturated from {force_magnitude:.3f}N to {max_allowed:.3f}N")
#             return saturated_force
        
#         return force

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.plot_start_time is None:
#             self.plot_start_time = current_time
#         self.time_stamps.append(current_time - self.plot_start_time)
#         raw_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         self.robot_force_plot.append(raw_force)
#         self.raw_force_plot.append(raw_force.copy())
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
#         if rospy.get_time() - self.start_time < 1:
#             rospy.loginfo("booting")
#         # Direct force reading - impedance control handles smoothing
#         self.robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

#     def haptic_force_callback(self, event):
#         """
#         Main force processing pipeline:
        
#         FT Sensor (Raw Force)
#                ↓
#         [Coordinate Transform] - Convert to base frame
#                ↓
#         [Impedance Control] - Smooth transitions, eliminate bounce
#                ↓
#         [Weber Fraction] - Filter imperceptible changes (5% JND)
#                ↓
#         [Saturation] - Limit to 3N max
#                ↓
#         Internal Topics → Pose Code → Master Side → Haptic Device
#         """
#         if self.shutdown_flag:
#             return

#         # Get current time and calculate dt
#         current_time = rospy.get_time()
#         if self.last_time is None:
#             self.last_time = current_time
#             dt = 0.001  # Initial dt
#         else:
#             dt = current_time - self.last_time
#             self.last_time = current_time
            
#         # Sanity check on dt
#         if dt > 0.1:  # More than 100ms is suspicious
#             rospy.logwarn(f"Large dt detected: {dt*1000:.1f}ms")
#             dt = 0.001

#         # Step 1: Get raw force from robot (with coordinate transformation)
#         raw_robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        
#         # ========== NEW: LOG RAW FORCE ==========
#         # This is the force after coordinate transform but BEFORE impedance/weber
#         current_log_time = rospy.get_time() - self.start_time  # Time in seconds since start
#         self.raw_force_log.append((current_log_time, raw_robot_force[0], raw_robot_force[1], raw_robot_force[2]))
        
#         # Step 2: Apply impedance control for smooth transitions
#         # This removes bounce while preserving object characteristics
#         smoothed_force = self.apply_transition_smoothing(raw_robot_force, dt)
        
#         # Step 3: Apply Weber fraction (perceptual filtering)
#         # Only send updates if change is perceptible (> 5% JND)
#         perceptually_filtered_force = self.apply_weber_fraction(smoothed_force)
        
#         # Step 4: Saturate to haptic device limits (3N max)
#         final_force = self.saturate_force(perceptually_filtered_force)
        
#         # ========== NEW: LOG FINAL FORCE ==========
#         # This is the force AFTER impedance + weber + saturation (sent to haptic)
#         self.final_force_log.append((current_log_time, final_force[0], final_force[1], final_force[2]))
        
#         # Create force message (Vector3)
#         force_msg = Vector3()
#         force_msg.x = final_force[0]
#         force_msg.y = final_force[1]
#         force_msg.z = final_force[2]
        
#         # Publish to internal topic (pose code will bundle this)
#         self.internal_force_pub.publish(force_msg)

#         # Publish force timestamp to internal topic (ROS time)
#         force_timestamp = rospy.get_time()
#         self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

#         # Store ROS time for RTD calculation
#         self.sent_force_times[force_timestamp] = rospy.get_time()

#         # Logging - use ROS time for consistency
#         gen_time = rospy.get_time()
#         self.force_sent_timestamps.append(force_timestamp)
#         diff = force_timestamp - gen_time
#         self.force_gen_send_log.append((gen_time, force_timestamp, diff))

#         # Store haptic force for plotting/logging
#         self.haptic_force = final_force

#         # Update plots and lists
#         self.update_list()

#     def plot_data(self):
#         times = np.array(self.time_stamps)
#         raw_force_plot = np.array(self.raw_force_plot)
#         haptic_force_plot = np.array(self.haptic_force_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, raw_force_plot[:, 0], label='raw force', alpha=0.5, linewidth=1)
#         axs[0, 0].plot(times, haptic_force_plot[:, 0], label='smoothed force', linewidth=2)
#         axs[0, 0].set_title('Force in X (Transition Smoothing)')
#         axs[0, 0].set_ylabel('Newtons')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)
#         axs[0, 0].legend()

#         axs[1, 0].plot(times, raw_force_plot[:, 1], label='raw force', alpha=0.5, linewidth=1)
#         axs[1, 0].plot(times, haptic_force_plot[:, 1], label='smoothed force', linewidth=2)
#         axs[1, 0].set_title('Force in Y (Transition Smoothing)')
#         axs[1, 0].set_ylabel('Newtons')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)
#         axs[1, 0].legend()

#         axs[2, 0].plot(times, raw_force_plot[:, 2], label='raw force', alpha=0.5, linewidth=1)
#         axs[2, 0].plot(times, haptic_force_plot[:, 2], label='smoothed force', linewidth=2)
#         axs[2, 0].set_title('Force in Z (Transition Smoothing)')
#         axs[2, 0].set_ylabel('Newtons')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)
#         axs[2, 0].legend()

#         axs[0, 1].plot(times, raw_force_plot[:, 0] - haptic_force_plot[:, 0])
#         axs[0, 1].set_title('Smoothing Effect in X')
#         axs[0, 1].set_ylabel('Newtons')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)

#         axs[1, 1].plot(times, raw_force_plot[:, 1] - haptic_force_plot[:, 1])
#         axs[1, 1].set_title('Smoothing Effect in Y')
#         axs[1, 1].set_ylabel('Newtons')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)

#         axs[2, 1].plot(times, raw_force_plot[:, 2] - haptic_force_plot[:, 2])
#         axs[2, 1].set_title('Smoothing Effect in Z')
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
#                 'Raw Force X (N)', 'Raw Force Y (N)', 'Raw Force Z (N)',
#                 'Smoothed Haptic Force X (N)', 'Smoothed Haptic Force Y (N)', 'Smoothed Haptic Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 ts_sent = self.force_sent_timestamps[i] if i < len(self.force_sent_timestamps) else ''
#                 writer.writerow([
#                     f"{ts_sent:.9f}" if ts_sent != '' else '',
#                     f"{self.time_stamps[i]:.9f}",
#                     self.raw_force_plot[i][0], self.raw_force_plot[i][1], self.raw_force_plot[i][2],
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

#     def make_csv_for_raw_force(self):
#         """Save raw force (after coordinate transform, before impedance/weber) to CSV"""
#         with open('/home/user/Desktop/delay/raw_force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Time (s)', 'Raw Force X (N)', 'Raw Force Y (N)', 'Raw Force Z (N)'])
#             for time_sec, fx, fy, fz in self.raw_force_log:
#                 writer.writerow([f"{time_sec:.9f}", f"{fx:.6f}", f"{fy:.6f}", f"{fz:.6f}"])
        
#         rospy.loginfo(f"Raw force log saved to: /home/user/Desktop/delay/raw_force.csv")
#         rospy.loginfo(f"  Total samples logged: {len(self.raw_force_log)}")

#     def make_csv_for_final_force(self):
#         """Save final force (after impedance + weber, sent to haptic) to CSV"""
#         with open('/home/user/Desktop/delay/final_force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Time (s)', 'Final Force X (N)', 'Final Force Y (N)', 'Final Force Z (N)'])
#             for time_sec, fx, fy, fz in self.final_force_log:
#                 writer.writerow([f"{time_sec:.9f}", f"{fx:.6f}", f"{fy:.6f}", f"{fz:.6f}"])
        
#         rospy.loginfo(f"Final force log saved to: /home/user/Desktop/delay/final_force.csv")
#         rospy.loginfo(f"  Total samples logged: {len(self.final_force_log)}")

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
#         self.make_csv_for_raw_force()
#         self.make_csv_for_final_force()

# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass


#improve code

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

#         # For initialization parameter - USE ROS TIME
#         self.start_time = rospy.get_time()
#         self.robot_force = np.zeros(3)
#         self.robot_force_initial = np.zeros(3)
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_force = np.zeros(3)
        
#         self.force_sent_timestamps = []
#         self.force_gen_send_log = []
#         self.sent_force_times = {}
        
#         # For RTD logging
#         self.force_rtd_log = []

#         # For logging raw and final forces
#         self.raw_force_log = []  # Stores (timestamp, raw_force_x, raw_force_y, raw_force_z)
#         self.final_force_log = []  # Stores (timestamp, final_force_x, final_force_y, final_force_z)

#         # IMPEDANCE CONTROL PARAMETERS
#         self.target_force = np.zeros(3)  # The force we're smoothly transitioning TO
#         self.current_smooth_force = np.zeros(3)  # Current smoothed force value
        
#         # Impedance parameters (not final force)
#         self.force_damping = 3.0  # N·s/m - Higher = slower, smoother transitions
#         self.force_mass = 0.8  # kg - Virtual mass for force transitions
#         self.force_velocity = np.zeros(3)  # Rate of change of force
        
#         self.last_time = None
        
#         # WEBER FRACTION PARAMETERS 
#         self.weber_fraction = 0.05  # 5% Weber fraction (delta = 0.05)
#         self.previous_sent_force = np.zeros(3)  # Last force sent to haptic device
        
#         # HAPTIC DEVICE LIMITS
#         self.max_haptic_force = 3.0  # Maximum force haptic device can handle (N)
#         self.force_safety_margin = 0.95  # Use 95% of max to stay safe

#         # Shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
#         self.shutdown_flag = False

#         # For plotting
#         self.plot_start_time = None
#         self.time_stamps = []
#         self.robot_force_plot = []
#         self.haptic_force_plot = []
#         self.raw_force_plot = []
        
#         rospy.loginfo("="*60)
#         rospy.loginfo("Force controller initialized with:")
#         rospy.loginfo("  - Impedance Control (transition smoothing)")
#         rospy.loginfo(f"  - Force damping: {self.force_damping} N·s/m")
#         rospy.loginfo(f"  - Force mass: {self.force_mass} kg")
#         rospy.loginfo(f"  - Weber fraction: {self.weber_fraction*100}% (delta = {self.weber_fraction})")
#         rospy.loginfo(f"  - Max haptic force: {self.max_haptic_force} N")
#         rospy.loginfo("  - Processing: FT Sensor → Impedance → Weber → Haptic Device")
#         rospy.loginfo("  - Contact detection: ENABLED (fast response)")
#         rospy.loginfo("="*60)

#     def master_echo_callback(self, msg: MasterToSlaveData):
#         """Receives data FROM master for RTD calculation"""
#         force_timestamp_echo = msg.force_timestamp
        
#         if force_timestamp_echo > 0 and force_timestamp_echo in self.sent_force_times:
#             # Use ROS time consistently for accurate RTD
#             current_time = rospy.get_time()
#             sent_time = self.sent_force_times[force_timestamp_echo]
            
#             # Calculate RTD
#             rtd = current_time - force_timestamp_echo
            
#             # Sanity check
#             if rtd < 0:
#                 rospy.logwarn(f"Negative RTD detected: {rtd*1000:.3f}ms - clock issue!")
#             elif rtd > 1.0:
#                 rospy.logwarn(f"Very large RTD: {rtd*1000:.3f}ms - network issue!")
            
#             self.force_rtd_log.append((force_timestamp_echo, rtd))
#             rospy.loginfo_throttle(1.0, f"Force RTD: {rtd*1000:.3f}ms")
#             del self.sent_force_times[force_timestamp_echo]

#     def apply_transition_smoothing(self, target_force, dt):
#         """
#         Apply impedance-based smoothing to TRANSITIONS only.
#         The final force value equals target_force (preserving hardness).
#         Only the RATE OF CHANGE is controlled (eliminating bounce).
        
#         Special handling:
#         - Fast response on contact onset (immediate force rise)
#         - Fast decay on contact release (no lingering)
        
#         Args:
#             target_force: The actual force from the robot (what we want to reach)
#             dt: Time step
            
#         Returns:
#             Smoothly transitioning force that will eventually equal target_force
#         """
#         if dt <= 0 or dt > 0.1:  # Sanity check on dt
#             return target_force
        
#         # Calculate the force ERROR (how far we are from target)
#         force_error = target_force - self.current_smooth_force
        
#         # Calculate magnitudes
#         target_magnitude = np.linalg.norm(target_force)
#         current_magnitude = np.linalg.norm(self.current_smooth_force)
        
#         # ========== CONTACT ONSET DETECTION (NEW!) ==========
#         # If target force suddenly increases BUT current is still small
#         # This means NEW CONTACT → give immediate response (reduce delay)
#         if target_magnitude > 0.2 and current_magnitude < 0.1:
#             # Jump to 30% of target immediately (faster initial response)
#             self.current_smooth_force = 0.3 * target_force  #original=0.3
#             # Give initial velocity toward target
#             self.force_velocity = 2.0 * (target_force - self.current_smooth_force)
#             rospy.logdebug(f"Contact onset detected! Immediate response: {target_magnitude:.3f}N")
#             return self.current_smooth_force
        
#         # ========== CONTACT RELEASE DETECTION ==========
#         # If target force drops to near zero BUT current is still significant
#         # This means contact RELEASED → fast decay (no lingering)
#         if target_magnitude < 0.1 and current_magnitude > 0.2:
#             # Aggressive decay when releasing contact
#             decay_factor = 0.8  # Decay 20% per step (very fast!)
#             self.current_smooth_force *= decay_factor
#             self.force_velocity *= decay_factor
            
#             rospy.logdebug(f"Contact release detected! Fast decay: {current_magnitude:.3f}N")
            
#             # If close enough to zero, snap to zero immediately
#             if np.linalg.norm(self.current_smooth_force) < 0.05:  # Within 50mN
#                 self.current_smooth_force = np.zeros(3)
#                 self.force_velocity = np.zeros(3)
#                 rospy.logdebug("Snapped to zero force")
            
#             return self.current_smooth_force
        
#         # ========== NORMAL IMPEDANCE DYNAMICS ==========
#         # For all other cases (holding contact, gradual changes)
        
#         # Calculate desired acceleration toward target
#         # Using: a = (error - D*v) / M
#         acceleration = (force_error - self.force_damping * self.force_velocity) / self.force_mass
        
#         # Update velocity (rate of force change)
#         self.force_velocity += acceleration * dt
        
#         # Update current force
#         self.current_smooth_force += self.force_velocity * dt
        
#         # Prevent overshoot - snap to target when very close
#         error_magnitude = np.linalg.norm(force_error)
#         if error_magnitude < 0.01:  # Within 10mN
#             self.current_smooth_force = target_force.copy()
#             self.force_velocity = np.zeros(3)
        
#         # Decay velocity when close to target (prevents oscillation)
#         if error_magnitude < 0.1:  # Within 100mN
#             self.force_velocity *= 0.98
        
#         return self.current_smooth_force

#     def apply_weber_fraction(self, new_force):
#         """
#         Apply Weber fraction to determine if force change is perceptible.
#         Only update haptic device if change exceeds Just Noticeable Difference (JND).
        
#         Weber's Law: ΔI/I = constant (where delta = 0.05 or 5%)
        
#         Args:
#             new_force: New force vector after impedance smoothing
            
#         Returns:
#             Force to send to haptic device (either new or previous)
#         """
#         # Calculate magnitude of previous force
#         prev_magnitude = np.linalg.norm(self.previous_sent_force)
        
#         # Calculate the change in force
#         force_diff = new_force - self.previous_sent_force
#         diff_magnitude = np.linalg.norm(force_diff)
        
#         # Calculate Just Noticeable Difference (JND) threshold
#         # JND = weber_fraction × reference_stimulus
#         if prev_magnitude > 0.01:  # Normal case (force > 10mN)
#             jnd_threshold = self.weber_fraction * prev_magnitude  # 5% of current force
#         else:
#             # For very small forces, use absolute threshold
#             jnd_threshold = 0.05  # 50mN absolute threshold
        
#         # Only update if change exceeds JND (perceptible change)
#         if diff_magnitude > jnd_threshold:
#             self.previous_sent_force = new_force.copy()
#             rospy.logdebug(f"Weber: Force updated - change {diff_magnitude:.4f}N > threshold {jnd_threshold:.4f}N")
#             return new_force
#         else:
#             # Change not perceptible, keep sending previous force
#             rospy.logdebug(f"Weber: Force held - change {diff_magnitude:.4f}N < threshold {jnd_threshold:.4f}N")
#             return self.previous_sent_force

#     def saturate_force(self, force):
#         """
#         Saturate force to stay within haptic device limits.
        
#         Args:
#             force: Input force vector
            
#         Returns:
#             Saturated force vector
#         """
#         force_magnitude = np.linalg.norm(force)
#         max_allowed = self.max_haptic_force * self.force_safety_margin
        
#         if force_magnitude > max_allowed:
#             # Scale down to maximum allowed
#             saturated_force = force * (max_allowed / force_magnitude)
#             rospy.logwarn_throttle(1.0, f"Force saturated from {force_magnitude:.3f}N to {max_allowed:.3f}N")
#             return saturated_force
        
#         return force

#     def update_list(self):
#         if self.shutdown_flag:
#             return
#         current_time = rospy.get_time()
#         if self.plot_start_time is None:
#             self.plot_start_time = current_time
#         self.time_stamps.append(current_time - self.plot_start_time)
#         raw_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
#         self.robot_force_plot.append(raw_force)
#         self.raw_force_plot.append(raw_force.copy())
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
#         if rospy.get_time() - self.start_time < 1:
#             rospy.loginfo("booting")
#         # Direct force reading - impedance control handles smoothing
#         self.robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

#     def haptic_force_callback(self, event):
#         """
#         Main force processing pipeline:
        
#         FT Sensor (Raw Force)
#                ↓
#         [Coordinate Transform] - Convert to base frame
#                ↓
#         [Impedance Control] - Smooth transitions, eliminate bounce, fast contact response
#                ↓
#         [Weber Fraction] - Filter imperceptible changes (5% JND)
#                ↓
#         [Saturation] - Limit to 3N max
#                ↓
#         Internal Topics → Pose Code → Master Side → Haptic Device
#         """
#         if self.shutdown_flag:
#             return

#         # Get current time and calculate dt
#         current_time = rospy.get_time()
#         if self.last_time is None:
#             self.last_time = current_time
#             dt = 0.001  # Initial dt
#         else:
#             dt = current_time - self.last_time
#             self.last_time = current_time
            
#         # Sanity check on dt
#         if dt > 0.1:  # More than 100ms is suspicious
#             rospy.logwarn(f"Large dt detected: {dt*1000:.1f}ms")
#             dt = 0.001

#         # Step 1: Get raw force from robot (with coordinate transformation)
#         raw_robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        
#         # Log raw force (after coordinate transform, before impedance)
#         current_log_time = rospy.get_time() - self.start_time
#         self.raw_force_log.append((current_log_time, raw_robot_force[0], raw_robot_force[1], raw_robot_force[2]))
        
#         # Step 2: Apply impedance control for smooth transitions
#         # NOW WITH FAST CONTACT ONSET RESPONSE!
#         smoothed_force = self.apply_transition_smoothing(raw_robot_force, dt)
        
#         # Step 3: Apply Weber fraction (perceptual filtering)
#         perceptually_filtered_force = self.apply_weber_fraction(smoothed_force)
        
#         # Step 4: Saturate to haptic device limits (3N max)
#         final_force = self.saturate_force(perceptually_filtered_force)
        
#         # Log final force (after impedance + weber + saturation)
#         self.final_force_log.append((current_log_time, final_force[0], final_force[1], final_force[2]))
        
#         # Create force message (Vector3)
#         force_msg = Vector3()
#         force_msg.x = final_force[0]
#         force_msg.y = final_force[1]
#         force_msg.z = final_force[2]
        
#         # Publish to internal topic (pose code will bundle this)
#         self.internal_force_pub.publish(force_msg)

#         # Publish force timestamp to internal topic (ROS time)
#         force_timestamp = rospy.get_time()
#         self.internal_force_timestamp_pub.publish(Float64(force_timestamp))

#         # Store ROS time for RTD calculation
#         self.sent_force_times[force_timestamp] = rospy.get_time()

#         # Logging - use ROS time for consistency
#         gen_time = rospy.get_time()
#         self.force_sent_timestamps.append(force_timestamp)
#         diff = force_timestamp - gen_time
#         self.force_gen_send_log.append((gen_time, force_timestamp, diff))

#         # Store haptic force for plotting/logging
#         self.haptic_force = final_force

#         # Update plots and lists
#         self.update_list()

#     def plot_data(self):
#         times = np.array(self.time_stamps)
#         raw_force_plot = np.array(self.raw_force_plot)
#         haptic_force_plot = np.array(self.haptic_force_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, raw_force_plot[:, 0], label='raw force', alpha=0.5, linewidth=1)
#         axs[0, 0].plot(times, haptic_force_plot[:, 0], label='smoothed force', linewidth=2)
#         axs[0, 0].set_title('Force in X (Transition Smoothing)')
#         axs[0, 0].set_ylabel('Newtons')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)
#         axs[0, 0].legend()

#         axs[1, 0].plot(times, raw_force_plot[:, 1], label='raw force', alpha=0.5, linewidth=1)
#         axs[1, 0].plot(times, haptic_force_plot[:, 1], label='smoothed force', linewidth=2)
#         axs[1, 0].set_title('Force in Y (Transition Smoothing)')
#         axs[1, 0].set_ylabel('Newtons')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)
#         axs[1, 0].legend()

#         axs[2, 0].plot(times, raw_force_plot[:, 2], label='raw force', alpha=0.5, linewidth=1)
#         axs[2, 0].plot(times, haptic_force_plot[:, 2], label='smoothed force', linewidth=2)
#         axs[2, 0].set_title('Force in Z (Transition Smoothing)')
#         axs[2, 0].set_ylabel('Newtons')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)
#         axs[2, 0].legend()

#         axs[0, 1].plot(times, raw_force_plot[:, 0] - haptic_force_plot[:, 0])
#         axs[0, 1].set_title('Smoothing Effect in X')
#         axs[0, 1].set_ylabel('Newtons')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)

#         axs[1, 1].plot(times, raw_force_plot[:, 1] - haptic_force_plot[:, 1])
#         axs[1, 1].set_title('Smoothing Effect in Y')
#         axs[1, 1].set_ylabel('Newtons')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)

#         axs[2, 1].plot(times, raw_force_plot[:, 2] - haptic_force_plot[:, 2])
#         axs[2, 1].set_title('Smoothing Effect in Z')
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
#                 'Raw Force X (N)', 'Raw Force Y (N)', 'Raw Force Z (N)',
#                 'Smoothed Haptic Force X (N)', 'Smoothed Haptic Force Y (N)', 'Smoothed Haptic Force Z (N)'
#             ])
#             for i in range(len(self.time_stamps)):
#                 ts_sent = self.force_sent_timestamps[i] if i < len(self.force_sent_timestamps) else ''
#                 writer.writerow([
#                     f"{ts_sent:.9f}" if ts_sent != '' else '',
#                     f"{self.time_stamps[i]:.9f}",
#                     self.raw_force_plot[i][0], self.raw_force_plot[i][1], self.raw_force_plot[i][2],
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

#     def make_csv_for_raw_force(self):
#         """Save raw force (after coordinate transform, before impedance/weber) to CSV"""
#         with open('/home/user/Desktop/delay/raw_force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Time (s)', 'Raw Force X (N)', 'Raw Force Y (N)', 'Raw Force Z (N)'])
#             for time_sec, fx, fy, fz in self.raw_force_log:
#                 writer.writerow([f"{time_sec:.9f}", f"{fx:.6f}", f"{fy:.6f}", f"{fz:.6f}"])
        
#         rospy.loginfo(f"Raw force log saved to: /home/user/Desktop/delay/raw_force.csv")
#         rospy.loginfo(f"  Total samples logged: {len(self.raw_force_log)}")

#     def make_csv_for_final_force(self):
#         """Save final force (after impedance + weber, sent to haptic) to CSV"""
#         with open('/home/user/Desktop/delay/final_force.csv', 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             writer.writerow(['Time (s)', 'Final Force X (N)', 'Final Force Y (N)', 'Final Force Z (N)'])
#             for time_sec, fx, fy, fz in self.final_force_log:
#                 writer.writerow([f"{time_sec:.9f}", f"{fx:.6f}", f"{fy:.6f}", f"{fz:.6f}"])
        
#         rospy.loginfo(f"Final force log saved to: /home/user/Desktop/delay/final_force.csv")
#         rospy.loginfo(f"  Total samples logged: {len(self.final_force_log)}")

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
#             self.internal_force_pub.publish(zero_msg)  # pyright: ignore[reportUndefinedVariable]
#             time.sleep(0.1)
#         rospy.loginfo("Shutting down, sent zero force.")
        
#         # Save all CSVs
#         self.make_csv()
#         self.make_csv_for_generation_and_send()
#         self.make_csv_for_force_rtd()
#         self.make_csv_for_raw_force()
#         self.make_csv_for_final_force()

# if __name__ == "__main__":
#     try:
#         controller = HapticForceController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass