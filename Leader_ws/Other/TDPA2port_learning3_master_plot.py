#!/usr/bin/env python3
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from omni_msgs.msg import OmniFeedback, OmniState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray 
from collections import deque

class MasterController:
    def __init__(self):
        rospy.init_node('master', anonymous=True)

        self.fmd = np.zeros(3)
        self.fmd_corrected = np.zeros(3)
        self.vmc = np.zeros(3)
        self.E_M_out = 0.0
        self.E_M_in = 0.0
        self.E_S_in = 0.0
        self.prev_time = None

        self.window_size = 100
        self.fmd_corrected_buffer = deque(maxlen=self.window_size)
        self.velocity_buffer = deque(maxlen=25)

        # For plotting
        self.time_log = []
        self.diff_energy_log = []
        self.alpha_log = []
        self.vmc_z_log = []  
        self.E_M_out_log = []
        self.E_S_in_log = []
        self.start_time = time.time()
        self.fmd_log = []
        self.fmd_corrected_log = []

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
        self.msg_pub = rospy.Publisher('message_from_master_to_slave', Float64MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('force_from_slave_to_master', Float64MultiArray, self.force_from_slave_to_master_callback)
        rospy.Subscriber('energy_from_slave_to_master', Float64MultiArray, self.energy_from_slave_to_master_callback)
        rospy.Subscriber('/phantom/phantom/state', OmniState, self.velocity_callback)

        # Shutdown hook
        rospy.on_shutdown(self.plot_results)

    def force_from_slave_to_master_callback(self, msg: Float64MultiArray):
        self.fmd = np.array(msg.data)

    def energy_from_slave_to_master_callback(self, msg: Float64MultiArray):
        self.E_S_in = msg.data[0] if msg.data else 0.0

    def log_data(self, alpha, elapsed_time, fmd_corrected, fmd):
        self.time_log.append(elapsed_time)
        self.E_M_out_log.append(self.E_M_out)
        self.E_S_in_log.append(self.E_S_in)
        self.diff_energy_log.append(self.E_M_out - self.E_S_in)
        self.alpha_log.append(alpha)
        self.vmc_z_log.append(self.vmc[2])
        self.fmd_corrected_log.append(fmd_corrected)
        self.fmd_log.append(fmd)

    def apply_TDPA2port(self, E_M_out, E_M_in, E_S_in, fmd, vmc, delta_t):

        fmd_corrected = np.zeros(3)

        # calculating energies
        P1 = fmd[2] * vmc[2]
        if P1 < 0:
            E_M_out -= P1 * delta_t
        elif P1 > 0:
            E_M_in += P1 * delta_t

        # alpha calculation
        alpha = 0.0  
        if delta_t > 0:
            vmc_z_abs = abs(vmc[2])
            if vmc_z_abs >= 0.001:  
                if E_M_out > E_S_in:
                    denominator = delta_t * (vmc[2]**2 + 1e-9) 
                    alpha = (E_M_out - E_S_in) / denominator

        # force correction
        fmd_corrected[2] = fmd[2] + alpha * vmc[2]

        # applying filter
        fmd_corr_val = np.clip(fmd_corrected[2], -0.5, 0.5) 
        self.fmd_corrected_buffer.append(fmd_corr_val)
        fmd_corrected[2] = np.mean(self.fmd_corrected_buffer)

        # calculating energies again
        P2 = fmd_corrected[2] * vmc[2]
        if P2 < 0:
            E_M_out = E_M_out - P2 * delta_t + P1 * delta_t

        return E_M_out, E_M_in, fmd_corrected, alpha

    def velocity_callback(self, msg: OmniState):
        new_velocity = 0.001 * np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        self.velocity_buffer.append(new_velocity)
        self.vmc = np.mean(self.velocity_buffer, axis=0)

        current_time = time.time()
        if self.prev_time is not None:
            delta_t = current_time - self.prev_time
            self.E_M_out, self.E_M_in, self.fmd_corrected, alpha = self.apply_TDPA2port(self.E_M_out, self.E_M_in, self.E_S_in, self.fmd, self.vmc, delta_t)
            elapsed_time = current_time - self.start_time
            self.log_data(alpha, elapsed_time, self.fmd_corrected[2], self.fmd[2])
            print(f"alpha: {alpha:.3f}, E_M_out: {self.E_M_out:.3f}, E_M_in: {self.E_M_in:.3f}, E_S_in: {self.E_S_in:.3f}, delta_t: {delta_t:.3f}, fmd: {self.fmd[2]:.3f}, vmc: {self.vmc[2]:.3f}")
        else:
            self.fmd_corrected = self.fmd

        self.prev_time = current_time

        force_pub_msg = OmniFeedback()
        force_pub_msg.force.x = 0.0
        force_pub_msg.force.y = 0.0
        force_pub_msg.force.z = self.fmd_corrected[2]
        self.force_pub.publish(force_pub_msg) 

        message = Float64MultiArray()
        combined_message = np.concatenate(([self.E_M_in], self.vmc.flatten())).tolist()
        message.data = combined_message
        self.msg_pub.publish(message)

    def plot_results(self):
        plt.figure(figsize=(12, 12))
        
        # --- Subplot 1: Energy Difference ---
        plt.subplot(4, 1, 1)
        plt.plot(self.time_log, self.E_M_out_log, label='E_M_out', color='tab:blue', linewidth=2)
        plt.plot(self.time_log, self.E_S_in_log, label='E_S_in', color='tab:orange', linewidth=2)
        plt.plot(self.time_log, self.diff_energy_log, label='E_M_out - E_S_in', color='tab:red', linestyle='--', linewidth=2)
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Energy (J)', fontsize=12)
        plt.title('Energy Transfer and Difference Over Time', fontsize=14)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(loc='best', fontsize=10)

        # --- Subplot 2: Alpha ---
        plt.subplot(4, 1, 2)
        plt.plot(self.time_log, self.alpha_log, label='Alpha', color='tab:purple', linewidth=2)
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Alpha', fontsize=12)
        plt.title('Adaptive Damping Coefficient (Alpha) Over Time', fontsize=14)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(loc='best', fontsize=10)

        # --- Subplot 3: Z Velocity ---
        plt.subplot(4, 1, 3)
        plt.plot(self.time_log, self.vmc_z_log, label='vmc[2] (Z Velocity)', color='tab:green', linewidth=2)
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Velocity (m/s)', fontsize=12)
        plt.title('Z-Axis Velocity Over Time', fontsize=14)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(loc='best', fontsize=10)

        # --- Subplot 4: Force Modulation Data ---
        plt.subplot(4, 1, 4)
        plt.plot(self.time_log, self.fmd_corrected_log, label='FMD Corrected', color='tab:cyan', linewidth=2)
        plt.plot(self.time_log, self.fmd_log, label='FMD Raw', color='tab:brown', linestyle='--', linewidth=2)
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Force (N)', fontsize=12)
        plt.title('Force Modulation: Corrected vs Raw', fontsize=14)
        plt.grid(True, linestyle='--', alpha=0.6)
        plt.legend(loc='best', fontsize=10)

        plt.tight_layout(pad=2.0)
        plt.show()

    def main_loop(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = MasterController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        controller.plot_results()
