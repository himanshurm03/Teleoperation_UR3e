#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import time
import matplotlib.pyplot as plt

class TDPAslave:
    def __init__(self):
        rospy.init_node('Slave_tdpa', anonymous=True)
        self.E_M_in = 0.0
        self.E_S_out = 0.0
        self.E_S_in = 0.0
        self.vsd = np.zeros(3)
        self.fs = np.zeros(3)
        self.prev_time = None
        self.start_time = time.time()

        # For plotting
        self.time_log = []
        self.E_out_minus_E_in_log = []
        self.f_z_log = []
        self.beta_log = []

        self.velocity_publisher = rospy.Publisher("corrected_haptic_velocity", Float64MultiArray, queue_size=1)
        self.energy_publisher = rospy.Publisher("energy_from_slave_to_master", Float64MultiArray, queue_size=1)

        rospy.Subscriber("message_from_master_to_slave", Float64MultiArray, self.velocity_and_energy_callback)
        rospy.Subscriber('corrected_force', Float64MultiArray, self.robot_force_callback)

    @staticmethod
    def E_S_calculation(fs, vsd, E_S_out, E_S_in, delta_t):
        P2 = fs[2]*vsd[2]
        if P2 < 0:
            E_S_out -= delta_t*P2
        elif P2 > 0:
            E_S_in += delta_t*P2
        return E_S_out, E_S_in

    @staticmethod
    def beta_calculation(E_S_out, E_M_in, fs, delta_t):
        if delta_t <= 0 or abs(fs[2]) < 0.1:
            return 0.0
        if E_S_out > E_M_in:
            return (E_S_out - E_M_in) / (delta_t * (fs[2]**2))
        else:
            return 0.0
        
    def robot_force_callback(self, msg: Float64MultiArray):
        self.fs = np.array(msg.data)

    def velocity_and_energy_callback(self, msg: Float64MultiArray):
        self.E_M_in = np.array(msg.data[:1]).item()
        self.vsd = np.array(msg.data[1:])
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if self.prev_time is not None:
            delta_t = current_time - self.prev_time
            self.E_S_out, self.E_S_in = self.E_S_calculation(self.fs, self.vsd, self.E_S_out, self.E_S_in, delta_t)
            beta = self.beta_calculation(self.E_S_out, self.E_M_in, self.fs, delta_t)

            self.time_log.append(elapsed_time)
            self.f_z_log.append(self.fs[2])
            self.E_out_minus_E_in_log.append(self.E_S_out - self.E_M_in)
            self.beta_log.append(beta)
        else:
            beta = 0.0

        self.prev_time = current_time

        corrected_haptic_velocity = self.vsd + beta * self.fs
        print(f"beta: {beta:.4f}, corrected_velocity: {corrected_haptic_velocity[2]:.4f}, abs_force: {abs(self.fs[2]):.4f}")

        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = corrected_haptic_velocity.tolist()
        self.velocity_publisher.publish(velocity_pub_msg)

        energy_pub_msg = Float64MultiArray()
        energy_pub_msg.data = [self.E_S_in]
        self.energy_publisher.publish(energy_pub_msg)

    def plot_data(self):
        plt.figure(figsize=(12, 6))

        # Subplot 1: E_S_out - E_M_in vs Time
        plt.subplot(3, 1, 1)
        plt.plot(self.time_log, self.E_out_minus_E_in_log, label='E_S_out - E_M_in', color='blue')
        plt.axhline(0, color='gray', linestyle='--')
        plt.xlabel("Time (s)")
        plt.ylabel("Energy Diff (J)")
        plt.title("E_S_out - E_M_in vs Time")
        plt.legend()
        plt.grid(True)

        # Subplot 2: Beta vs Time
        plt.subplot(3, 1, 2)
        plt.plot(self.time_log, self.beta_log, label='Beta', color='green')
        plt.xlabel("Time (s)")
        plt.ylabel("Beta")
        plt.title("Beta vs Time")
        plt.legend()
        plt.grid(True)

        # Subplot 2: Force vs Time
        plt.subplot(3, 1, 3)
        plt.plot(self.time_log, self.f_z_log, label='Force', color='orange')
        plt.xlabel("Time (s)")
        plt.ylabel("Force")
        plt.title("Force vs Time")
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.show()

    def main_loop(self):
        rospy.spin()

if __name__ == "__main__":
    node = TDPAslave()
    rospy.on_shutdown(node.plot_data)
    node.main_loop()