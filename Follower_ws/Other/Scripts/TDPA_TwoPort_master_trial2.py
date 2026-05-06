#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from omni_msgs.msg import OmniFeedback, OmniState
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from collections import deque

class TDPAmaster:

    def __init__(self):
        
        # Initialize node
        rospy.init_node('Master', anonymous=True)

        # Initialize flags and variables
        self.haptic_pose_flag = True
        self.haptic_previous_angles = None
        self.haptic_stylus_position = np.zeros((6, 1))
        self.haptic_stylus_initial_position = np.zeros((6, 1))
        self.haptic_stylus_velocity = np.zeros((3, 1))
        self.previous_time_of_msg = None
        self.force_from_slave = np.zeros((3, 1))
        self.energy_from_slave = np.zeros((3, 1))
        self.Em_in = np.zeros((3, 1))
        self.Em_out = np.zeros((3, 1))
        self.force_history = deque(maxlen=100)

        # For plotting
        self.start_time = None
        self.time_stamps = []
        self.stylus_pose_plot = []
        self.stylus_velocity_plot = []
        self.stylus_force_plot = []
        self.Em_out_plot = []
        self.Em_in_plot = []
        self.energy_from_slave_plot = []

        self.force_publisher = rospy.Publisher("/phantom/phantom/force_feedback", OmniFeedback, queue_size=10)
        self.master_publisher = rospy.Publisher("message_from_master", Float64MultiArray, queue_size=10)
        rospy.Subscriber('/phantom/phantom/state', OmniState, self.haptic_pose_callback)
        rospy.Subscriber('message_from_slave', Float64MultiArray, self.slave_message_callback)

    def slave_message_callback(self, msg: Float64MultiArray):
        message = np.array(msg.data).reshape(6, 1)
        self.force_from_slave[:3, 0] = message[:3, 0]
        self.energy_from_slave[:3, 0] = message[3:, 0]

    def haptic_pose_callback(self, msg: OmniState):
        euler_from_haptic = self.haptic_quat2rpy([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        if self.haptic_previous_angles is not None:
            euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
        self.haptic_previous_angles = euler_from_haptic
        haptic_stylus_position = np.array([[msg.pose.position.x],[msg.pose.position.y],[msg.pose.position.z],[euler_from_haptic[0]],[euler_from_haptic[1]],[euler_from_haptic[2]]])
        if self.haptic_pose_flag:
            self.haptic_stylus_initial_position = haptic_stylus_position
            self.haptic_pose_flag = False
        self.haptic_stylus_position = 0.001*(haptic_stylus_position - self.haptic_stylus_initial_position)
        self.haptic_stylus_velocity = 0.001*np.array([[msg.velocity.x],[msg.velocity.y],[msg.velocity.z]])

    def update_list(self,force):
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        self.stylus_velocity_plot.append(self.haptic_stylus_velocity.flatten())
        self.stylus_force_plot.append(force.flatten())
        self.Em_out_plot.append(self.Em_out.flatten())
        self.Em_in_plot.append(self.Em_in.flatten())
        self.energy_from_slave_plot.append(self.energy_from_slave.flatten())
        
    @staticmethod
    def haptic_quat2rpy(quaternion):
        rotation = R.from_quat(quaternion)
        resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
        resulting_matrix[0, 1] = -resulting_matrix[0, 1]
        resulting_matrix[0, 2] = -resulting_matrix[0, 2]
        resulting_matrix[1, 0] = -resulting_matrix[1, 0]
        resulting_matrix[2, 0] = -resulting_matrix[2, 0]
        euler_rpy= R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
        return euler_rpy

    @staticmethod
    def unwrap_angle(angle, previous_angle):
        if previous_angle is None:
            return angle
        delta = angle - previous_angle
        delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
        delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
        delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
        return previous_angle + delta

    def TDPA_twoport(self, velocity, force, current_time, Es_in_delayed, position):
        if self.previous_time_of_msg is None:
            self.previous_time_of_msg = current_time
            return np.zeros((3,1)), np.zeros((3,1)), np.zeros((3,1)), np.zeros((6,1))
        P = np.zeros((3,1))
        alpha = np.zeros((3,1))
        delta_t = (current_time - self.previous_time_of_msg)
        vm = -velocity
        P = np.multiply(force,vm)
        for i in range(3):
            if P[i,0]>0:
                self.Em_in[i,0] = self.Em_in[i,0] + P[i,0]*delta_t
            elif P[i,0]<0:
                self.Em_out[i,0] = self.Em_out[i,0] - P[i,0]*delta_t
            if self.Em_out[i,0]>Es_in_delayed[i,0] and vm[i,0]!=0:
                alpha[i,0] = (self.Em_out[i,0]-Es_in_delayed[i,0])/(delta_t*(vm[i,0]**2))
        total_force = force + np.multiply(alpha, vm)
        self.force_history.append(total_force)
        smoothed_force = np.mean(self.force_history, axis=0)
        return smoothed_force, self.Em_in, velocity, position
        
    def publisher_callback(self, event):
        force, energy, velocity, position = self.TDPA_twoport(self.haptic_stylus_velocity, self.force_from_slave, rospy.get_time(), self.energy_from_slave, self.haptic_stylus_position)
        force_msg = OmniFeedback()
        force_msg.force.x, force_msg.force.y, force_msg.force.z = force[:3, 0]
        master_msg = Float64MultiArray()
        message = np.vstack((velocity[:3, 0], energy[:3, 0], position[:6, 0]))
        master_msg.data = message.flatten().tolist()
        self.force_publisher.publish(force_msg)
        self.master_publisher.publish(master_msg)
        self.update_list(force)

    def main_loop(self):
        rate = 1000
        rospy.Timer(rospy.Duration(1.0 / rate), self.publisher_callback)
        rospy.spin()

    def plot_pose(self):
        times = np.array(self.time_stamps)
        stylus_pose_plot = np.array(self.stylus_pose_plot)
        
        fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
        axs[0, 0].plot(times, stylus_pose_plot[:, 0])
        axs[0, 0].set_title('Pose in x')
        axs[0, 0].set_ylabel('Meters')
        axs[0, 0].set_xlabel('Seconds')
        axs[0, 0].grid(True)

        axs[1, 0].plot(times, stylus_pose_plot[:, 1])
        axs[1, 0].set_title('Pose in y')
        axs[1, 0].set_ylabel('Meters')
        axs[1, 0].set_xlabel('Seconds')
        axs[1, 0].grid(True)

        axs[2, 0].plot(times, stylus_pose_plot[:, 2])
        axs[2, 0].set_title('Pose in z')
        axs[2, 0].set_ylabel('Meters')
        axs[2, 0].set_xlabel('Seconds')
        axs[2, 0].grid(True)

        axs[0, 1].plot(times, stylus_pose_plot[:, 3])
        axs[0, 1].set_title('Pose in roll')
        axs[0, 1].set_ylabel('Radians')
        axs[0, 1].set_xlabel('Seconds')
        axs[0, 1].grid(True)

        axs[1, 1].plot(times, stylus_pose_plot[:, 4])
        axs[1, 1].set_title('Pose in pitch')
        axs[1, 1].set_ylabel('Radians')
        axs[1, 1].set_xlabel('Seconds')
        axs[1, 1].grid(True)

        axs[2, 1].plot(times, stylus_pose_plot[:, 5])
        axs[2, 1].set_title('Pose in yaw')
        axs[2, 1].set_ylabel('Radians')
        axs[2, 1].set_xlabel('Seconds')
        axs[2, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_velocity(self):
        times = np.array(self.time_stamps)
        stylus_velocity_plot = np.array(self.stylus_velocity_plot)

        fig, axs = plt.subplots(3, 1, figsize=(16, 9))
        
        axs[0].plot(times, stylus_velocity_plot[:, 0])
        axs[0].set_title('Velocity in x')
        axs[0].set_ylabel('Meters/Seconds')
        axs[0].set_xlabel('Seconds')
        axs[0].grid(True)

        axs[1].plot(times, stylus_velocity_plot[:, 1])
        axs[1].set_title('Velocity in y')
        axs[1].set_ylabel('Meters/Seconds')
        axs[1].set_xlabel('Seconds')
        axs[1].grid(True)

        axs[2].plot(times, stylus_velocity_plot[:, 2])
        axs[2].set_title('Velocity in z')
        axs[2].set_ylabel('Meters/Seconds')
        axs[2].set_xlabel('Seconds')
        axs[2].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_force(self):
        times = np.array(self.time_stamps)
        stylus_force_plot = np.array(self.stylus_force_plot)
      
        fig, axs = plt.subplots(3, 1, figsize=(16, 9))
        
        axs[0].plot(times, stylus_force_plot[:, 0])
        axs[0].set_title('Force in x')
        axs[0].set_ylabel('Meters')
        axs[0].set_xlabel('Seconds')
        axs[0].grid(True)

        axs[1].plot(times, stylus_force_plot[:, 1])
        axs[1].set_title('Force in y')
        axs[1].set_ylabel('Meters')
        axs[1].set_xlabel('Seconds')
        axs[1].grid(True)

        axs[2].plot(times, stylus_force_plot[:, 2])
        axs[2].set_title('Force in z')
        axs[2].set_ylabel('Meters')
        axs[2].set_xlabel('Seconds')
        axs[2].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_energy(self):
        times = np.array(self.time_stamps)
        Em_in_plot = np.array(self.Em_in_plot)
        Em_out_plot = np.array(self.Em_out_plot)
        energy_from_slave_plot = np.array(self.energy_from_slave_plot)
      
        fig, axs = plt.subplots(3, 1, figsize=(16, 9))
        
        axs[0].plot(times, Em_in_plot[:, 0],label='Em_in')
        axs[0].plot(times, Em_out_plot[:, 0],label='Em_out')
        axs[0].plot(times, energy_from_slave_plot[:, 0],label='Es_in delayed')
        axs[0].set_title('Energy in x')
        axs[0].set_ylabel('Nm')
        axs[0].set_xlabel('Seconds')
        axs[0].grid(True)
        axs[0].legend()

        axs[1].plot(times, Em_in_plot[:, 1],label='Em_in')
        axs[1].plot(times, Em_out_plot[:, 1],label='Em_out')
        axs[1].plot(times, energy_from_slave_plot[:, 1],label='Es_in delayed')
        axs[1].set_title('Energy in y')
        axs[1].set_ylabel('Nm')
        axs[1].set_xlabel('Seconds')
        axs[1].grid(True)
        axs[1].legend()

        axs[2].plot(times, Em_in_plot[:, 2],label='Em_in')
        axs[2].plot(times, Em_out_plot[:, 2],label='Em_out')
        axs[2].plot(times, energy_from_slave_plot[:, 2],label='Es_in delayed')
        axs[2].set_title('Energy in z')
        axs[2].set_ylabel('Nm')
        axs[2].set_xlabel('Seconds')
        axs[2].grid(True)
        axs[2].legend()

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    try:
        node = TDPAmaster()
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.plot_pose()
        node.plot_velocity()
        node.plot_force()
        node.plot_energy()
