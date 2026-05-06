#!/usr/bin/env python3

import rospy
import numpy as np
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
        self.energy_obs = np.zeros((3,1))
        self.current_time_of_msg = None
        self.previous_time_of_msg = None
        self.alpha_previous = np.zeros((3,1))
        self.previous_haptic_stylus_velocity = np.zeros((3,1))
        self.energy_without_tdpa = np.zeros((3,1))
        self.force_history = deque(maxlen=100)
        self.actual_force = np.zeros((3,1))

        # For plotting
        self.start_time = None
        self.time_stamps = []
        self.stylus_pose_plot = []
        self.stylus_velocity_plot = []
        self.stylus_force_plot = []
        self.observed_energy_plot = []
        self.energy_without_tdpa_plot = []
        self.actual_force_plot = []

        self.force_publisher = rospy.Publisher("/phantom/phantom/force_feedback", OmniFeedback, queue_size=10)
        rospy.Subscriber('/phantom/phantom/state', OmniState, self.haptic_pose_callback)

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
        self.current_time_of_msg = msg.header.stamp.secs + (msg.header.stamp.nsecs*(10**(-9)))

    def update_list(self,force):
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        self.stylus_velocity_plot.append(self.haptic_stylus_velocity.flatten())
        self.stylus_force_plot.append(force.flatten())
        self.observed_energy_plot.append(self.energy_obs.flatten())
        self.energy_without_tdpa_plot.append(self.energy_without_tdpa.flatten())
        self.actual_force_plot.append(self.actual_force.flatten())
        
    @staticmethod
    def haptic_quat2rpy(quaternion):
        rotation = R.from_quat(quaternion)
        resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_dcm()
        resulting_matrix[0, 1] = -resulting_matrix[0, 1]
        resulting_matrix[0, 2] = -resulting_matrix[0, 2]
        resulting_matrix[1, 0] = -resulting_matrix[1, 0]
        resulting_matrix[2, 0] = -resulting_matrix[2, 0]
        euler_rpy= R.from_dcm(resulting_matrix).as_euler('xyz', degrees=False)
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

    @staticmethod
    def force_function(position, velocity):
        x = position[0,0]
        y = position[1,0]
        z = position[2,0]
        dx = velocity[0,0]
        dy = velocity[1,0]
        dz = velocity[2,0]
        a = 0.01
        k = 100
        b = -5
        fx = k * (a - x) - b * dx if x >= a else 0.0
        fy = k * (a - y) - b * dy if y >= a else 0.0
        fz = k * (a - z) - b * dz if z >= a else 0.0
        return np.array([[fx],[fy],[fz]])

    def TDPA_oneport(self, pose, velocity):
        if self.previous_time_of_msg is None:
            self.previous_time_of_msg = self.current_time_of_msg
            return np.zeros((3,1))
        alpha = np.zeros((3,1))
        force = self.force_function(pose, velocity)
        velocity = -velocity
        delta_t = (self.current_time_of_msg - self.previous_time_of_msg)
        self.energy_without_tdpa = self.energy_without_tdpa + np.multiply(force,velocity)*delta_t
        self.energy_obs = self.energy_obs + (np.multiply(force, velocity) + np.multiply(self.alpha_previous,(self.previous_haptic_stylus_velocity**2)))*delta_t
        for i in range(3):
            if self.energy_obs[i,0]<0:
                alpha[i,0] = -self.energy_obs[i,0]/(delta_t*(velocity[i,0]**2)+0.0001)
                alpha[i, 0] = min(alpha[i, 0], 100000) 
            else:
                alpha[i,0] = 0.0
        self.previous_time_of_msg = self.current_time_of_msg
        self.alpha_previous = alpha
        self.previous_haptic_stylus_velocity = velocity
        total_force = force + np.multiply(alpha, velocity)
        self.force_history.append(total_force)
        smoothed_force = np.mean(self.force_history, axis=0)
        self.actual_force = force
        return smoothed_force

    def publisher_callback(self, event):
        if rospy.is_shutdown():
            return
        force = self.TDPA_oneport(self.haptic_stylus_position, self.haptic_stylus_velocity)
        force_msg = OmniFeedback()
        force_msg.force.x = force[0,0]
        force_msg.force.y = force[1,0]
        force_msg.force.z = force[2,0]
        self.force_publisher.publish(force_msg)
        self.update_list(force)

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
        actual_force_plot = np.array(self.actual_force_plot)
      
        fig, axs = plt.subplots(3, 1, figsize=(16, 9))
        
        axs[0].plot(times, stylus_force_plot[:, 0], label='with filter')
        axs[0].plot(times, actual_force_plot[:, 0], label='without filter')
        axs[0].set_title('Force in x')
        axs[0].set_ylabel('Newtons')
        axs[0].set_xlabel('Seconds')
        axs[0].grid(True)
        axs[0].legend()

        axs[1].plot(times, stylus_force_plot[:, 1], label='with filter')
        axs[1].plot(times, actual_force_plot[:, 1], label='without filter')
        axs[1].set_title('Force in y')
        axs[1].set_ylabel('Newtons')
        axs[1].set_xlabel('Seconds')
        axs[1].grid(True)
        axs[1].legend()

        axs[2].plot(times, stylus_force_plot[:, 2], label='with filter')
        axs[2].plot(times, actual_force_plot[:, 2], label='without filter')
        axs[2].set_title('Force in z')
        axs[2].set_ylabel('Newtons')
        axs[2].set_xlabel('Seconds')
        axs[2].grid(True)
        axs[2].legend()

        plt.tight_layout()
        plt.show()

    def plot_energy(self):
        times = np.array(self.time_stamps)
        observed_energy_plot = np.array(self.observed_energy_plot)
        energy_without_tdpa_plot = np.array(self.energy_without_tdpa_plot)
      
        fig, axs = plt.subplots(3, 1, figsize=(16, 9))
        
        axs[0].plot(times, observed_energy_plot[:, 0],label='with TDPA')
        axs[0].plot(times, energy_without_tdpa_plot[:, 0],label='without TDPA')
        axs[0].set_title('Energy in x')
        axs[0].set_ylabel('Nm')
        axs[0].set_xlabel('Seconds')
        axs[0].grid(True)
        axs[0].legend()

        axs[1].plot(times, observed_energy_plot[:, 1],label='with TDPA')
        axs[1].plot(times, energy_without_tdpa_plot[:, 1],label='without TDPA')
        axs[1].set_title('Energy in y')
        axs[1].set_ylabel('Nm')
        axs[1].set_xlabel('Seconds')
        axs[1].grid(True)
        axs[1].legend()

        axs[2].plot(times, observed_energy_plot[:, 2],label='with TDPA')
        axs[2].plot(times, energy_without_tdpa_plot[:, 2],label='without TDPA')
        axs[2].set_title('Energy in z')
        axs[2].set_ylabel('Nm')
        axs[2].set_xlabel('Seconds')
        axs[2].grid(True)
        axs[2].legend()

        plt.tight_layout()
        plt.show()

    def plot_all_for_report(self):
        times = np.array(self.time_stamps)
        stylus_pose_plot = np.array(self.stylus_pose_plot)
        stylus_velocity_plot = np.array(self.stylus_velocity_plot)
        stylus_force_plot = np.array(self.stylus_force_plot)
        actual_force_plot = np.array(self.actual_force_plot)
        observed_energy_plot = np.array(self.observed_energy_plot)
        energy_without_tdpa_plot = np.array(self.energy_without_tdpa_plot)
        fig, axs = plt.subplots(4, 1, figsize=(12, 12))
        axs[0].plot(times, stylus_pose_plot[:, 0], color='blue')
        axs[0].set_title('Pose in x')
        axs[0].set_ylabel('Meters')
        axs[0].set_xlabel('Seconds')
        axs[1].plot(times, stylus_velocity_plot[:, 0], color='orange')
        axs[1].set_title('Velocity in x')
        axs[1].set_ylabel('Meters/Seconds')
        axs[1].set_xlabel('Seconds')
        axs[2].plot(times, stylus_force_plot[:, 0], color='green')
        axs[2].set_title('Force in x')
        axs[2].set_ylabel('Newtons')
        axs[2].set_xlabel('Seconds')
        axs[2].legend()
        axs[3].plot(times, observed_energy_plot[:, 0], label='with TDPA', color='purple')
        axs[3].plot(times, energy_without_tdpa_plot[:, 0], label='without TDPA', color='brown', linestyle='--')
        axs[3].set_title('Energy in x')
        axs[3].set_ylabel('Nm')
        axs[3].set_xlabel('Seconds')
        axs[3].legend()
        plt.tight_layout()
        plt.show()

    def main_loop(self):
        rate = 1000
        rospy.Timer(rospy.Duration(1.0 / rate), self.publisher_callback)
        rospy.spin()

if __name__ == "__main__":
    try:
        node = TDPAmaster()
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        # node.plot_pose()
        # node.plot_velocity()
        # node.plot_force()
        node.plot_energy()
        #node.plot_all_for_report()