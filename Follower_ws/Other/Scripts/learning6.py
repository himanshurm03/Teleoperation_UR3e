#!/usr/bin/env python3

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import math
import tf.transformations as transformations
from threading import Timer
import matplotlib.pyplot as plt
from collections import deque
from scipy.signal import butter, filtfilt

class KalmanFilter:
    def __init__(self, n):
        self.n = n
        self.x = np.zeros((n, 1))  # state
        self.P = np.eye(n)  # covariance matrix
        self.F = np.eye(n)  # state transition matrix
        self.H = np.eye(n)  # measurement matrix
        self.Q = np.eye(n) * 0.01  # Increased process noise covariance
        self.R = np.eye(n) * 0.01  # measurement noise covariance

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = self.P - K @ self.H @ self.P

class RobotEndEffectorController:

    def __init__(self):

        # Initialising node
        rospy.init_node('robot_end_effector_controller', anonymous=True)

        # Publisher that publish joint angle velocity to the manipulator
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

        # Provides joint angles of haptic device
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.haptic_jointstate_callback)

        # Provides joint angles of the manipulator
        rospy.Subscriber('/joint_states', JointState, self.robot_jointstate_callback)

        # Initialising variables
        self.haptic_jointstate = np.zeros((6, 1))
        self.robot_jointstate = np.zeros((6,1))
        self.previous_haptic_jointstate = np.zeros((6,1))

        # Kalman filter for each joint
        self.kalman_filters = [KalmanFilter(1) for _ in range(6)]

        # Low-pass filter parameters
        self.fs = 500  # Sampling frequency
        self.cutoff = 2  # Lower cutoff frequency for more noise reduction, Hz
        self.b, self.a = butter(4, self.cutoff / (0.5 * self.fs), btype='low', analog=False)

        # Lists to store data for plotting
        self.time_stamps = []
        self.jointspace_velocity_plot = []
        self.robot_jointstate_plot = []
        self.haptic_jointstate_plot = []
        
    def jointspace_velocity_calculator(self):

        c1 = 50
        c2 = 50
        c3 = 50
        c4 = 0
        c5 = 0
        c6 = 0
        k = np.array([[c1, 0, 0, 0, 0, 0],
                      [0, c2, 0, 0, 0, 0],
                      [0, 0, c3, 0, 0, 0],
                      [0, 0, 0, c4, 0, 0],
                      [0, 0, 0, 0, c5, 0],
                      [0, 0, 0, 0, 0, c6]])

        # jointspace_velocity = k@(self.haptic_jointstate - self.previous_haptic_jointstate)
        jointspace_velocity = k @ (self.haptic_jointstate - self.previous_haptic_jointstate)
        print("jointspace_velocity:",jointspace_velocity)

        current_time = rospy.get_time()
        self.time_stamps.append(current_time)
        self.jointspace_velocity_plot.append(jointspace_velocity.flatten())
        self.robot_jointstate_plot.append(self.robot_jointstate.flatten())
        self.haptic_jointstate_plot.append(self.haptic_jointstate.flatten())
    
        return jointspace_velocity.flatten()
    
    def robot_jointstate_callback(self, msg: JointState):
        self.robot_jointstate = np.array([[msg.position[0]], [msg.position[1]], [msg.position[2]], [msg.position[3]], [msg.position[4]], [msg.position[5]]])
    
    def haptic_jointstate_callback(self, msg: JointState):
        self.previous_haptic_jointstate = self.haptic_jointstate.copy()
        self.haptic_jointstate = np.array([[msg.position[0]], [msg.position[1]], [msg.position[2]], [msg.position[3]], [msg.position[4]], [msg.position[5]]])

    # def haptic_jointstate_callback(self, msg: JointState):
    #     self.previous_haptic_jointstate = self.haptic_jointstate.copy()
    #     raw_haptic_jointstate = np.array([[msg.position[0]], [msg.position[1]], [msg.position[2]], [msg.position[3]], [msg.position[4]], [msg.position[5]]])
        
    #     # Applying Low-Pass Filter
    #     filtered_haptic_jointstate = np.zeros_like(raw_haptic_jointstate)
    #     for i in range(6):
    #         filtered_haptic_jointstate[i] = filtfilt(self.b, self.a, [raw_haptic_jointstate[i]], axis=0)

    #     # Applying Kalman filter
    #     for i in range(6):
    #         self.kalman_filters[i].predict()
    #         self.kalman_filters[i].update(filtered_haptic_jointstate[i])
    #         self.haptic_jointstate[i, 0] = self.kalman_filters[i].x[0, 0]

    def velocity_callback(self, event):
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.jointspace_velocity_calculator()
        #self.velocity_pub.publish(velocity_pub_msg)

    def plot_data(self):
        """ Plot the logged data. """

        times = np.array(self.time_stamps)
        jointspace_velocity_plot = np.array(self.jointspace_velocity_plot)
        robot_jointstate_plot = np.array(self.robot_jointstate_plot)
        haptic_jointstate_plot = np.array(self.haptic_jointstate_plot)

        fig, axs = plt.subplots(3, 1, figsize=(10, 15))
        
        axs[0].plot(times, jointspace_velocity_plot)
        axs[0].set_title('Joint Space Velocities')
        axs[0].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        axs[1].plot(times, robot_jointstate_plot)
        axs[1].set_title('Robot Joint State')
        axs[1].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        axs[2].plot(times, haptic_jointstate_plot)
        axs[2].set_title('Haptic Joint State')
        axs[2].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        plt.tight_layout()
        plt.show()

    def main_loop(self): 
        rate = 500
        rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
        rospy.spin()

if __name__ == "__main__":
    controller = RobotEndEffectorController()
    try:
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.plot_data()