#!/usr/bin/env python3

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import math
import tf.transformations as transformations
from threading import Timer
import matplotlib.pyplot as plt
from omni_msgs.msg import OmniState, OmniFeedback
from collections import deque
import time 

class RobotEndEffectorController:

    def __init__(self):

        # Initializing node
        rospy.init_node('robot_end_effector_controller', anonymous=True)

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)

        # For initialization parameter
        self.start_time = None
        self.robot_force = np.zeros(3)
        self.haptic_force = np.zeros(3)
        self.haptic_force_inital = np.zeros(3)
        self.joint_position_robot = np.zeros(6)
        self.haptic_force_flag = True
        self.start_time = time.time()

        # For plotting
        self.time_stamps = []
        self.robot_force_plot = []
        self.haptic_force_plot = []

        # For moving average filter
        self.haptic_window_size = 100
        self.force_window = deque(maxlen=self.haptic_window_size)

        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.shutdown_flag = False 

    def update_list(self):
        if self.shutdown_flag:
            return
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.robot_force_plot.append(self.robot_force)
        self.haptic_force_plot.append(self.haptic_force)
    
    @staticmethod
    def forcevector_conversion(joint_position_robot, robot_force):

        q1, q2, q3, q4, q5, q6 = joint_position_robot[0], joint_position_robot[1], joint_position_robot[2], joint_position_robot[3], joint_position_robot[4], joint_position_robot[5]
   
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

        robot_force_converted = np.dot(R,robot_force.reshape((3,1)))
        return robot_force_converted.reshape((3,))
    
    def joint_callback_robot(self, msg: JointState):
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

    def robot_force_callback(self, msg: WrenchStamped):
        if time.time() - self.start_time < 1.0:
            return
        if self.haptic_force_flag:
            self.haptic_force_inital = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            self.haptic_force_flag = False
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]) - self.haptic_force_inital
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

    def haptic_force_callback(self, event):
        if self.shutdown_flag:
            return
        force_pub_msg = OmniFeedback()
        robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        force_pub_msg.force.x = robot_force[0]
        force_pub_msg.force.y = robot_force[1]
        force_pub_msg.force.z = robot_force[2]
        self.force_pub.publish(force_pub_msg)

        print("robot_force", self.robot_force)
        print("haptic_force:", force_pub_msg.force)
        self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])
        self.update_list()

    def plot_data(self):

        times = np.array(self.time_stamps)
        robot_force_plot = np.array(self.robot_force_plot)
        haptic_force_plot = np.array(self.haptic_force_plot)

        min_len = min(len(times), len(robot_force_plot), len(haptic_force_plot))
        times = times[:min_len]
        robot_force_plot = robot_force_plot[:min_len]
        haptic_force_plot = haptic_force_plot[:min_len]

        fig, axs = plt.subplots(3, 1, figsize=(18, 11))

        axs[0].plot(times, robot_force_plot[:, 0], times, haptic_force_plot[:, 0])
        axs[0].set_title('Force in x direction')
        axs[0].legend(['Robot', 'Haptic'])

        axs[1].plot(times, robot_force_plot[:, 1], times, haptic_force_plot[:, 1])
        axs[1].set_title('Force in y direction')
        axs[1].legend(['Robot', 'Haptic'])

        axs[2].plot(times, robot_force_plot[:, 2], times, haptic_force_plot[:, 2])
        axs[2].set_title('Force in z direction')
        axs[2].legend(['Robot', 'Haptic'])
        
        plt.tight_layout()
        plt.show()

    def main_loop(self):
        rate = 1000
        rospy.Timer(rospy.Duration(1.0 / rate), self.haptic_force_callback)
        rospy.spin()

    def shutdown_hook(self):
        self.shutdown_flag = True
        zero_force_msg = OmniFeedback()
        zero_force_msg.force.x = 0
        zero_force_msg.force.y = 0
        zero_force_msg.force.z = 0

        rospy.loginfo("Sending zero force to haptic device.")
        for _ in range(10):
            self.force_pub.publish(zero_force_msg)
            time.sleep(0.1)

        rospy.loginfo("Shutting down, sent zero force to haptic device.")
        self.plot_data()

if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass