#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniFeedback
from collections import deque
import time
import matplotlib.pyplot as plt 
import csv
from geometry_msgs.msg import PoseStamped

class HapticForceController:

    def __init__(self):

        # Initializing node
        rospy.init_node('confined_space', anonymous=True)

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)

        # Subscribers
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

        # For initialization parameter
        self.start_time = rospy.get_time()
        self.robot_force = np.zeros(3)
        self.robot_force_initial = np.zeros(3)
        self.joint_position_robot = np.zeros(6)
        self.haptic_force = np.zeros(3)

        # For moving average filter
        self.haptic_window_size = 100
        self.force_window = deque(maxlen=self.haptic_window_size)

        # Shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        self.shutdown_flag = False

        # For plotting
        self.start_time = rospy.get_time()
        self.time_stamps = []
        self.robot_force_plot = []
        self.haptic_force_plot = []

        # Confined force parameters
        self.haptic_pose_flag = True 
        self.haptic_stylus_position = np.zeros((3, 1))
        self.haptic_stylus_initial_position = np.zeros((3,1))
        self.half_cube_length = 0.05

    def update_list(self):
        if self.shutdown_flag:
            return
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.robot_force_plot.append(self.forcevector_conversion(self.joint_position_robot, self.robot_force))
        self.haptic_force_plot.append(self.haptic_force)    

    def haptic_callback(self, msg: PoseStamped):
        haptic_stylus_position = np.array([[msg.pose.position.x],[msg.pose.position.y],[msg.pose.position.z]])
        if self.haptic_pose_flag:
            self.haptic_stylus_initial_position = haptic_stylus_position
            self.haptic_pose_flag = False
        self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position
        #print("self.haptic_stylus_position",self.haptic_stylus_position)

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

        # correction_factor = (1.34 - 0.5)*sin(angle)*unit_cross_product_sensor
        correction_factor = (1.34+0.15)*sin(angle)*unit_cross_product_sensor

        # g_base = np.array([0, 0, -3.6335+1.5])
        g_base = np.array([0, 0, -3.6335])
        g_sensor = np.dot(R.T, g_base)

        # can be added 0.2 in the z component
        # F_offset = np.array([0, 0, -3.834+0.25+1.5])
        F_offset = np.array([0, 0, -3.834+0.51-0.4+0.02+0.04])

        robot_force = robot_force - g_sensor - F_offset - correction_factor
        return (np.dot(R, robot_force))
    
    def confined_force(self, haptic_stylus_position, robot_force):
        rospy.loginfo(f"haptic_stylus_position shape: {haptic_stylus_position.shape}")
        rospy.loginfo(f"robot_force shape: {robot_force.shape}")
        delta =100
        a = self.half_cube_length
        x = haptic_stylus_position[0, 0]  # Correct indexing for 2D array (3, 1)
        y = haptic_stylus_position[1, 0]  # Accessing the second row, first column
        z = haptic_stylus_position[2, 0]  # Accessing the third row, first column
        
        fx, fy, fz = 0, 0, 0
        
        # if x >= a:
        #     fx = -delta
        # elif x <= -a:
        #     fx = delta
            
        # if y >= a:
        #     fy = -delta
        # elif y <= -a:
        #     fy = delta
            
        # if z >= a:
        #     fz = -delta
        # elif z <= -a:
        #     fz = delta

        if x >= a:
            fx = -delta*(x-a)
        elif x <= -a:
            fx = -delta*(x+a)
            
        if y >= a:
            fy = -delta*(y-a)
        elif y <= -a:
            fy = -delta*(y+a)
            
        if z >= a:
            fz = -delta*(z-a)
        elif z <= -a:
            fz = -delta*(z+a)
        
        robot_force[0, 0] += fx
        robot_force[1, 0] += fy
        robot_force[2, 0] += fz
        
        return robot_force


    def joint_callback_robot(self, msg: JointState):
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])

    def robot_force_callback(self, msg: WrenchStamped):
        if rospy.get_time() - self.start_time < 1:
            rospy.loginfo("booting")
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

    def haptic_force_callback(self, event):
        if self.shutdown_flag:
            return
        try:
            force_pub_msg = OmniFeedback()
            robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
            if len(robot_force.shape) == 1:
                robot_force = robot_force.reshape(-1, 1)
            rospy.loginfo(f"Haptic stylus position: {self.haptic_stylus_position}")
            rospy.loginfo(f"Robot force before confined_force: {robot_force}")
            robot_force = self.confined_force(self.haptic_stylus_position, robot_force)
            rospy.loginfo(f"Robot force after confined_force: {robot_force}")
            force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = robot_force
            self.force_pub.publish(force_pub_msg)
            self.haptic_force = np.array([force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z])
            self.update_list()
        except Exception as e:
            rospy.logerr(f"Error in haptic_force_callback: {e}")

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
        with open('/home/user/experiment_data/exp_c_s_11.5/robot_force_data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time (s)', 'Robot Force X (N)', 'Robot Force Y (N)', 'Robot Force Z (N)',
                             'Haptic Force X (N)', 'Haptic Force Y (N)', 'Haptic Force Z (N)'])
            for i in range(len(self.time_stamps)):
                writer.writerow([self.time_stamps[i], 
                                 self.robot_force_plot[i][0], self.robot_force_plot[i][1], self.robot_force_plot[i][2],
                                 self.haptic_force_plot[i][0], self.haptic_force_plot[i][1], self.haptic_force_plot[i][2]])

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
        #self.make_csv()
        self.plot_data()

if __name__ == "__main__":
    try:
        controller = HapticForceController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass