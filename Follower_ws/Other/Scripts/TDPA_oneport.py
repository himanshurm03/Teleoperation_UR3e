#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped
from collections import deque
import time

class RobotEndEffectorController:

    def __init__(self):
        # Initializing node
        rospy.init_node('TDPA_OnePort', anonymous=True)

        # Publishers
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        rospy.Subscriber('/ft_wrench', WrenchStamped, self.robot_force_callback)

        # For initialization parameter
        self.robot_pose_flag = True
        self.haptic_pose_flag = True 
        self.initial_ee_pose = np.zeros((6, 1))
        self.current_ee_pose = np.zeros((6, 1))
        self.joint_position_robot = np.zeros(6)
        self.haptic_stylus_position = np.zeros((6, 1))
        self.haptic_stylus_initial_position = np.zeros((6,1))
        self.robot_force = np.zeros((1,3))
        self.power = 0.0
        self.start_time = None

        # For unwrapping
        self.robot_previous_angles = None
        self.haptic_previous_angles = None

        # For moving average filter
        self.haptic_window_size = 100
        self.force_window = deque(maxlen=self.haptic_window_size)

        # For TDPA
        self.velocity_n_minus_1 = 0.0
        self.alpha_n_minus_1 = 0.0
        self.Pobs_n = 0.0
        self.Pact_n = 0.0

        # For plotting
        self.time_stamps = []
        self.robot_force_plot = []
        self.velocity_plot = []
        self.power_plot = []
        self.power_TDPA_plot = []

    def update_list(self, robot_force, velocity):
        self.series_one_port_TDPA(robot_force, velocity)
        current_time = rospy.get_time()
        self.time_stamps.append(current_time - self.start_time)
        self.robot_force_plot.append(robot_force)
        self.velocity_plot.append(velocity)
        self.power = self.power + (robot_force[2] * velocity[2])
        self.power_plot.append(self.power)
        self.power_TDPA_plot.append(self.Pact_n)
        
    @staticmethod
    def compute_desired_z_velocity(current_time):
        amplitude = 0.005 
        frequency = 0.5 
        desired_velocity_z = amplitude * np.sin(2 * np.pi * frequency * current_time)
        return desired_velocity_z
    
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
    def Jointspace2GeometricJacobian(joint_position_robot):
        t1, t2, t3, t4, t5, t6 = joint_position_robot
        cos, sin = np.cos, np.sin
        J_geometrical = np.array([
            [
                (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
                (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
                (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
                (533*cos(t2)*cos(t3)*sin(t1))/2500, 
                cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
                sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
                (1707*cos(t2 + t3)*cos(t4))/20000), 
                cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
                (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
                cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
                (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
                (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
                (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
                (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
                0
            ],
            [
                (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
                (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
                (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
                (533*cos(t1)*cos(t2)*cos(t3))/2500, 
                sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
                sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
                (1707*cos(t2 + t3)*cos(t4))/20000), 
                sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
                (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
                sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
                (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
                (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
                (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
                (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
                0
            ],
            [
                0, 
                (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
                (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
                (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
                (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
                (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
                (2371*sin(t2 + t3 + t4 + t5))/20000, 
                - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
                0
            ],
            [
                0, 
                sin(t1), 
                sin(t1), 
                sin(t1), 
                sin(t2 + t3 + t4)*cos(t1), 
                cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
            ],
            [
                0, 
                -cos(t1), 
                -cos(t1), 
                -cos(t1), 
                sin(t2 + t3 + t4)*sin(t1), 
                - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
            ],
            [
                1, 
                0, 
                0, 
                0, 
                -cos(t2 + t3 + t4), 
                -sin(t2 + t3 + t4)*sin(t5)
            ]
        ])
        lamda = 0.001
        return J_geometrical + lamda * np.eye(6)
    
    def rpy2joint_space_vel(self):

        alpha, beta, gamma = self.current_ee_pose[3:6, 0]
        cos, sin = np.cos, np.sin
        
        J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
                              [0, 1, 0,                    0,           0, 0],
                              [0, 0, 1,                    0,           0, 0],
                              [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
                              [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
                              [0, 0, 0,           -sin(beta),           0, 1]])

        J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

        c = 8
        k = np.array([[c, 0, 0, 0, 0, 0],
                      [0, c, 0, 0, 0, 0],
                      [0, 0, c, 0, 0, 0],
                      [0, 0, 0, c, 0, 0],
                      [0, 0, 0, 0, c, 0],
                      [0, 0, 0, 0, 0, c]])
 
        current_time = rospy.get_time()
        ee_pose = self.current_ee_pose - self.initial_ee_pose
        desired_pose = np.array([[0.0],[0.0],[self.compute_desired_z_velocity(current_time - self.start_time)],[0.0],[0.0],[0.0]])
        joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (desired_pose - ee_pose)

        robot_force = self.forcevector_conversion(self.joint_position_robot, self.robot_force)
        self.update_list(robot_force, desired_pose - ee_pose)
        return joint_space_velocity.flatten()

    def pose_and_rotation(self,joint_position_robot):
        q1, q2, q3, q4, q5, q6 = joint_position_robot

        cos = np.cos
        sin = np.sin
        a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
        # Calculate each component of the position
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

        # Compute each element of the rotation matrix
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

    def joint_callback_robot(self, msg: JointState):
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
        self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
        if self.robot_pose_flag:
            self.start_time = rospy.get_time()
            self.initial_ee_pose = self.current_ee_pose
            self.robot_pose_flag = False

    def velocity_callback(self, event):
        if self.start_time is None:
            return
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.rpy2joint_space_vel()
        self.velocity_pub.publish(velocity_pub_msg)

    def robot_force_callback(self, msg: WrenchStamped):
        robot_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_window.append(robot_force)
        self.robot_force = np.mean(self.force_window, axis=0)

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

        correction_factor = (1.34)*sin(angle)*unit_cross_product_sensor

        g_base = np.array([0, 0, -3.6335+0.15])
        g_sensor = np.dot(R.T, g_base)

        # can be added 0.2 in the z component
        F_offset = np.array([0, 0, -3.834])

        robot_force = robot_force - g_sensor - F_offset - correction_factor
        return (np.dot(R, robot_force))
    
    def series_one_port_TDPA(self, robot_force, velocity):
        f2_n = robot_force[2]
        velocity_n = velocity[2]
        velocity_n_minus_1 = self.velocity_n_minus_1
        alpha_n_minus_1 = self.alpha_n_minus_1
        self.velocity_n_minus_1 = velocity[2]

        self.Pobs_n = self.Pobs_n + (f2_n * velocity_n + alpha_n_minus_1 * velocity_n_minus_1 ** 2)
        
        if self.Pobs_n < 0:
            alpha_n = -self.Pobs_n / (velocity_n ** 2)
        else:
            alpha_n = 0

        self.alpha_n_minus_1 = alpha_n
        f1_n = f2_n + alpha_n * velocity_n
        self.Pact_n = self.Pact_n + f1_n * velocity_n 

    def plot_force_data(self):

        times = np.array(self.time_stamps)
        robot_force_plot = np.array(self.robot_force_plot)

        fig, axs = plt.subplots(3, 1, figsize=(16, 9))
        
        axs[0].plot(times, robot_force_plot[:, 0])
        axs[0].set_title('Force in x')
        axs[0].set_ylabel('Newtons')
        axs[0].set_xlabel('Seconds')
        axs[0].grid(True)

        axs[1].plot(times, robot_force_plot[:, 1])
        axs[1].set_title('Force in y')
        axs[1].set_ylabel('Newtons')
        axs[1].set_xlabel('Seconds')
        axs[1].grid(True)
 
        axs[2].plot(times, robot_force_plot[:, 2])
        axs[2].set_title('Force in z')
        axs[2].set_ylabel('Newtons')
        axs[2].set_xlabel('Seconds')
        axs[2].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_velocity_data(self):

        times = np.array(self.time_stamps)
        velocity_plot = np.array(self.velocity_plot)

        fig, axs = plt.subplots(3, 2, figsize=(16, 9))

        axs[0, 0].plot(times, velocity_plot[:, 0])
        axs[0, 0].set_title('Velocity in x')
        axs[0, 0].set_ylabel('Meters/seconds')
        axs[0, 0].set_xlabel('Seconds')
        axs[0, 0].grid(True)
        axs[0, 0].legend()

        axs[1, 0].plot(times, velocity_plot[:, 1])
        axs[1, 0].set_title('Velocity in y')
        axs[1, 0].set_ylabel('Meters/seconds')
        axs[1, 0].set_xlabel('Seconds')
        axs[1, 0].grid(True)
        axs[1, 0].legend()

        axs[2, 0].plot(times, velocity_plot[:, 2])
        axs[2, 0].set_title('Velocity in z')
        axs[2, 0].set_ylabel('Meters/seconds')
        axs[2, 0].set_xlabel('Seconds')
        axs[2, 0].grid(True)
        axs[2, 0].legend()

        axs[0, 1].plot(times, velocity_plot[:, 3])
        axs[0, 1].set_title('Velocity of roll')
        axs[0, 1].set_ylabel('Radians/seconds')
        axs[0, 1].set_xlabel('Seconds')
        axs[0, 1].grid(True)

        axs[1, 1].plot(times, velocity_plot[:, 4])
        axs[1, 1].set_title('Velocity of pitch')
        axs[1, 1].set_ylabel('Radians/seconds')
        axs[1, 1].set_xlabel('Seconds')
        axs[1, 1].grid(True)

        axs[2, 1].plot(times, velocity_plot[:, 5])
        axs[2, 1].set_title('Velocity of yaw')
        axs[2, 1].set_ylabel('Radians/seconds')
        axs[2, 1].set_xlabel('Seconds')
        axs[2, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_power_data(self):
        times = np.array(self.time_stamps)
        power_plot = np.array(self.power_plot)
        plt.figure(figsize=(12, 6))
        plt.plot(times, power_plot, label='Power (Force * Velocity in z)', color='purple')
        plt.title('Power(without TDPA) over Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Power (Watts)')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

    def plot_power_data_TDPA(self):
        times = np.array(self.time_stamps)
        power_plot = np.array(self.power_TDPA_plot)
        plt.figure(figsize=(12, 6))
        plt.plot(times, power_plot, label='Power (Force * Velocity in z)', color='green')
        plt.title('Power(with TDPA) over Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Power (Watts)')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()    

    def main_loop(self):
        rate = 500
        rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.plot_force_data()
        controller.plot_velocity_data()
        controller.plot_power_data()
        controller.plot_power_data_TDPA()