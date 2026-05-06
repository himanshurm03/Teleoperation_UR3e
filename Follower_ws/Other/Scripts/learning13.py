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

class RobotEndEffectorController:

    def __init__(self):

        rospy.init_node('robot_end_effector_controller', anonymous=True)

        # Publisher that publish joint angle velocity to the manipulator
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

        # Provides end effector pose of haptic device in task space 
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

        # Provides joint angles of the manipulator
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)

        # Provides joint angles of haptic device
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.haptic_offset)

        self.ee_start = True # to set initial offset to the end effector
        self.haptic_start_1 = True # to set initial offset to the stylus for /phantom/phantom/pose
        self.haptic_start_2 = True # to set initial offset to the stylus for /phantom/phantom/joint_states
        self.initial_ee_pose = np.zeros((6, 1))
        self.current_ee_pose = np.zeros((6, 1))
        self.haptic_current_linear_pose = np.zeros((3, 1))
        self.joint_position_robot = np.zeros((6, 1))
        self.haptic_current_angular_pose = np.zeros((3, 1))
        self.haptic_stylus_position = np.zeros((6, 1))
        self.timer_set = False

        # Lists to store data for plotting
        self.time_stamps = []
        self.joint_velocities = []
        self.initial_ee_poses = []
        self.haptic_stylus_poses = []
        self.current_ee_poses = []
        self.joint_robot_positions = []

        # For unwrapping
        self.previous_angles = None

        self.previous_angles_haptic = None
        self.haptic_angular_inital = np.zeros((3, 1))

    def unwrap_angle_haptic(self, angle, previous_angle):

        if previous_angle is None:
            return angle
        
        # Calculate the difference between current and previous angles
        delta = angle - previous_angle

        # Unwrap the angle difference to be within -pi to pi
        delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
        delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
        delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi

        # Adjust the current angle to be continuous with the previous angle
        return previous_angle + delta

    def haptic_offset(self, msg: JointState):
        if self.haptic_start_2:
                self.haptic_angular_inital = self.transformation_matrix(msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5])
                self.haptic_start_2 = False
        self.haptic_current_angular_pose = self.transformation_matrix(msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5]) - self.haptic_angular_inital
        return self.haptic_current_angular_pose 
    
    # def haptic_offset(self, msg: JointState):
    #     if self.haptic_start_2:
    #             self.haptic_angular_inital = self.transformation_matrix(msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], 0)
    #             self.haptic_start_2 = False
    #     self.haptic_current_angular_pose = self.transformation_matrix(msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], 0) - self.haptic_angular_inital
    #     return self.haptic_current_angular_pose 

    # def transformation_matrix(self, t1, t2, t3, t4, t5, t6):
    #     # Compute the elements of the matrix using the new matrix provided
    #     m00 = np.cos(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) - np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2))) - np.sin(t6)*(np.cos(t4)*np.sin(t1) + np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3)))
    #     m01 = -np.cos(t6)*(np.cos(t4)*np.sin(t1) + np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.sin(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) - np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2)))
    #     m02 = np.sin(t5)*(np.sin(t1)*np.sin(t4) - np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.cos(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2))
        
    #     m10 = np.sin(t6)*(np.cos(t1)*np.cos(t4) - np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.cos(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) + np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2)))
    #     m11 = np.cos(t6)*(np.cos(t1)*np.cos(t4) - np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) + np.sin(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) + np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2)))
    #     m12 = -np.sin(t5)*(np.cos(t1)*np.sin(t4) + np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.cos(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2))
        
    #     m20 = np.sin(t2 + t3)*np.sin(t4)*np.sin(t6) - np.cos(t6)*(np.cos(t2 + t3)*np.sin(t5) - np.sin(t2 + t3)*np.cos(t4)*np.cos(t5))
    #     m21 = np.sin(t6)*(np.cos(t2 + t3)*np.sin(t5) - np.sin(t2 + t3)*np.cos(t4)*np.cos(t5)) + np.sin(t2 + t3)*np.cos(t6)*np.sin(t4)
    #     m22 = np.cos(t2 + t3)*np.cos(t5) + np.sin(t2 + t3)*np.cos(t4)*np.sin(t5)
        
    #     # Construct the matrix
    #     matrix = np.array([
    #         [m00, m01, m02],
    #         [m10, m11, m12],
    #         [m20, m21, m22]
    #     ])

    #     rotation = R.from_matrix(matrix)
    #     euler_angles = rotation.as_euler('xyz', degrees=False)
    #     euler_angles = np.array(euler_angles)

    #     if self.previous_angles_haptic is not None:
    #         # Unwrap each angle to avoid discontinuities
    #         euler_angles = self.unwrap_angle_haptic(euler_angles, self.previous_angles_haptic)

    #     # Update the previous angles
    #     self.previous_angles_haptic = euler_angles

    #     #return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [0]])
    #     return np.array([[euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])


    def transformation_matrix(self, t1, t2, t3, t4, t5, t6):
        # Compute the elements of the matrix using the new matrix provided
        m00 = np.cos(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) - np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2))) - np.sin(t6)*(np.cos(t4)*np.sin(t1) + np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3)))
        m01 = -np.cos(t6)*(np.cos(t4)*np.sin(t1) + np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.sin(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) - np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2)))
        m02 = np.sin(t5)*(np.sin(t1)*np.sin(t4) - np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.cos(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2))
        
        m10 = np.sin(t6)*(np.cos(t1)*np.cos(t4) - np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.cos(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) + np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2)))
        m11 = np.cos(t6)*(np.cos(t1)*np.cos(t4) - np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) + np.sin(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) + np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2)))
        m12 = -np.sin(t5)*(np.cos(t1)*np.sin(t4) + np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.cos(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2))
        
        m20 = np.sin(t2 + t3)*np.sin(t4)*np.sin(t6) - np.cos(t6)*(np.cos(t2 + t3)*np.sin(t5) - np.sin(t2 + t3)*np.cos(t4)*np.cos(t5))
        m21 = np.sin(t6)*(np.cos(t2 + t3)*np.sin(t5) - np.sin(t2 + t3)*np.cos(t4)*np.cos(t5)) + np.sin(t2 + t3)*np.cos(t6)*np.sin(t4)
        m22 = np.cos(t2 + t3)*np.cos(t5) + np.sin(t2 + t3)*np.cos(t4)*np.sin(t5)
        
        # Construct the matrix
        matrix = np.array([
            [m00, m01, m02],
            [m10, m11, m12],
            [m20, m21, m22]
        ])

        rotation = R.from_matrix(matrix)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        euler_angles = np.array(euler_angles)

        if self.previous_angles_haptic is not None:
            # Unwrap each angle to avoid discontinuities
            euler_angles = self.unwrap_angle_haptic(euler_angles, self.previous_angles_haptic)

        # Update the previous angles
        self.previous_angles_haptic = euler_angles

        #return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [0]])
        return np.array([[euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])


    def desired_traj(self):

        """ 
        Calculates desired pose for the end effector.

        Parameters:
        JointState: 1x6 list containing joint angles of haptic device.

        Returns:
        self.haptic_current_angular_pose: 3x1 list containing joint angles of haptic device.
        """
        # Concatenating haptic linear and angular pose
        haptic_current_pose = np.vstack((self.haptic_current_linear_pose, self.haptic_current_angular_pose))
        return haptic_current_pose

    def unwrap_angle(self, angle, previous_angle):

        if previous_angle is None:
            return angle
        
        # Calculate the difference between current and previous angles
        delta = angle - previous_angle

        # Unwrap the angle difference to be within -pi to pi
        delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
        delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
        delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi

        # Adjust the current angle to be continuous with the previous angle
        return previous_angle + delta
    
    def Jointspace2GeometricJacobian(self, joint_position_robot):

        """ 
        Canculates geometrical jacobian.

        Parameters:
        self.joint_position_robot: 6x1 matrix containing joint angles of manipulator.

        Returns:
        J_geometrical: 6x6 matrix.
        """
        #print("joint_position_robot:",joint_position_robot)
        self.joint_robot_positions.append(joint_position_robot.flatten())

        t1 = joint_position_robot[0,0]
        t2 = joint_position_robot[1,0]
        t3 = joint_position_robot[2,0]
        t4 = joint_position_robot[3,0]
        t5 = joint_position_robot[4,0]
        t6 = joint_position_robot[5,0]

        # Precompute sin and cos values
        cos = np.cos
        sin = np.sin

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

        """ 
        Calculate joint space velocity.

        Parameters:
        roll, pitch, angles: roll, pitch, yaw angles of end effector.

        Returns:
        joint_space_velocity: 6x1 list.
        """

        alpha = self.current_ee_pose[3,0]
        beta = self.current_ee_pose[4,0]
        gamma = self.current_ee_pose[5,0]
        
        cos = np.cos
        sin = np.sin
        
        # Inverse of matrix M
        J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
                              [0, 1, 0,                    0,           0, 0],
                              [0, 0, 1,                    0,           0, 0],
                              [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
                              [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
                              [0, 0, 0,           -sin(beta),           0, 1]])
        
        J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

        cond_number = np.linalg.cond(J_geometrical)
        print(f"Condition Number: {cond_number}")

        c1 = 2
        c2 = 2
        k = np.array([[c1, 0, 0, 0, 0, 0],
                      [0, c1, 0, 0, 0, 0],
                      [0, 0, c1, 0, 0, 0],
                      [0, 0, 0, c2, 0, 0],
                      [0, 0, 0, 0, c2, 0],
                      [0, 0, 0, 0, 0, c2]])
        
        # Joint space velocity = k * inv(j_go) * inv(M) * (desired - actual)
        # joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position + self.initial_ee_pose - self.current_ee_pose)
        self.haptic_stylus_position = self.desired_traj() 
        joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position  + self.initial_ee_pose - self.current_ee_pose)
        
        # joint_space_velocity = np.linalg.inv(J_analytical) @ k @ (self.haptic_stylus_position + self.initial_ee_pose - self.current_ee_pose)
        #joint_space_velocity = np.linalg.inv(J_analytical) @ k @ np.array([[1], [1], [1], [1], [1], [1]])
        #print(k)
        #print("J_analytical:", J_analytical)
        #joint_space_velocity = np.array([[1], [1], [1], [1], [1], [1]])

        #print("joint_space_velocity:", joint_space_velocity)
        
        #print("haptic_stylus_position:", self.haptic_stylus_position)
        #print("initial_ee_pose:", self.initial_ee_pose)
        #print("current_ee_pose:", self.current_ee_pose)
        #print("delta:",self.haptic_stylus_position + self.initial_ee_pose - self.current_ee_pose)
        print("haptic angular pose:", self.haptic_current_angular_pose+self.haptic_angular_inital)

        # Log data
        current_time = rospy.get_time()
        self.time_stamps.append(current_time)
        self.joint_velocities.append(joint_space_velocity.flatten())
        self.initial_ee_poses.append(self.initial_ee_pose.flatten())
        self.haptic_stylus_poses.append(self.haptic_stylus_position.flatten())
        self.current_ee_poses.append(self.current_ee_pose.flatten())

        return joint_space_velocity.flatten()

    #@staticmethod
    def pose_and_rotation(self,joint_position_robot):

        q1 = joint_position_robot[0,0]
        q2 = joint_position_robot[1,0]
        q3 = joint_position_robot[2,0]
        q4 = joint_position_robot[3,0]
        q5 = joint_position_robot[4,0]
        q6 = joint_position_robot[5,0]

        # Define constants
        a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
        # Precompute sin and cos values
        cos = np.cos
        sin = np.sin

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
        
        # Convert rotation matrix to Euler angles in 'xyz' sequence
        rotation = R.from_matrix(R_matrix)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        euler_angles = np.array(euler_angles)

        if self.previous_angles is not None:
            # Unwrap each angle to avoid discontinuities
            euler_angles = self.unwrap_angle(euler_angles, self.previous_angles)

        # Update the previous angles
        self.previous_angles = euler_angles

        #return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [0]])
        return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])

    def joint_callback_robot(self, msg: JointState):

        """ 
        Callback function of /joint_states topic

        Parameters:
        JointState: 1x6 list containing joint angles of manipulator.

        Returns:
        self.joint_position_context: 6x1 list containing joint angles of manipulator.

        Frequency: 500 Hz
        """
        try:
            #self.joint_position_robot = np.array(msg.position).reshape((6, 1))
            self.joint_position_robot[0,0] = msg.position[2]
            self.joint_position_robot[1,0] = msg.position[1]
            self.joint_position_robot[2,0] = msg.position[0]
            self.joint_position_robot[3,0] = msg.position[3]
            self.joint_position_robot[4,0] = msg.position[4]
            self.joint_position_robot[5,0] = msg.position[5]

            self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)

            if self.ee_start:
                self.initial_ee_pose = self.current_ee_pose
                self.ee_start = False

            #print('joint_position_robot:', self.joint_position_robot)
            return self.joint_position_robot
        
        except Exception as e:
            rospy.logerr("Error in joint_callback_robot: %s", str(e))

    def haptic_callback(self, msg: PoseStamped):

        """ 
        Callback function of /phantom/phantom/pose topic

        Parameters:
        PoseStamped: stylus pose in task space [position, quaternion]

        Returns:
        self.haptic_current_linear_pose: 3x1 matrix containing linear pose of stylus.

        Frequency: 1000 Hz
        """
        
        try:
            if self.haptic_start_1:
                quat_from_haptic_1 = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
                euler_from_haptic_1 = transformations.euler_from_quaternion(quat_from_haptic_1)

                self.h_offset_array = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, euler_from_haptic_1[0], euler_from_haptic_1[1], euler_from_haptic_1[2]])
                self.haptic_start_1 = False

            #quat_from_haptic = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            #euler_from_haptic = transformations.euler_from_quaternion(quat_from_haptic)

            # haptic_stylus_position = np.array([msg.pose.position.x - self.h_offset_array[0],
            #                                 msg.pose.position.y - self.h_offset_array[1],
            #                                 msg.pose.position.z - self.h_offset_array[2],
            #                                 euler_from_haptic[0]- self.h_offset_array[3],
            #                                 euler_from_haptic[1]- self.h_offset_array[4],
            #                                 euler_from_haptic[2]- self.h_offset_array[5]])
            
            #self.haptic_stylus_position = haptic_stylus_position.reshape((6,1))
            #print('haptic_stylus_position:', haptic_stylus_position.reshape((6, 1)))
            #return self.haptic_stylus_position

            # Change in position = current position - initial position
            haptic_stylus_position = np.array([msg.pose.position.x - self.h_offset_array[0],
                                                    msg.pose.position.y - self.h_offset_array[1],
                                                    msg.pose.position.z - self.h_offset_array[2]])

            # Linear pose of stylus
            self.haptic_current_linear_pose = haptic_stylus_position.reshape(3, 1)
        
        except Exception as e:
            rospy.logerr("Error in haptic_callback: %s", str(e))

    def velocity_callback(self, event):
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.rpy2joint_space_vel()
        #print("velocity_callback:", velocity_pub_msg.data.reshape((6,1)))
        self.velocity_pub.publish(velocity_pub_msg)

    def plot_data(self):
        """ Plot the logged data. """
        times = np.array(self.time_stamps)
        
        joint_velocities = np.array(self.joint_velocities)
        initial_ee_poses = np.array(self.initial_ee_poses)
        haptic_stylus_poses = np.array(self.haptic_stylus_poses)
        current_ee_poses = np.array(self.current_ee_poses)
        joint_robot_positions = np.array(self.joint_robot_positions)

        fig, axs = plt.subplots(5, 1, figsize=(10, 15))
        
        axs[0].plot(times, joint_velocities)
        axs[0].set_title('Joint Space Velocities')
        axs[0].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        # axs[1].plot(times[1000:], joint_velocities[1000:, 4:6])
        # axs[1].set_title('Joint Space Velocities')
        # axs[1].legend(['J5', 'J6'])

        axs[1].plot(times, initial_ee_poses)
        axs[1].set_title('Initial End Effector Pose')
        axs[1].legend(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

        axs[2].plot(times, haptic_stylus_poses)
        axs[2].set_title('Haptic Stylus Pose')
        axs[2].legend(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

        # axs[2].plot(times, haptic_stylus_poses[:,3:6])
        # #axs[2].plot(times, haptic_stylus_poses[:,:3])
        # axs[2].set_title('Haptic Stylus Pose')
        # axs[2].legend(['X', 'Y', 'Z'])

        axs[3].plot(times, current_ee_poses)
        axs[3].set_title('Current End Effector Pose')
        axs[3].legend(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

        # axs[3].plot(times, current_ee_poses[:,3:6])
        # #axs[3].plot(times, haptic_stylus_poses[:,:3])
        # axs[3].set_title('Current End Effector Pose')
        # axs[3].legend(['X', 'Y', 'Z'])

        axs[4].plot(times, joint_robot_positions)
        axs[4].set_title('Robot joint positions')
        axs[4].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        plt.tight_layout()
        plt.show()
        print("maximum value for joint velocities", np.max(joint_velocities))

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
        # Plot the data after the ROS node is shutdown
        controller.plot_data()