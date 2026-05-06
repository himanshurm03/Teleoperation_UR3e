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

        # Provides end effector pose of manipulator in task space
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)

        # Provides end effector pose of haptic device in task space 
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

        # Provides joint angles of the manipulator
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)

        # Provides joint angles of haptic device
        #rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.haptic_offset)

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

        self.coff1 = 0.01  # Filter coefficient for EWA
        self.coff2 = 0.01  # Filter coefficient for EWA

    @staticmethod
    def quaternion_to_euler(quat):

        """ 
        Convert a quaternion into Euler angles (roll, pitch, yaw).

        Parameters:
        quat (list or tuple): A quaternion in the format [w, x, y, z].

        Returns:
        euler (numpy.ndarray): Euler angles in radians, in the order (angle about x, angle about y, angle about z).
        """

        # Create a Rotation object from the quaternion
        rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        
        # Convert the rotation object to Euler angles in the 'XYZ' sequence (intrinsic)
        # When using tf.transformations xyz has been used 
        euler = rotation.as_euler('XYZ', degrees=False)
        
        return euler
    
    #@staticmethod
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

        # End effector coordinate wrt to end effector frame     
        a_end = np.array([0, 0, 0.145, 1])

        # 1 = revolute joint, 0 = prismatic joint
        epsilon = np.array([1, 1, 1, 1, 1, 1])

        # DH parameters (classical), taken from UR official websit
        dh = np.array([[np.pi/2, 0, 0.15185, joint_position_robot[0,0]],
                       [0, -0.24355, 0, joint_position_robot[1,0]],
                       [0, -0.2132, 0, joint_position_robot[2,0]],
                       [np.pi/2, 0, 0.13105, joint_position_robot[3,0]],
                       [-np.pi/2, 0, 0.08535, joint_position_robot[4,0]],
                       [0, 0, 0.0921, joint_position_robot[5,0]]])

        # Loop to calculate coordinate of endeffector wrt base frame
        T = np.eye(4)
        for n in range(dh.shape[0]):
            alp, a, d, theta = dh[n]
            t = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alp), np.sin(theta) * np.sin(alp), a * np.cos(theta)],
                          [np.sin(theta), np.cos(theta) * np.cos(alp), -np.cos(theta) * np.sin(alp), a * np.sin(theta)],
                          [0, np.sin(alp), np.cos(alp), d],
                          [0, 0, 0, 1]])
            T = T @ t

        P = (T @ a_end)[:3]

        # Loop to calculate Jacobian
        T1 = np.eye(4)
        Jv = np.zeros((3, dh.shape[0]))
        Jw = np.zeros((3, dh.shape[0]))

        for n in range(dh.shape[0]):
            alp, a, d, theta = dh[n]
            t = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alp), np.sin(theta) * np.sin(alp), a * np.cos(theta)],
                          [np.sin(theta), np.cos(theta) * np.cos(alp), -np.cos(theta) * np.sin(alp), a * np.sin(theta)],
                          [0, np.sin(alp), np.cos(alp), d],
                          [0, 0, 0, 1]])
            T1 = T1 @ t

            Z = T1[:3, 2]
            oo = T1[:3, 3]
            Jv[:, n] = epsilon[n] * np.cross(Z, P - oo) + (1 - epsilon[n]) * Z
            Jw[:, n] = epsilon[n] * Z

        # A snall constant to remove singularity
        lamda = 0.001

        # Concatenating Jv and Jw to get geometrical jacobian
        J_geometrical = np.vstack((Jv, Jw)) + lamda * np.eye(6)
        return J_geometrical
    
    def rpy2joint_space_vel(self):

        """ 
        Calculate joint space velocity.

        Parameters:
        roll, pitch, angles: roll, pitch, yaw angles of end effector.

        Returns:
        joint_space_velocity: 6x1 list.
        """

        gamma = self.current_ee_pose[3,0]
        beta = self.current_ee_pose[4,0]
        alpha = self.current_ee_pose[5,0]
        
        A = np.array([[1, 0            , np.sin(beta)               ], 
                      [0, np.cos(gamma), -np.cos(beta)*np.sin(gamma)], 
                      [0, np.sin(gamma), np.cos(beta)*np.cos(gamma) ]])

        J_geo2ana = np.block([[np.eye(3), np.zeros((3, 3))],
                              [np.zeros((3, 3)), np.linalg.inv(A)]])
        
        J_analytical = J_geo2ana @ self.Jointspace2GeometricJacobian(self.joint_position_robot)

        c1 = 1
        c2 = 1
        k = np.array([[c1, 0, 0, 0, 0, 0],
                      [0, c1, 0, 0, 0, 0],
                      [0, 0, c1, 0, 0, 0],
                      [0, 0, 0, c2, 0, 0],
                      [0, 0, 0, 0, c2, 0],
                      [0, 0, 0, 0, 0, c2]])
        
        joint_space_velocity = np.linalg.inv(J_analytical) @ k @ (self.haptic_stylus_position + self.initial_ee_pose - self.current_ee_pose)
        #joint_space_velocity = np.linalg.inv(J_analytical) @ k @ np.array([[1], [1], [1], [1], [1], [1]])
        #print(k)
        #print("J_analytical:", J_analytical)
        #joint_space_velocity = np.array([[1], [1], [1], [1], [1], [1]])
        #print("joint_space_velocity:", joint_space_velocity)
        #print("haptic_stylus_position:", self.haptic_stylus_position)
        #print("initial_ee_pose:", self.initial_ee_pose)
        #print("current_ee_pose:", self.current_ee_pose)
        #print("delta:",self.haptic_stylus_position + self.initial_ee_pose - self.current_ee_pose)

        # Log data
        current_time = rospy.get_time()
        self.time_stamps.append(current_time)
        self.joint_velocities.append(joint_space_velocity.flatten())
        self.initial_ee_poses.append(self.initial_ee_pose.flatten())
        self.haptic_stylus_poses.append(self.haptic_stylus_position.flatten())
        self.current_ee_poses.append(self.current_ee_pose.flatten())

        return joint_space_velocity.flatten()

    def tf_callback(self, msg: TFMessage):

        """ 
        Callback function of /tf topic

        Parameters:
        TFMessage: end effector pose in task space [position, quaternion]

        Returns:
        self.current_ee_pose: 6x1 matrix containing pose of end effector at each time stamp.

        Frequency: 500 Hz
        """

        try:
            # End effector pose in task space
            X_pos_ee = msg.transforms[0].transform.translation.x
            Y_pos_ee = msg.transforms[0].transform.translation.y
            Z_pos_ee = msg.transforms[0].transform.translation.z
            W_ori_ee = msg.transforms[0].transform.rotation.w
            X_ori_ee = msg.transforms[0].transform.rotation.x
            Y_ori_ee = msg.transforms[0].transform.rotation.y
            Z_ori_ee = msg.transforms[0].transform.rotation.z

            # Changing quaternion to euler 
            quat_from_robot = [X_ori_ee, Y_ori_ee, Z_ori_ee, W_ori_ee]
            euler_from_robot = transformations.euler_from_quaternion(quat_from_robot)
            # quat_from_robot = [W_ori_ee, X_ori_ee, Y_ori_ee, Z_ori_ee]
            # euler_from_robot = self.quaternion_to_euler(quat_from_robot)
            
            # Current end effector pose in a 6x1 array
            #current_ee_pose = np.array([[X_pos_ee], [Y_pos_ee], [Z_pos_ee], [euler_from_robot[0]], [euler_from_robot[1]], [euler_from_robot[2]]])

            #print('X_pos_ee:', X_pos_ee, 'Y_pos_ee:', Y_pos_ee, 'Z_pos_ee:', Z_pos_ee, 'W_ori_ee:', W_ori_ee, 'X_ori_ee:', X_ori_ee, 'Y_ori_ee:', Y_ori_ee, 'Z_ori_ee:', Z_ori_ee)
            #print('current_ee_pose:', current_ee_pose)

            # Apply low-pass filter to Euler angles
            linear_from_robot = [X_pos_ee, Y_pos_ee, Z_pos_ee]
            if not hasattr(self, 'filtered_euler'):
                self.filtered_euler = np.array(euler_from_robot)
                self.filtered_linear = np.array(linear_from_robot)
            else:
                self.filtered_euler = self.coff1 * np.array(euler_from_robot) + (1 - self.coff1) * self.filtered_euler
                self.filtered_linear = self.coff2 * np.array(linear_from_robot) + (1 - self.coff2) * self.filtered_linear

            # Construct current end effector pose in a 6x1 array
            current_ee_pose = np.array([
                [self.filtered_linear[0]], [self.filtered_linear[1]], [self.filtered_linear[2]],
                [self.filtered_euler[0]], [self.filtered_euler[1]], [self.filtered_euler[2]]
            ])
            
            # Store current end effector pose
            self.current_ee_pose = current_ee_pose

            if self.ee_start and not self.timer_set:
                # Set a timer to store initial_ee_pose after 1 seconds
                Timer(1.0, self.set_initial_ee_pose).start()
                self.timer_set = True

            #print('self.initial_ee_pose :', self.initial_ee_pose )
            self.current_ee_pose = current_ee_pose
            print("current_ee_pose:", self.current_ee_pose)

        except Exception as e:
            rospy.logerr("Error in tf_callback: %s", str(e))

    def set_initial_ee_pose(self):
            
            """ Function to set the initial end effector pose after 2 seconds. """

            self.initial_ee_pose = self.current_ee_pose
            self.ee_start = False
            #print("###################################################")
            #rospy.loginfo(f"Initial EE Pose Set: {self.initial_ee_pose}")

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

            quat_from_haptic = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            euler_from_haptic = transformations.euler_from_quaternion(quat_from_haptic)

            haptic_stylus_position = np.array([msg.pose.position.x - self.h_offset_array[0],
                                            msg.pose.position.y - self.h_offset_array[1],
                                            msg.pose.position.z - self.h_offset_array[2],
                                            euler_from_haptic[0]- self.h_offset_array[3],
                                            euler_from_haptic[1]- self.h_offset_array[4],
                                            euler_from_haptic[2]- self.h_offset_array[5]])
            
            self.haptic_stylus_position = haptic_stylus_position.reshape((6,1))
            #print('haptic_stylus_position:', haptic_stylus_position.reshape((6, 1)))
            return self.haptic_stylus_position
        
        except Exception as e:
            rospy.logerr("Error in haptic_callback: %s", str(e))

    def velocity_callback(self, event):
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.rpy2joint_space_vel()
        #print("velocity_callback:", velocity_pub_msg.data.reshape((6,1)))
        #self.velocity_pub.publish(velocity_pub_msg)

    def plot_data(self):
        """ Plot the logged data. """
        times = np.array(self.time_stamps)
        
        joint_velocities = np.array(self.joint_velocities)
        initial_ee_poses = np.array(self.initial_ee_poses)
        haptic_stylus_poses = np.array(self.haptic_stylus_poses)
        current_ee_poses = np.array(self.current_ee_poses)
        joint_robot_positions = np.array(self.joint_robot_positions)

        fig, axs = plt.subplots(5, 1, figsize=(10, 15))
        
        axs[0].plot(times[1000:], joint_velocities[1000:])
        axs[0].set_title('Joint Space Velocities')
        axs[0].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        # axs[1].plot(times[1000:], joint_velocities[1000:, 4:6])
        # axs[1].set_title('Joint Space Velocities')
        # axs[1].legend(['J5', 'J6'])

        axs[1].plot(times[1000:], initial_ee_poses[1000:])
        axs[1].set_title('Initial End Effector Pose')
        axs[1].legend(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

        axs[2].plot(times[1000:], haptic_stylus_poses[1000:])
        axs[2].set_title('Haptic Stylus Pose')
        axs[2].legend(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

        # axs[2].plot(times, haptic_stylus_poses[:,3:6])
        # #axs[2].plot(times, haptic_stylus_poses[:,:3])
        # axs[2].set_title('Haptic Stylus Pose')
        # axs[2].legend(['X', 'Y', 'Z'])

        axs[3].plot(times[1000:], current_ee_poses[1000:])
        axs[3].set_title('Current End Effector Pose')
        axs[3].legend(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

        # axs[3].plot(times, current_ee_poses[:,3:6])
        # #axs[3].plot(times, haptic_stylus_poses[:,:3])
        # axs[3].set_title('Current End Effector Pose')
        # axs[3].legend(['X', 'Y', 'Z'])

        axs[4].plot(times[1000:], joint_robot_positions[1000:])
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