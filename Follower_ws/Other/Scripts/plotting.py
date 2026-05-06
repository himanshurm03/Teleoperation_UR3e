#!/usr/bin/env python3

# import rospy
# import numpy as np
# from tf2_msgs.msg import TFMessage
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import math
# import tf.transformations as transformations
# from threading import Timer
# import matplotlib.pyplot as plt

# class RobotEndEffectorController:

#     def __init__(self):

#         rospy.init_node('robot_end_effector_controller', anonymous=True)

#         # Provides joint angles of haptic device
#         rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.haptic_offset)

#         # Provides end effector pose of haptic device in task space 
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

#         self.ee_start = True # to set initial offset to the end effector
#         self.haptic_start_1 = True # to set initial offset to the stylus for /phantom/phantom/pose
#         self.haptic_start_2 = True # to set initial offset to the stylus for /phantom/phantom/joint_states
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.haptic_current_linear_pose = np.zeros((3, 1))
#         self.joint_position_robot = np.zeros((6, 1))
#         self.haptic_current_angular_pose = np.zeros((3, 1))

#         # Lists to store data for plotting
#         self.time_stamps = []
#         self.thetas = []
#         self.originals = []
#         self.haptic_current_angular_poses = []
#         self.haptic_stylus_positions = []

#         self.previous_angles_haptic = None
#         self.haptic_stylus_position = np.zeros((3, 1))

#     def unwrap_angle_haptic(self, angle, previous_angle):

#         if previous_angle is None:
#             return angle
        
#         # Calculate the difference between current and previous angles
#         delta = angle - previous_angle

#         # Unwrap the angle difference to be within -pi to pi
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi

#         # Adjust the current angle to be continuous with the previous angle
#         return previous_angle + delta

#     def transformation_matrix(self, t1, t2, t3, t4, t5, t6):

#         # Compute the elements of the matrix
#         m00 = -np.sin(t6)*(np.cos(t4)*np.sin(t1) - np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.cos(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) + np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2)))
#         m01 = np.sin(t5)*(np.sin(t1)*np.sin(t4) + np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.cos(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2))
#         m02 = np.cos(t6)*(np.cos(t4)*np.sin(t1) - np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.sin(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) + np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2)))
        
#         m10 = np.sin(t6)*(np.cos(t1)*np.cos(t4) + np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) + np.cos(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) - np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2)))
#         m11 = -np.sin(t5)*(np.cos(t1)*np.sin(t4) - np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.cos(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2))
#         m12 = np.sin(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) - np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2))) - np.cos(t6)*(np.cos(t1)*np.cos(t4) + np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1)))
        
#         m20 = np.cos(t6)*(np.cos(t2 + t3)*np.sin(t5) + np.sin(t2 + t3)*np.cos(t4)*np.cos(t5)) - np.sin(t2 + t3)*np.sin(t4)*np.sin(t6)
#         m21 = np.cos(t2 + t3)*np.cos(t5) - np.sin(t2 + t3)*np.cos(t4)*np.sin(t5)
#         m22 = np.sin(t6)*(np.cos(t2 + t3)*np.sin(t5) + np.sin(t2 + t3)*np.cos(t4)*np.cos(t5)) + np.sin(t2 + t3)*np.cos(t6)*np.sin(t4)
        
#         # Construct the matrix
#         matrix = np.array([
#             [m00, m01, m02],
#             [m10, m11, m12],
#             [m20, m21, m22]
#         ])

#         rotation = R.from_matrix(matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)

#         if self.previous_angles_haptic is not None:
#             # Unwrap each angle to avoid discontinuities
#             euler_angles = self.unwrap_angle_haptic(euler_angles, self.previous_angles_haptic)

#         # Update the previous angles
#         self.previous_angles_haptic = euler_angles

#         #return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [0]])
#         return np.array([[euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])
    
#     # def haptic_offset(self, msg: JointState):

#     #     theta = np.array([msg.position[0], msg.position[1], -msg.position[2], msg.position[3], msg.position[4], msg.position[5]])
#     #     print(theta)
#     #     current_time = rospy.get_time()
#     #     self.time_stamps.append(current_time)
#     #     self.thetas.append(theta)

#     def haptic_callback(self, msg: PoseStamped):
#         try:
#             quat_from_haptic = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
#             euler_from_haptic = transformations.euler_from_quaternion(quat_from_haptic)
#             self.haptic_stylus_position = np.array([[euler_from_haptic[0]],
#                                                     [euler_from_haptic[1]],
#                                                     [euler_from_haptic[2]]])
#             return self.haptic_stylus_position
#         except Exception as e:
#             rospy.logerr("Error in haptic_callback: %s", str(e))

#     def haptic_offset(self, msg: JointState):
#         self.haptic_current_angular_pose = self.transformation_matrix(msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5])
        
#         current_time = rospy.get_time()
#         self.time_stamps.append(current_time)
#         self.haptic_current_angular_poses.append(self.haptic_current_angular_pose.flatten())
#         self.haptic_stylus_positions.append(self.haptic_stylus_position.flatten())
        
#         return self.haptic_current_angular_pose 

#     def plot_data(self):
#         """ Plot the logged data. """

#         times = np.array(self.time_stamps)
#         thetas = np.array(self.thetas)
#         haptic_current_angular_poses = np.array(self.haptic_current_angular_poses)
#         haptic_stylus_positions = np.array(self.haptic_stylus_positions)

#         fig, axs = plt.subplots(3, 2, figsize=(15, 10))
        
#         axs[0,0].plot(times, haptic_current_angular_poses[:,0]-haptic_stylus_positions[:,0])
#         axs[0,0].set_title('angle 1')

#         axs[1,0].plot(times, haptic_current_angular_poses[:,1]-haptic_stylus_positions[:,1])
#         axs[1,0].set_title('angle 2')

#         axs[2,0].plot(times, haptic_current_angular_poses[:,2]-haptic_stylus_positions[:,2])
#         axs[2,0].set_title('angle 3')

#         axs[0,1].plot(times, haptic_current_angular_poses[:,0], times, haptic_stylus_positions[:,0])
#         axs[0,1].set_title('angle 1')
#         axs[0,1].legend(['Transformation', 'Direct'])

#         axs[1,1].plot(times, haptic_current_angular_poses[:,1], times, haptic_stylus_positions[:,1])
#         axs[1,1].set_title('angle 2')
#         axs[1,1].legend(['Transformation', 'Direct'])

#         axs[2,1].plot(times, haptic_current_angular_poses[:,2], times, haptic_stylus_positions[:,2])
#         axs[2,1].set_title('angle 3')
#         axs[2,1].legend(['Transformation', 'Direct'])

#         for ax in axs.flat:
#             ax.set(xlabel='Time', ylabel='angle')
            
#         plt.tight_layout()
#         plt.show()

#     def plot_data1(self):
#         """ Plot the logged data. """

#         times = np.array(self.time_stamps)
#         thetas = np.array(self.thetas)

#         plt.figure()
#         plt.plot(times, thetas)
#         plt.title('Robot joint positions')
#         plt.legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
#         plt.show()


#     def main_loop(self): 
#         rospy.spin()

# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         # Plot the data after the ROS node is shutdown
#         controller.plot_data()

'''for ur3e'''

# import rospy
# import numpy as np
# from tf2_msgs.msg import TFMessage
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import math
# import tf.transformations as transformations
# from threading import Timer
# import matplotlib.pyplot as plt

# class RobotEndEffectorController:

#     def __init__(self):

#         rospy.init_node('robot_end_effector_controller', anonymous=True)

#         # Provides joint angles of haptic device
#         rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.haptic_offset)

#         # Provides end effector pose of haptic device in task space 
#         rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)

#         self.ee_start = True # to set initial offset to the end effector
#         self.haptic_start_1 = True # to set initial offset to the stylus for /phantom/phantom/pose
#         self.haptic_start_2 = True # to set initial offset to the stylus for /phantom/phantom/joint_states
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.haptic_current_linear_pose = np.zeros((3, 1))
#         self.joint_position_robot = np.zeros((6, 1))
#         self.haptic_current_angular_pose = np.zeros((3, 1))

#         # Lists to store data for plotting
#         self.time_stamps = []
#         self.thetas = []
#         self.originals = []
#         self.haptic_current_angular_poses = []
#         self.haptic_stylus_positions = []

#         self.previous_angles_haptic = None
#         self.haptic_stylus_position = np.zeros((3, 1))

#         self.joint_positions = [] 
#         self.timestamps = []

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
#         self.joint_positions.append(self.joint_position_robot)
#         self.timestamps.append(msg.header.stamp.to_sec())
#         # self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         # if self.robot_pose_flag:
#         #     self.initial_ee_pose = self.current_ee_pose
#         #     self.robot_pose_flag = False

#     def unwrap_angle_haptic(self, angle, previous_angle):

#         if previous_angle is None:
#             return angle
        
#         # Calculate the difference between current and previous angles
#         delta = angle - previous_angle

#         # Unwrap the angle difference to be within -pi to pi
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi

#         # Adjust the current angle to be continuous with the previous angle
#         return previous_angle + delta

#     def transformation_matrix(self, t1, t2, t3, t4, t5, t6):

#         # Compute the elements of the matrix
#         m00 = -np.sin(t6)*(np.cos(t4)*np.sin(t1) - np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.cos(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) + np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2)))
#         m01 = np.sin(t5)*(np.sin(t1)*np.sin(t4) + np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.cos(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2))
#         m02 = np.cos(t6)*(np.cos(t4)*np.sin(t1) - np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.sin(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) + np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2)))
        
#         m10 = np.sin(t6)*(np.cos(t1)*np.cos(t4) + np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) + np.cos(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) - np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2)))
#         m11 = -np.sin(t5)*(np.cos(t1)*np.sin(t4) - np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.cos(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2))
#         m12 = np.sin(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) - np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2))) - np.cos(t6)*(np.cos(t1)*np.cos(t4) + np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1)))
        
#         m20 = np.cos(t6)*(np.cos(t2 + t3)*np.sin(t5) + np.sin(t2 + t3)*np.cos(t4)*np.cos(t5)) - np.sin(t2 + t3)*np.sin(t4)*np.sin(t6)
#         m21 = np.cos(t2 + t3)*np.cos(t5) - np.sin(t2 + t3)*np.cos(t4)*np.sin(t5)
#         m22 = np.sin(t6)*(np.cos(t2 + t3)*np.sin(t5) + np.sin(t2 + t3)*np.cos(t4)*np.cos(t5)) + np.sin(t2 + t3)*np.cos(t6)*np.sin(t4)
        
#         # Construct the matrix
#         matrix = np.array([
#             [m00, m01, m02],
#             [m10, m11, m12],
#             [m20, m21, m22]
#         ])

#         rotation = R.from_matrix(matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)

#         if self.previous_angles_haptic is not None:
#             # Unwrap each angle to avoid discontinuities
#             euler_angles = self.unwrap_angle_haptic(euler_angles, self.previous_angles_haptic)

#         # Update the previous angles
#         self.previous_angles_haptic = euler_angles

#         #return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [0]])
#         return np.array([[euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])
    
#     # def haptic_offset(self, msg: JointState):

#     #     theta = np.array([msg.position[0], msg.position[1], -msg.position[2], msg.position[3], msg.position[4], msg.position[5]])
#     #     print(theta)
#     #     current_time = rospy.get_time()
#     #     self.time_stamps.append(current_time)
#     #     self.thetas.append(theta)

#     def haptic_callback(self, msg: PoseStamped):
#         try:
#             quat_from_haptic = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
#             euler_from_haptic = transformations.euler_from_quaternion(quat_from_haptic)
#             self.haptic_stylus_position = np.array([[euler_from_haptic[0]],
#                                                     [euler_from_haptic[1]],
#                                                     [euler_from_haptic[2]]])
#             return self.haptic_stylus_position
#         except Exception as e:
#             rospy.logerr("Error in haptic_callback: %s", str(e))

#     def haptic_offset(self, msg: JointState):
#         self.haptic_current_angular_pose = self.transformation_matrix(msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5])
        
#         current_time = rospy.get_time()
#         self.time_stamps.append(current_time)
#         self.haptic_current_angular_poses.append(self.haptic_current_angular_pose.flatten())
#         self.haptic_stylus_positions.append(self.haptic_stylus_position.flatten())
        
#         return self.haptic_current_angular_pose 

#     def plot_data(self):
#         """ Plot the logged data. """

#         times = np.array(self.time_stamps)
#         thetas = np.array(self.thetas)
#         haptic_current_angular_poses = np.array(self.haptic_current_angular_poses)
#         haptic_stylus_positions = np.array(self.haptic_stylus_positions)

#         fig, axs = plt.subplots(3, 2, figsize=(15, 10))
        
#         axs[0,0].plot(times, haptic_current_angular_poses[:,0]-haptic_stylus_positions[:,0])
#         axs[0,0].set_title('angle 1')

#         axs[1,0].plot(times, haptic_current_angular_poses[:,1]-haptic_stylus_positions[:,1])
#         axs[1,0].set_title('angle 2')

#         axs[2,0].plot(times, haptic_current_angular_poses[:,2]-haptic_stylus_positions[:,2])
#         axs[2,0].set_title('angle 3')

#         axs[0,1].plot(times, haptic_current_angular_poses[:,0], times, haptic_stylus_positions[:,0])
#         axs[0,1].set_title('angle 1')
#         axs[0,1].legend(['Transformation', 'Direct'])

#         axs[1,1].plot(times, haptic_current_angular_poses[:,1], times, haptic_stylus_positions[:,1])
#         axs[1,1].set_title('angle 2')
#         axs[1,1].legend(['Transformation', 'Direct'])

#         axs[2,1].plot(times, haptic_current_angular_poses[:,2], times, haptic_stylus_positions[:,2])
#         axs[2,1].set_title('angle 3')
#         axs[2,1].legend(['Transformation', 'Direct'])

#         for ax in axs.flat:
#             ax.set(xlabel='Time', ylabel='angle')
            
#         plt.tight_layout()
#         plt.show()

#     def plot_data1(self):
#         """ Plot the logged data. """

#         times = np.array(self.time_stamps)
#         thetas = np.array(self.thetas)

#         plt.figure()
#         plt.plot(times, thetas)
#         plt.title('Robot joint positions')
#         plt.legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
#         plt.show()

#     def plot_joint_angles(self):
#         # Convert lists to numpy arrays for easier manipulation
#         timestamps = np.array(self.timestamps)
#         joint_positions = np.array(self.joint_positions)
        
#         # Check if there are enough data points to plot
#         if joint_positions.shape[0] < 2:
#             print("Not enough data to plot.")
#             return

#         # Create a 3x2 subplot layout for the six joints
#         fig, axes = plt.subplots(3, 2, figsize=(10, 8))
#         fig.suptitle('Joint Angles vs. Time')

#         # Titles for each subplot
#         joint_titles = [f"Joint {i+1}" for i in range(6)]

#         # Plot each joint angle
#         for i in range(6):
#             ax = axes[i // 2, i % 2]  # Calculate subplot position
#             ax.plot(timestamps, joint_positions[:, i], label=f"Joint {i+1}")
#             ax.set_title(joint_titles[i])
#             ax.set_xlabel("Time (s)")
#             ax.set_ylabel("Angle (rad)")
#             ax.grid(True)
#             ax.legend()

#         plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout to include title
#         plt.show()

#     def main_loop(self): 
#         rospy.spin()

# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         # Plot the data after the ROS node is shutdown
#         #controller.plot_data()
#         controller.plot_joint_angles()

'''for ploting quaternion'''

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

        # Provides joint angles of haptic device
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.haptic_offset)

        # Provides end effector pose of haptic device in task space 
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)

        self.ee_start = True # to set initial offset to the end effector
        self.haptic_start_1 = True # to set initial offset to the stylus for /phantom/phantom/pose
        self.haptic_start_2 = True # to set initial offset to the stylus for /phantom/phantom/joint_states
        self.initial_ee_pose = np.zeros((6, 1))
        self.current_ee_pose = np.zeros((6, 1))
        self.haptic_current_linear_pose = np.zeros((3, 1))
        self.joint_position_robot = np.zeros((6, 1))
        self.haptic_current_angular_pose = np.zeros((3, 1))

        # Lists to store data for plotting
        self.time_stamps = []
        self.thetas = []
        self.originals = []
        self.haptic_current_angular_poses = []
        self.haptic_stylus_positions = []

        self.previous_angles_haptic = None
        self.haptic_stylus_position = np.zeros((3, 1))

        self.joint_positions = [] 
        self.timestamps = []
        self.quaternion_data = {"x": [], "y": [], "z": [], "w": []}
        self.start_time = rospy.Time.now().to_sec()
        self.time_data = []

    def joint_callback_robot(self, msg: JointState):
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
        self.joint_positions.append(self.joint_position_robot)
        self.timestamps.append(msg.header.stamp.to_sec())
        # self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
        # if self.robot_pose_flag:
        #     self.initial_ee_pose = self.current_ee_pose
        #     self.robot_pose_flag = False

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

    def transformation_matrix(self, t1, t2, t3, t4, t5, t6):

        # Compute the elements of the matrix
        m00 = -np.sin(t6)*(np.cos(t4)*np.sin(t1) - np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.cos(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) + np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2)))
        m01 = np.sin(t5)*(np.sin(t1)*np.sin(t4) + np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.cos(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2))
        m02 = np.cos(t6)*(np.cos(t4)*np.sin(t1) - np.sin(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) - np.sin(t6)*(np.cos(t5)*(np.sin(t1)*np.sin(t4) + np.cos(t4)*(np.cos(t1)*np.sin(t2)*np.sin(t3) - np.cos(t1)*np.cos(t2)*np.cos(t3))) + np.sin(t5)*(np.cos(t1)*np.cos(t2)*np.sin(t3) + np.cos(t1)*np.cos(t3)*np.sin(t2)))
        
        m10 = np.sin(t6)*(np.cos(t1)*np.cos(t4) + np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) + np.cos(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) - np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2)))
        m11 = -np.sin(t5)*(np.cos(t1)*np.sin(t4) - np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.cos(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2))
        m12 = np.sin(t6)*(np.cos(t5)*(np.cos(t1)*np.sin(t4) - np.cos(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1))) - np.sin(t5)*(np.cos(t2)*np.sin(t1)*np.sin(t3) + np.cos(t3)*np.sin(t1)*np.sin(t2))) - np.cos(t6)*(np.cos(t1)*np.cos(t4) + np.sin(t4)*(np.sin(t1)*np.sin(t2)*np.sin(t3) - np.cos(t2)*np.cos(t3)*np.sin(t1)))
        
        m20 = np.cos(t6)*(np.cos(t2 + t3)*np.sin(t5) + np.sin(t2 + t3)*np.cos(t4)*np.cos(t5)) - np.sin(t2 + t3)*np.sin(t4)*np.sin(t6)
        m21 = np.cos(t2 + t3)*np.cos(t5) - np.sin(t2 + t3)*np.cos(t4)*np.sin(t5)
        m22 = np.sin(t6)*(np.cos(t2 + t3)*np.sin(t5) + np.sin(t2 + t3)*np.cos(t4)*np.cos(t5)) + np.sin(t2 + t3)*np.cos(t6)*np.sin(t4)
        
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
    
    # def haptic_offset(self, msg: JointState):

    #     theta = np.array([msg.position[0], msg.position[1], -msg.position[2], msg.position[3], msg.position[4], msg.position[5]])
    #     print(theta)
    #     current_time = rospy.get_time()
    #     self.time_stamps.append(current_time)
    #     self.thetas.append(theta)

    def haptic_callback(self, msg: PoseStamped):
        quat_from_haptic = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        current_time = rospy.Time.now().to_sec() - self.start_time
        self.time_data.append(current_time)
        self.quaternion_data["x"].append(quat_from_haptic[0])
        self.quaternion_data["y"].append(quat_from_haptic[1])
        self.quaternion_data["z"].append(quat_from_haptic[2])
        self.quaternion_data["w"].append(quat_from_haptic[3])

    def haptic_offset(self, msg: JointState):
        self.haptic_current_angular_pose = self.transformation_matrix(msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5])
        
        current_time = rospy.get_time()
        self.time_stamps.append(current_time)
        self.haptic_current_angular_poses.append(self.haptic_current_angular_pose.flatten())
        self.haptic_stylus_positions.append(self.haptic_stylus_position.flatten())
        
        return self.haptic_current_angular_pose 

    def plot_data(self):
        """ Plot the logged data. """

        times = np.array(self.time_stamps)
        thetas = np.array(self.thetas)
        haptic_current_angular_poses = np.array(self.haptic_current_angular_poses)
        haptic_stylus_positions = np.array(self.haptic_stylus_positions)

        fig, axs = plt.subplots(3, 2, figsize=(15, 10))
        
        axs[0,0].plot(times, haptic_current_angular_poses[:,0]-haptic_stylus_positions[:,0])
        axs[0,0].set_title('angle 1')

        axs[1,0].plot(times, haptic_current_angular_poses[:,1]-haptic_stylus_positions[:,1])
        axs[1,0].set_title('angle 2')

        axs[2,0].plot(times, haptic_current_angular_poses[:,2]-haptic_stylus_positions[:,2])
        axs[2,0].set_title('angle 3')

        axs[0,1].plot(times, haptic_current_angular_poses[:,0], times, haptic_stylus_positions[:,0])
        axs[0,1].set_title('angle 1')
        axs[0,1].legend(['Transformation', 'Direct'])

        axs[1,1].plot(times, haptic_current_angular_poses[:,1], times, haptic_stylus_positions[:,1])
        axs[1,1].set_title('angle 2')
        axs[1,1].legend(['Transformation', 'Direct'])

        axs[2,1].plot(times, haptic_current_angular_poses[:,2], times, haptic_stylus_positions[:,2])
        axs[2,1].set_title('angle 3')
        axs[2,1].legend(['Transformation', 'Direct'])

        for ax in axs.flat:
            ax.set(xlabel='Time', ylabel='angle')
            
        plt.tight_layout()
        plt.show()

    def plot_data1(self):
        """ Plot the logged data. """

        times = np.array(self.time_stamps)
        thetas = np.array(self.thetas)

        plt.figure()
        plt.plot(times, thetas)
        plt.title('Robot joint positions')
        plt.legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])
        plt.show()

    def plot_joint_angles(self):
        # Convert lists to numpy arrays for easier manipulation
        timestamps = np.array(self.timestamps)
        joint_positions = np.array(self.joint_positions)
        
        # Check if there are enough data points to plot
        if joint_positions.shape[0] < 2:
            print("Not enough data to plot.")
            return

        # Create a 3x2 subplot layout for the six joints
        fig, axes = plt.subplots(3, 2, figsize=(10, 8))
        fig.suptitle('Joint Angles vs. Time')

        # Titles for each subplot
        joint_titles = [f"Joint {i+1}" for i in range(6)]

        # Plot each joint angle
        for i in range(6):
            ax = axes[i // 2, i % 2]  # Calculate subplot position
            ax.plot(timestamps, joint_positions[:, i], label=f"Joint {i+1}")
            ax.set_title(joint_titles[i])
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Angle (rad)")
            ax.grid(True)
            ax.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout to include title
        plt.show()

    def plot_quaternion(self):
        # Check if there's data to plot
        if not self.time_data:
            rospy.logwarn("No data collected. Exiting without plotting.")
            return

        # Create subplots
        fig, axs = plt.subplots(2, 2, figsize=(10, 8))
        axs = axs.ravel()

        components = ["x", "y", "z", "w"]
        for i, component in enumerate(components):
            axs[i].plot(self.time_data, self.quaternion_data[component], label=f"Quaternion {component}")
            axs[i].set_title(f"Quaternion {component} vs Time")
            axs[i].set_xlabel("Time (s)")
            axs[i].set_ylabel(f"{component} Value")
            axs[i].legend()
            axs[i].grid()

        plt.tight_layout()
        plt.show()

    def main_loop(self): 
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Plot the data after the ROS node is shutdown
        #controller.plot_data()
        #controller.plot_joint_angles()
        controller.plot_quaternion()