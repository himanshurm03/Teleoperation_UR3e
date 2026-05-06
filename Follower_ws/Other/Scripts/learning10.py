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
from geometry_msgs.msg import Twist, Vector3
from collections import deque

class RobotEndEffectorController:

    def __init__(self):

        rospy.init_node('robot_end_effector_controller', anonymous=True)

        # Publisher that publish joint angle velocity to the manipulator
        self.velocity_pub = rospy.Publisher("/twist_controller/command", Twist, queue_size=1)

        # Provides end effector pose of haptic device in task space 
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_callback)

        # Provides joint angles of the manipulator
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)

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

        # For moving average filter
        self.window_size = 100  # Adjust the window size for smoothing
        self.velocity_window = deque(maxlen=self.window_size)

        # Lists to store data for plotting
        self.time_stamps = []
        self.task_velocities = []
        self.initial_ee_poses = []
        self.haptic_stylus_poses = []
        self.current_ee_poses = []
        self.joint_robot_positions = []

        # For unwrapping
        self.previous_angles = None

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
    
    def rpy2task_space_vel(self):

        c1 = 1
        c2 = 1
        k = np.array([[c1, 0, 0, 0, 0, 0],
                      [0, c1, 0, 0, 0, 0],
                      [0, 0, c1, 0, 0, 0],
                      [0, 0, 0, c2, 0, 0],
                      [0, 0, 0, 0, c2, 0],
                      [0, 0, 0, 0, 0, c2]])
  
        task_space_velocity = k @ (self.haptic_stylus_position + self.initial_ee_pose - self.current_ee_pose)

        # Add the new velocity to the window
        self.velocity_window.append(task_space_velocity.flatten())
        
        # Calculate the moving average
        filtered_velocity = np.mean(self.velocity_window, axis=0)
        print("jointspace_velocity:",filtered_velocity.reshape((6, 1)))

        # Log data
        current_time = rospy.get_time()
        self.time_stamps.append(current_time)
        self.task_velocities.append(filtered_velocity)
        self.initial_ee_poses.append(self.initial_ee_pose.flatten())
        self.haptic_stylus_poses.append(self.haptic_stylus_position.flatten())
        self.current_ee_poses.append(self.current_ee_pose.flatten())

        return filtered_velocity

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


        try:
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
            return self.joint_position_robot
        
        except Exception as e:
            rospy.logerr("Error in joint_callback_robot: %s", str(e))

    def haptic_callback(self, msg: PoseStamped):

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
            return self.haptic_stylus_position
        
        except Exception as e:
            rospy.logerr("Error in haptic_callback: %s", str(e))

    def velocity_callback(self, event):
        velocity_pub_msg = Twist()
        # vector = self.rpy2task_space_vel()
        # velocity_pub_msg.linear.x = vector[0]
        # velocity_pub_msg.linear.y = vector[1]
        # velocity_pub_msg.linear.z = vector[2]
        # velocity_pub_msg.angular.x = vector[3]
        # velocity_pub_msg.angular.x = vector[4]
        # velocity_pub_msg.angular.x = vector[5]
        vector = self.rpy2task_space_vel()
        velocity_pub_msg.angular = Vector3(0, 0, 0)
        velocity_pub_msg.linear = Vector3(0, 0, -0.1)
        print("velocity_callback:", velocity_pub_msg)
        self.velocity_pub.publish(velocity_pub_msg)

    def plot_data(self):
        """ Plot the logged data. """
        times = np.array(self.time_stamps)
        
        task_velocities = np.array(self.task_velocities)
        initial_ee_poses = np.array(self.initial_ee_poses)
        haptic_stylus_poses = np.array(self.haptic_stylus_poses)
        current_ee_poses = np.array(self.current_ee_poses)
        joint_robot_positions = np.array(self.joint_robot_positions)

        fig, axs = plt.subplots(4, 1, figsize=(10, 15))
        
        axs[0].plot(times, task_velocities)
        axs[0].set_title('Task Space Velocities')
        axs[0].legend(['x', 'y', 'z', 'Rx', 'Ry', 'Rz'])

        axs[1].plot(times, initial_ee_poses)
        axs[1].set_title('Initial End Effector Pose')
        axs[1].legend(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

        axs[2].plot(times, haptic_stylus_poses)
        axs[2].set_title('Haptic Stylus Pose')
        axs[2].legend(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

        axs[3].plot(times, current_ee_poses)
        axs[3].set_title('Current End Effector Pose')
        axs[3].legend(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])

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
        # Plot the data after the ROS node is shutdown
        controller.plot_data()