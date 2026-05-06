#!/usr/bin/env python3

#THIS CODE IS WORKING PERFECTLY FINE


# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# import csv

# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = [] 



#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
#         self.time_back_to_master_pose_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=1)
#         self.time_back_to_master_force_pub = rospy.Publisher('time_from_slave_to_master_for_force', Float64, queue_size=1)
#         self.haptic_timestamps = []
#         #self.time_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=1, tcp_nodelay=True)
#         self.time_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=10)

        


#         # Subscribers
#         rospy.Subscriber('pose_from_master_to_slave', PoseStamped, self.haptic_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
#         rospy.Subscriber('time_from_master_to_slave_for_pose', Float64, self.time_from_master_to_slave_for_pose_callback)
#         rospy.Subscriber('time_from_master_to_slave_for_force', Float64, self.time_from_master_to_slave_for_force_callback)



#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta
    
#     def time_from_master_to_slave_for_force_callback(self, msg):
#         self.last_received_master_force_timestamp = msg.data

    
#     # def time_from_master_to_slave_for_pose_callback(self, msg: Float64):
#     #     self.time_pub.publish(msg)
#     #     self.forwarded_time_stamps.append(msg.data)
    
#     def time_from_master_to_slave_for_pose_callback(self, msg):
#         #self.last_received_master_timestamp = msg.data
#         #self.time_back_to_master_pose_pub.publish(msg)
#         self.time_pub.publish(msg.data)  # This sends the same timestamp back to the master:here msg has been changed to msg.data
#         self.forwarded_time_stamps.append(msg.data)

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):

#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8  #Original c ws 8
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     def pose_and_rotation(self,joint_position_robot):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot

#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         # Calculate each component of the position
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         # Compute each element of the rotation matrix
#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles

#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         if self.robot_pose_flag:
#             self.initial_ee_pose = self.current_ee_pose
#             self.robot_pose_flag = False

#     #def haptic_callback(self, msg: PoseStamped):
#     # Store header timestamp from incoming haptic pose
#     #     self.haptic_timestamps.append(msg.header.stamp.to_sec())

#     #     euler_from_haptic = self.haptic_quat2rpy([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
#     #     if self.haptic_previous_angles is not None:
#     #         euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#     #     self.haptic_previous_angles = euler_from_haptic

#     #     haptic_stylus_position = np.array([[msg.pose.position.x],[msg.pose.position.y],[msg.pose.position.z],
#     #                                    [euler_from_haptic[0]],[euler_from_haptic[1]],[euler_from_haptic[2]]])
    
#     #     if self.haptic_pose_flag:
#     #         self.haptic_stylus_initial_position = haptic_stylus_position
#     #         self.haptic_pose_flag = False

#     #     self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#     # # --- Send back the master timestamp for delay calculation ---
#     #     if self.last_received_master_timestamp is not None:
#     #         self.time_pub.publish(Float64(self.last_received_master_timestamp))

#     #     return self.haptic_stylus_position
 
# ##THIS IS THE ORIGINAL HAPTIC CALLBACK FUNCTION
 
#     def haptic_callback(self, msg: PoseStamped):
#     # --- Store and forward header timestamp from master ---
#         timestamp = msg.header.stamp.to_sec()
#         self.haptic_timestamps.append(timestamp)

# # New line
#         self.last_packet_received_time = rospy.get_time()
#     # Publish the same timestamp back to master for delay logging
#         self.time_pub.publish(Float64(timestamp))

#     # --- Pose and orientation processing ---
#         euler_from_haptic = self.haptic_quat2rpy([
#         msg.pose.orientation.x,
#         msg.pose.orientation.y,
#         msg.pose.orientation.z,
#         msg.pose.orientation.w
#     ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#         [msg.pose.position.x],
#         [msg.pose.position.y],
#         [msg.pose.position.z],
#         [euler_from_haptic[0]],
#         [euler_from_haptic[1]],
#         [euler_from_haptic[2]]
#         ])

#     # --- Apply initial offset if first message ---
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#         return self.haptic_stylus_position
# ## ABOVE IS THE ORIGINAL HAPTIC CALL BACK FUNCTION
 


#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy= R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     def plot_error_data(self):

#         times = np.array(self.time_stamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, stylus_pose_plot[:, 0] - ee_pose_plot[:, 0])
#         axs[0, 0].set_title('Error in x')
#         axs[0, 0].set_ylabel('Meters')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)

#         axs[1, 0].plot(times, stylus_pose_plot[:, 1] - ee_pose_plot[:, 1])
#         axs[1, 0].set_title('Error in y')
#         axs[1, 0].set_ylabel('Meters')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)

#         axs[2, 0].plot(times, stylus_pose_plot[:, 2] - ee_pose_plot[:, 2])
#         axs[2, 0].set_title('Error in z')
#         axs[2, 0].set_ylabel('Meters')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)

#         axs[0, 1].plot(times, stylus_pose_plot[:, 3] - ee_pose_plot[:, 3])
#         axs[0, 1].set_title('Error in roll')
#         axs[0, 1].set_ylabel('Radians')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)

#         axs[1, 1].plot(times, stylus_pose_plot[:, 4] - ee_pose_plot[:, 4])
#         axs[1, 1].set_title('Error in pitch')
#         axs[1, 1].set_ylabel('Radians')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)

#         axs[2, 1].plot(times, stylus_pose_plot[:, 5] - ee_pose_plot[:, 5])
#         axs[2, 1].set_title('Error in yaw')
#         axs[2, 1].set_ylabel('Radians')
#         axs[2, 1].set_xlabel('Seconds')
#         axs[2, 1].grid(True)

#         plt.tight_layout()
#         plt.show()

#     def plot_data(self):

#         times = np.array(self.time_stamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, stylus_pose_plot[:, 0], label='haptic')
#         axs[0, 0].plot(times, ee_pose_plot[:, 0], label='robot')
#         axs[0, 0].set_title('x')
#         axs[0, 0].set_ylabel('Meters')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)
#         axs[0, 0].legend()

#         axs[1, 0].plot(times, stylus_pose_plot[:, 1], label='haptic')
#         axs[1, 0].plot(times, ee_pose_plot[:, 1], label='robot')
#         axs[1, 0].set_title('y')
#         axs[1, 0].set_ylabel('Meters')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)
#         axs[1, 0].legend()

#         axs[2, 0].plot(times, stylus_pose_plot[:, 2], label='haptic')
#         axs[2, 0].plot(times, ee_pose_plot[:, 2], label='robot')
#         axs[2, 0].set_title('z')
#         axs[2, 0].set_ylabel('Meters')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)
#         axs[2, 0].legend()

#         axs[0, 1].plot(times, stylus_pose_plot[:, 3], label='haptic')
#         axs[0, 1].plot(times, ee_pose_plot[:, 3], label='robot')
#         axs[0, 1].set_title('roll')
#         axs[0, 1].set_ylabel('Radians')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)
#         axs[0, 1].legend()

#         axs[1, 1].plot(times, stylus_pose_plot[:, 4], label='haptic')
#         axs[1, 1].plot(times, ee_pose_plot[:, 4], label='robot')
#         axs[1, 1].set_title('pitch')
#         axs[1, 1].set_ylabel('Radians')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)
#         axs[1, 1].legend()

#         axs[2, 1].plot(times, stylus_pose_plot[:, 5], label='haptic')
#         axs[2, 1].plot(times, ee_pose_plot[:, 5], label='robot')
#         axs[2, 1].set_title('yaw')
#         axs[2, 1].set_ylabel('Radians')
#         axs[2, 1].set_xlabel('Seconds')
#         axs[2, 1].grid(True)
#         axs[2, 1].legend()

#         plt.tight_layout()
#         plt.show()

#     def plot_planes(self):

#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         fig, axs = plt.subplots(1, 3, figsize=(19, 4.5))
        
#         # Plot the xy plane
#         axs[0].plot(ee_pose_plot[:, 0], ee_pose_plot[:, 1], label='End-Effector')
#         axs[0].plot(stylus_pose_plot[:, 0], stylus_pose_plot[:, 1], label='Stylus')
#         axs[0].set_title('XY Plane')
#         axs[0].set_xlabel('X')
#         axs[0].set_ylabel('Y')
#         axs[0].axis('equal')
#         axs[0].grid(True)
#         axs[0].legend()

#         # Plot the yz plane
#         axs[1].plot(ee_pose_plot[:, 1], ee_pose_plot[:, 2], label='End-Effector')
#         axs[1].plot(stylus_pose_plot[:, 1], stylus_pose_plot[:, 2], label='Stylus')
#         axs[1].set_title('YZ Plane')
#         axs[1].set_xlabel('Y')
#         axs[1].set_ylabel('Z')
#         axs[1].axis('equal')
#         axs[1].grid(True)
#         axs[1].legend()

#         # Plot the zx plane
#         axs[2].plot(ee_pose_plot[:, 2], ee_pose_plot[:, 0], label='End-Effector')
#         axs[2].plot(stylus_pose_plot[:, 2], stylus_pose_plot[:, 0], label='Stylus')
#         axs[2].set_title('ZX Plane')
#         axs[2].set_xlabel('Z')
#         axs[2].set_ylabel('X')
#         axs[2].axis('equal')
#         axs[2].grid(True)
#         axs[2].legend()

#         plt.tight_layout()
#         plt.show()

#     def plot_all_data(self):
#         times = np.array(self.time_stamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         fig, axs = plt.subplots(6, 2, figsize=(20, 24))

#         # Plot error data
#         errors = ['Error in x', 'Error in y', 'Error in z', 'Error in roll', 'Error in pitch', 'Error in yaw']
#         units = ['Meters', 'Meters', 'Meters', 'Radians', 'Radians', 'Radians']
#         for i in range(6):
#             axs[i, 0].plot(times, stylus_pose_plot[:, i] - ee_pose_plot[:, i])
#             axs[i, 0].set_title(errors[i])
#             axs[i, 0].set_ylabel(units[i])
#             axs[i, 0].set_xlabel('Seconds')
#             axs[i, 0].grid(True)

#         # Plot pose data
#         poses = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
#         for i in range(6):
#             axs[i, 1].plot(times, stylus_pose_plot[:, i], label='haptic')
#             axs[i, 1].plot(times, ee_pose_plot[:, i], label='robot')
#             axs[i, 1].set_title(poses[i])
#             axs[i, 1].set_ylabel(units[i])
#             axs[i, 1].set_xlabel('Seconds')
#             axs[i, 1].grid(True)
#             axs[i, 1].legend()

#         plt.tight_layout()
#         plt.show()

#         # Plot planes
#         fig, axs = plt.subplots(1, 3, figsize=(19, 4.5))
        
#         # Plot the xy plane
#         axs[0].plot(ee_pose_plot[:, 0], ee_pose_plot[:, 1], label='End-Effector')
#         axs[0].plot(stylus_pose_plot[:, 0], stylus_pose_plot[:, 1], label='Stylus')
#         axs[0].set_title('XY Plane')
#         axs[0].set_xlabel('X')
#         axs[0].set_ylabel('Y')
#         axs[0].axis('equal')
#         axs[0].grid(True)
#         axs[0].legend()

#         # Plot the yz plane
#         axs[1].plot(ee_pose_plot[:, 1], ee_pose_plot[:, 2], label='End-Effector')
#         axs[1].plot(stylus_pose_plot[:, 1], stylus_pose_plot[:, 2], label='Stylus')
#         axs[1].set_title('YZ Plane')
#         axs[1].set_xlabel('Y')
#         axs[1].set_ylabel('Z')
#         axs[1].axis('equal')
#         axs[1].grid(True)
#         axs[1].legend()

#         # Plot the zx plane
#         axs[2].plot(ee_pose_plot[:, 2], ee_pose_plot[:, 0], label='End-Effector')
#         axs[2].plot(stylus_pose_plot[:, 2], stylus_pose_plot[:, 0], label='Stylus')
#         axs[2].set_title('ZX Plane')
#         axs[2].set_xlabel('Z')
#         axs[2].set_ylabel('X')
#         axs[2].axis('equal')
#         axs[2].grid(True)
#         axs[2].legend()

#         plt.tight_layout()
#         plt.show()


#     def make_csv(self):
#         times = np.array(self.haptic_timestamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
#                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
#                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

#             for i in range(len(times)):
#                 current_time = self.start_time + times[i]  # Just the wall-clock time at which this was recorded

#                 csvwriter.writerow([
#                     times[i],                     
#                     self.time_stamps[i],                  
#                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
#                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
#                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
#                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
#                 ])


#     # def make_csv_packet_times(self):
#     # # Convert to numpy for easier handling
#     #     packet_times = np.array(self.packet_times)  # list of (received, applied, diff)

#     #     with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#     #         csvwriter = csv.writer(csvfile)
#     #         csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#     #         for i in range(len(packet_times)):
#     #             csvwriter.writerow([
#     #                 packet_times[i][0],   # Received_Time
#     #                 packet_times[i][1],   # Applied_Time
#     #                 packet_times[i][2]    # Difference
#     #             ])
    
#     def make_csv_packet_times(self):
#         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#             for received, applied, diff in self.packet_times:
#                 csvwriter.writerow([received, applied, diff])

#     def main_loop(self):
#         rate = 500
#         rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
#         rospy.spin()

# # if __name__ == "__main__":
# #     try:
# #         controller = RobotEndEffectorController()
# #         controller.main_loop()
# #     except rospy.ROSInterruptException:
# #         pass
# #     finally:
# #         controller.make_csv()
# #         controller.make_csv_packet_times()

# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         try:
#             controller.make_csv()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")

#         try:
#             controller.make_csv_packet_times()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")

#         #controller.plot_error_data()
#         #controller.plot_data()
#         #controller.plot_planes()
#         #controller.plot_all_data()


#MESSAGE
#!/usr/bin/env python3

# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Vector3 
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData  # ADD THIS
# from omni_msgs.msg import OmniFeedback
# import csv
# # from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# # from geometry_msgs.msg import Vector3 


# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = []
        
#         # Storage for bundled message data
#         self.latest_force_data = Vector3()  # ← CORRECT type
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0  # Store latest force from force code
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0

#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # NEW: Single bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers
#         # Subscribe to bundled data from master
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # NEW: Subscribe to force data from your force code (internal communication)
#         rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
#         rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

#         # Subscribe to force data from your force code (internal communication)


#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []
        
#         rospy.loginfo("Slave controller initialized with bundled messaging")

#     # def internal_force_callback(self, msg: OmniFeedback):
#     #     """Receive force data from your separate force code"""
#     #     self.latest_force_data = msg

#     def internal_force_callback(self, msg: Vector3):
#         """Receive force data from your separate force code"""
#          # Store as Vector3 (not OmniFeedback!)
#         self.latest_force_data = msg
    
#     def internal_force_timestamp_callback(self, msg: Float64):
#         """Receive force timestamp from your separate force code"""
#         self.latest_force_timestamp = msg.data

#     def master_data_callback(self, msg: MasterToSlaveData):
#         """
#         Receives bundled data from master:
#         - Pose data (position + orientation)
#         - Pose timestamp
#         - Force timestamp
#         """
#         # Extract pose timestamp
#         self.latest_pose_timestamp = msg.header.stamp.to_sec()
        
#         # Extract pose data
#         position = msg.pose.position
#         orientation = msg.pose.orientation
        
#         # Extract force timestamp (if master sent one)
#         master_force_timestamp = msg.force_timestamp
        
#         # Process the pose (existing logic)
#         self.process_haptic_pose(msg)
        
#         # Send bundled response back to master
#         self.send_bundled_to_master()
        
#         # Log
#         self.haptic_timestamps.append(self.latest_pose_timestamp)
#         self.last_packet_received_time = rospy.get_time()

#     def send_bundled_to_master(self):
#         """
#         Send bundled data back to master:
#         - Force feedback
#         - Pose timestamp (echo)
#         - Force timestamp
#         """
#         bundled_msg = SlaveToMasterData()
        
#         # 1. Add force data
#         bundled_msg.force = self.latest_force_data
        
#         # 2. Echo pose timestamp back
#         bundled_msg.pose_timestamp = self.latest_pose_timestamp
        
#         # 3. Add force timestamp
#         bundled_msg.force_timestamp = self.latest_force_timestamp
        
#         # 4. Add header with current time
#         bundled_msg.header.stamp = rospy.Time.now()
        
#         # Publish
#         self.slave_to_master_pub.publish(bundled_msg)
        
#         rospy.loginfo_throttle(1.0, f"Sent bundled to master: pose_ts={self.latest_pose_timestamp:.3f}, force_ts={self.latest_force_timestamp:.3f}")

#     def process_haptic_pose(self, msg: MasterToSlaveData):
#         """Process pose data from bundled message"""
#         # Convert quaternion to RPY
#         euler_from_haptic = self.haptic_quat2rpy([
#             msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#             [msg.pose.position.x],
#             [msg.pose.position.y],
#             [msg.pose.position.z],
#             [euler_from_haptic[0]],
#             [euler_from_haptic[1]],
#             [euler_from_haptic[2]]
#         ])

#         # Apply initial offset if first message
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         # ... (keep your existing implementation)
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):
#         # ... (keep your existing implementation)
#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     # ... (keep all your other methods: pose_and_rotation, joint_callback_robot, haptic_quat2rpy, etc.)

#     def pose_and_rotation(self, joint_position_robot):
#         # ... (keep existing implementation)
#         pass  # Replace with your actual code

#     def joint_callback_robot(self, msg: JointState):
#         # ... (keep existing implementation)
#         pass  # Replace with your actual code

#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         # ... (keep existing implementation)
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy = R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     # ... (keep all your plotting and CSV methods)

#     def main_loop(self):
#         rate = 500
#         rospy.Timer(rospy.Duration(1.0/rate), self.velocity_callback)
#         rospy.spin()


# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         try:
#             controller.make_csv()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")

#         try:
#             controller.make_csv_packet_times()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")

#Message Debug
#!/usr/bin/env python3

# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Vector3
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# from omni_msgs.msg import OmniFeedback
# import csv

# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
        
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = []
        
#         # Storage for bundled data
#         self.latest_force_data = Vector3()
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0

#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # Bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # Subscribe to internal force data (FORCE DATA SENDS ALL DATA HERE)
#         rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
#         rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []
        
#         rospy.loginfo("Slave pose controller initialized with bundled messaging")

#     def internal_force_callback(self, msg: Vector3):
#         """Receive force data from force code"""
#         self.latest_force_data = msg
    
#     def internal_force_timestamp_callback(self, msg: Float64):
#         """Receive force timestamp from force code"""
#         self.latest_force_timestamp = msg.data

#     def master_data_callback(self, msg: MasterToSlaveData):
#         """
#         Receives bundled data from master.
#         """
#         # Extract and store timestamp
#         timestamp = msg.header.stamp.to_sec()
#         self.latest_pose_timestamp = timestamp
#         self.haptic_timestamps.append(timestamp)
#         self.last_packet_received_time = rospy.get_time()
        
#         # Process haptic data (your original working logic)
#         euler_from_haptic = self.haptic_quat2rpy([
#             msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#             [msg.pose.position.x],
#             [msg.pose.position.y],
#             [msg.pose.position.z],
#             [euler_from_haptic[0]],
#             [euler_from_haptic[1]],
#             [euler_from_haptic[2]]
#         ])

#         # Apply initial offset if first message
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False
#             rospy.loginfo("Haptic device initialized!")

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position
        
#         # Send bundled response back to master IMMEDIATELY
#         self.send_bundled_to_master()

#     def send_bundled_to_master(self):
#         """Send bundled data back to master"""
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.pose_timestamp = self.latest_pose_timestamp
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()
        
#         self.slave_to_master_pub.publish(bundled_msg)

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):
#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8  # Original working gain
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     def pose_and_rotation(self,joint_position_robot):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles
#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])


#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         if self.robot_pose_flag:
#             self.initial_ee_pose = self.current_ee_pose
#             self.robot_pose_flag = False
#             rospy.loginfo("Robot initialized!")

#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy= R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     def make_csv(self):
#         times = np.array(self.haptic_timestamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
#                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
#                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

#             for i in range(len(times)):
#                 csvwriter.writerow([
#                     times[i],                     
#                     self.time_stamps[i],                  
#                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
#                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
#                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
#                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
#                 ])

#     def make_csv_packet_times(self):
#         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#             for received, applied, diff in self.packet_times:
#                 csvwriter.writerow([received, applied, diff])

#     def main_loop(self):
#         rate = 500
#         rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
#         rospy.spin()


# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         try:
#             controller.make_csv()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")
#         try:
#             controller.make_csv_packet_times()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")


#3
#purpose : To make some internal chnages for for force rtd
#This code is working fine

# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Vector3
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# from omni_msgs.msg import OmniFeedback
# import csv

# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
        
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = []
        
#         # Storage for bundled data
#         self.latest_force_data = Vector3()
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0

#         # NEW: Publisher to forward force echo to force controller
        
#         self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)
#         # Existing subscriber to receive from master
#         #rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)

#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # Bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # Subscribe to internal force data (FORCE DATA SENDS ALL DATA HERE)
#         rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
#         rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []
        
#         rospy.loginfo("Slave pose controller initialized with bundled messaging")

#     def internal_force_callback(self, msg: Vector3):
#         """Receive force data from force code"""
#         self.latest_force_data = msg
    
#     def internal_force_timestamp_callback(self, msg: Float64):
#         """Receive force timestamp from force code"""
#         self.latest_force_timestamp = msg.data

#     def master_data_callback(self, msg: MasterToSlaveData):
#         """
#         Receives bundled data from master.
#         """
#         # Extract and store timestamp
#         timestamp = msg.header.stamp.to_sec()
#         self.latest_pose_timestamp = timestamp
#         self.haptic_timestamps.append(timestamp)
#         self.last_packet_received_time = rospy.get_time()
        
#         # Process haptic data (your original working logic)
#         euler_from_haptic = self.haptic_quat2rpy([
#             msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#             [msg.pose.position.x],
#             [msg.pose.position.y],
#             [msg.pose.position.z],
#             [euler_from_haptic[0]],
#             [euler_from_haptic[1]],
#             [euler_from_haptic[2]]
#         ])

#         # Apply initial offset if first message
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False
#             rospy.loginfo("Haptic device initialized!")

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#         if msg.force_timestamp > 0:
#             echo_msg = Float64()
#             echo_msg.data = msg.force_timestamp
#             self.force_echo_pub.publish(echo_msg)            
#             rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
#         # Send bundled response back to master IMMEDIATELY
#         self.send_bundled_to_master()

#     def send_bundled_to_master(self):
#         """Send bundled data back to master"""
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.pose_timestamp = self.latest_pose_timestamp
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()
        
#         self.slave_to_master_pub.publish(bundled_msg)

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):
#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8  # Original working gain
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     def pose_and_rotation(self,joint_position_robot):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles
#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])


#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         if self.robot_pose_flag:
#             self.initial_ee_pose = self.current_ee_pose
#             self.robot_pose_flag = False
#             rospy.loginfo("Robot initialized!")

#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy= R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     def make_csv(self):
#         times = np.array(self.haptic_timestamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
#                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
#                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

#             for i in range(len(times)):
#                 csvwriter.writerow([
#                     times[i],                     
#                     self.time_stamps[i],                  
#                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
#                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
#                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
#                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
#                 ])

#     def make_csv_packet_times(self):
#         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#             for received, applied, diff in self.packet_times:
#                 csvwriter.writerow([received, applied, diff])

#     def main_loop(self):
#         rate = 500
#         rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
#         rospy.spin()


# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         try:
#             controller.make_csv()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")
#         try:
#             controller.make_csv_packet_times()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")


#6
#Everything thing is running here at 500 hz
#works perfectly

# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Vector3
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# from omni_msgs.msg import OmniFeedback
# import csv

# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
        
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = []
        
#         # Storage for bundled data
#         self.latest_force_data = Vector3()
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0

#         # NEW: Publisher to forward force echo to force controller
        
#         self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)
#         # Existing subscriber to receive from master
#         #rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)

#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # Bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # Subscribe to internal force data (FORCE DATA SENDS ALL DATA HERE)
#         rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
#         rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []
        
#         rospy.loginfo("Slave pose controller initialized with bundled messaging")

#     def internal_force_callback(self, msg: Vector3):
#         """Receive force data from force code"""
#         self.latest_force_data = msg
    
#     def internal_force_timestamp_callback(self, msg: Float64):
#         """Receive force timestamp from force code"""
#         self.latest_force_timestamp = msg.data

#     def master_data_callback(self, msg: MasterToSlaveData):
#         """
#         Receives bundled data from master.
#         """
#         # Extract and store timestamp
#         timestamp = msg.header.stamp.to_sec()
#         self.latest_pose_timestamp = timestamp
#         self.haptic_timestamps.append(timestamp)
#         self.last_packet_received_time = rospy.get_time()
        
#         # Process haptic data (your original working logic)
#         euler_from_haptic = self.haptic_quat2rpy([
#             msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#             [msg.pose.position.x],
#             [msg.pose.position.y],
#             [msg.pose.position.z],
#             [euler_from_haptic[0]],
#             [euler_from_haptic[1]],
#             [euler_from_haptic[2]]
#         ])

#         # Apply initial offset if first message
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False
#             rospy.loginfo("Haptic device initialized!")

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#         if msg.force_timestamp > 0:
#             echo_msg = Float64()
#             echo_msg.data = msg.force_timestamp
#             self.force_echo_pub.publish(echo_msg)            
#             rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
#         # Send bundled response back to master IMMEDIATELY
#     def bundled_publish_callback(self, event):
#     # """
#     # Fixed 500 Hz bundled transmission to master
#     # """
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.pose_timestamp = self.latest_pose_timestamp
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()

#         self.slave_to_master_pub.publish(bundled_msg)


#     def send_bundled_to_master(self):
#         """Send bundled data back to master"""
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.pose_timestamp = self.latest_pose_timestamp
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()
        
#         self.slave_to_master_pub.publish(bundled_msg)

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):
#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8  # Original working gain
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     def pose_and_rotation(self,joint_position_robot):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles
#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])


#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         if self.robot_pose_flag:
#             self.initial_ee_pose = self.current_ee_pose
#             self.robot_pose_flag = False
#             rospy.loginfo("Robot initialized!")

#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy= R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     def make_csv(self):
#         times = np.array(self.haptic_timestamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
#                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
#                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

#             for i in range(len(times)):
#                 csvwriter.writerow([
#                     times[i],                     
#                     self.time_stamps[i],                  
#                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
#                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
#                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
#                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
#                 ])

#     def make_csv_packet_times(self):
#         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#             for received, applied, diff in self.packet_times:
#                 csvwriter.writerow([received, applied, diff])

#     # def main_loop(self):
#     #     rate = 500
#     #     rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
#     #     rospy.spin()

#     def main_loop(self):
#         rate = 500  # Hz

#     # Velocity control loop
#         rospy.Timer(rospy.Duration(1.0 / rate), self.velocity_callback)

#     # FIXED 500Hz bundled transmission loop
#         rospy.Timer(rospy.Duration(1.0 / rate), self.bundled_publish_callback)

#         rospy.spin()



# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         try:
#             controller.make_csv()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")
#         try:
#             controller.make_csv_packet_times()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")

#7
#control loop=500Hz
#transmission loop=1000hz
#Sum bug in this code:rectified in 8th

# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Vector3
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# from omni_msgs.msg import OmniFeedback
# import csv

# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
        
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = []
        
#         # Storage for bundled data
#         self.latest_force_data = Vector3()
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0

#         # NEW: Publisher to forward force echo to force controller
        
#         self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)
#         # Existing subscriber to receive from master
#         #rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)

#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # Bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # Subscribe to internal force data (FORCE DATA SENDS ALL DATA HERE)
#         rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
#         rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []
        
#         rospy.loginfo("Slave pose controller initialized with bundled messaging")

#     def internal_force_callback(self, msg: Vector3):
#         """Receive force data from force code"""
#         self.latest_force_data = msg
    
#     def internal_force_timestamp_callback(self, msg: Float64):
#         """Receive force timestamp from force code"""
#         self.latest_force_timestamp = msg.data

#     def master_data_callback(self, msg: MasterToSlaveData):
#         """
#         Receives bundled data from master.
#         """
#         # Extract and store timestamp
#         timestamp = msg.header.stamp.to_sec()
#         self.latest_pose_timestamp = timestamp
#         self.haptic_timestamps.append(timestamp)
#         self.last_packet_received_time = rospy.get_time()
        
#         # Process haptic data (your original working logic)
#         euler_from_haptic = self.haptic_quat2rpy([
#             msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#             [msg.pose.position.x],
#             [msg.pose.position.y],
#             [msg.pose.position.z],
#             [euler_from_haptic[0]],
#             [euler_from_haptic[1]],
#             [euler_from_haptic[2]]
#         ])

#         # Apply initial offset if first message
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False
#             rospy.loginfo("Haptic device initialized!")

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#         if msg.force_timestamp > 0:
#             echo_msg = Float64()
#             echo_msg.data = msg.force_timestamp
#             self.force_echo_pub.publish(echo_msg)            
#             rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
#         # Send bundled response back to master IMMEDIATELY
#     def bundled_publish_callback(self, event):
#     # """
#     # Fixed 500 Hz bundled transmission to master
#     # """
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.pose_timestamp = self.latest_pose_timestamp
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()

#         self.slave_to_master_pub.publish(bundled_msg)


#     def send_bundled_to_master(self):
#         """Send bundled data back to master"""
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.pose_timestamp = self.latest_pose_timestamp
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()
        
#         self.slave_to_master_pub.publish(bundled_msg)

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):
#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8  # Original working gain
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     def pose_and_rotation(self,joint_position_robot):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles
#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])


#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         if self.robot_pose_flag:
#             self.initial_ee_pose = self.current_ee_pose
#             self.robot_pose_flag = False
#             rospy.loginfo("Robot initialized!")

#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy= R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     def make_csv(self):
#         times = np.array(self.haptic_timestamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
#                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
#                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

#             for i in range(len(times)):
#                 csvwriter.writerow([
#                     times[i],                     
#                     self.time_stamps[i],                  
#                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
#                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
#                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
#                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
#                 ])

#     def make_csv_packet_times(self):
#         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#             for received, applied, diff in self.packet_times:
#                 csvwriter.writerow([received, applied, diff])

#     # def main_loop(self):
#     #     rate = 500
#     #     rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
#     #     rospy.spin()


#     # def main_loop(self):
#     #     rate = 500  # Hz

#     # # Velocity control loop
#     #     rospy.Timer(rospy.Duration(1.0 / rate), self.velocity_callback)

#     # # FIXED 500Hz bundled transmission loop
#     #     rospy.Timer(rospy.Duration(1.0 / rate), self.bundled_publish_callback)

#     #     rospy.spin()

#     def main_loop(self):

#         velocity_rate = 500     # Hz (control loop)
#         bundle_rate   = 1000    # Hz (network transmission)

#         # Velocity control loop (500 Hz)
#         rospy.Timer(
#             rospy.Duration(1.0 / velocity_rate),
#             self.velocity_callback
#         )

#     # Bundled slave → master transmission (1000 Hz)
#         rospy.Timer(
#             rospy.Duration(1.0 / bundle_rate),
#             self.bundled_publish_callback
#         )

#         rospy.spin()




# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         try:
#             controller.make_csv()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")
#         try:
#             controller.make_csv_packet_times()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")


#8
#Resolved code

# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Vector3
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# from omni_msgs.msg import OmniFeedback
# import csv

# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
        
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = []
#         self.new_pose_available = False
#         self.pose_timestamp_to_send = 0.0

        
#         # Storage for bundled data
#         self.latest_force_data = Vector3()
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0

#         # NEW: Publisher to forward force echo to force controller
        
#         self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)
#         # Existing subscriber to receive from master
#         #rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)

#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # Bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # Subscribe to internal force data (FORCE DATA SENDS ALL DATA HERE)
#         rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
#         rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []
        
#         rospy.loginfo("Slave pose controller initialized with bundled messaging")

#     def internal_force_callback(self, msg: Vector3):
#         """Receive force data from force code"""
#         self.latest_force_data = msg
    
#     def internal_force_timestamp_callback(self, msg: Float64):
#         """Receive force timestamp from force code"""
#         self.latest_force_timestamp = msg.data

#     def master_data_callback(self, msg: MasterToSlaveData):
#         """
#         Receives bundled data from master.
#         """
#         # Extract and store timestamp
#         timestamp = msg.header.stamp.to_sec()
#         self.latest_pose_timestamp = timestamp
#         self.new_pose_available = True
#         self.haptic_timestamps.append(timestamp)
#         self.last_packet_received_time = rospy.get_time()
        
#         # Process haptic data (your original working logic)
#         euler_from_haptic = self.haptic_quat2rpy([
#             msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#             [msg.pose.position.x],
#             [msg.pose.position.y],
#             [msg.pose.position.z],
#             [euler_from_haptic[0]],
#             [euler_from_haptic[1]],
#             [euler_from_haptic[2]]
#         ])

#         # Apply initial offset if first message
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False
#             rospy.loginfo("Haptic device initialized!")

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#         if msg.force_timestamp > 0:
#             echo_msg = Float64()
#             echo_msg.data = msg.force_timestamp
#             self.force_echo_pub.publish(echo_msg)            
#             rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
#         # Send bundled response back to master IMMEDIATELY
#     def bundled_publish_callback(self, event):
#     # """
#     # Fixed 500 Hz bundled transmission to master
#     # """
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.pose_timestamp = self.latest_pose_timestamp
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()

#         self.slave_to_master_pub.publish(bundled_msg)


#     def send_bundled_to_master(self):
#         # """Send bundled data back to master"""
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()
#         #bundled_msg.force_timestamp = self.latest_force_timestamp
#         #bundled_msg.header.stamp = rospy.Time.now()
#         if self.pose_timestamp_to_send > 0:
#             bundled_msg.pose_timestamp = self.pose_timestamp_to_send
#             self.pose_timestamp_to_send = 0.0  # prevent duplicates
#         else:
#             bundled_msg.pose_timestamp = 0.0   # master ignores
        
#         self.slave_to_master_pub.publish(bundled_msg)

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):
#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8  # Original working gain
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     def pose_and_rotation(self,joint_position_robot):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles
#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])


#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         if self.robot_pose_flag:
#             self.initial_ee_pose = self.current_ee_pose
#             self.robot_pose_flag = False
#             rospy.loginfo("Robot initialized!")

#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy= R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         if not self.new_pose_available:
#             return
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         self.velocity_pub.publish(velocity_pub_msg)
#         self.new_pose_available = False
#         self.pose_timestamp_to_send = self.latest_pose_timestamp

#     # Log processing delay (optional, for analysis)
#         applied_time = rospy.get_time()
    
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     def make_csv(self):
#         times = np.array(self.haptic_timestamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
#                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
#                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

#             for i in range(len(times)):
#                 csvwriter.writerow([
#                     times[i],                     
#                     self.time_stamps[i],                  
#                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
#                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
#                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
#                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
#                 ])

#     def make_csv_packet_times(self):
#         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#             for received, applied, diff in self.packet_times:
#                 csvwriter.writerow([received, applied, diff])

#     # def main_loop(self):
#     #     rate = 500
#     #     rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
#     #     rospy.spin()


#     # def main_loop(self):
#     #     rate = 500  # Hz

#     # # Velocity control loop
#     #     rospy.Timer(rospy.Duration(1.0 / rate), self.velocity_callback)

#     # # FIXED 500Hz bundled transmission loop
#     #     rospy.Timer(rospy.Duration(1.0 / rate), self.bundled_publish_callback)

#     #     rospy.spin()

#     def main_loop(self):

#         velocity_rate = 500     # Hz (control loop)
#         bundle_rate   = 1000    # Hz (network transmission)

#         # Velocity control loop (500 Hz)
#         rospy.Timer(
#             rospy.Duration(1.0 / velocity_rate),
#             self.velocity_callback
#         )

#     # Bundled slave → master transmission (1000 Hz)
#         rospy.Timer(
#             rospy.Duration(1.0 / bundle_rate),
#             self.bundled_publish_callback
#         )

#         rospy.spin()




# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         try:
#             controller.make_csv()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")
#         try:
#             controller.make_csv_packet_times()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")

#9
#Potentially Correct
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
from omni_msgs.msg import OmniFeedback
import csv

class RobotEndEffectorController:

    def __init__(self):
        # Initializing node
        rospy.init_node('robot_end_effector_controller', anonymous=True)
        
        self.forwarded_time_stamps = []
        self.last_received_master_timestamp = None
        self.last_received_master_force_timestamp = None
        self.start_time = rospy.get_time()
        self.packet_times = []
        self.new_pose_available = False
        self.pose_timestamp_to_send = 0.0

        
        # Storage for bundled data
        self.latest_force_data = Vector3()
        self.latest_force_data.x = 0.0
        self.latest_force_data.y = 0.0
        self.latest_force_data.z = 0.0
        self.latest_pose_timestamp = 0.0
        self.latest_force_timestamp = 0.0

        # NEW: Publisher to forward force echo to force controller
        
        self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)
        # Existing subscriber to receive from master
        #rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)

        # Publishers
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
        # Bundled publisher to master
        self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
        self.haptic_timestamps = []

        # Subscribers
        rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
        # Subscribe to internal force data (FORCE DATA SENDS ALL DATA HERE)
        rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
        rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

        # For initialization parameter
        self.robot_pose_flag = True
        self.haptic_pose_flag = True 
        self.initial_ee_pose = np.zeros((6, 1))
        self.current_ee_pose = np.zeros((6, 1))
        self.joint_position_robot = np.zeros(6)
        self.haptic_stylus_position = np.zeros((6, 1))
        self.haptic_stylus_initial_position = np.zeros((6,1))

        # For unwrapping
        self.robot_previous_angles = None
        self.haptic_previous_angles = None

        # For plotting
        self.start_time = None
        self.time_stamps = []
        self.ee_pose_plot = []
        self.stylus_pose_plot = []
        
        rospy.loginfo("Slave pose controller initialized with bundled messaging")

    def internal_force_callback(self, msg: Vector3):
        """Receive force data from force code"""
        self.latest_force_data = msg
    
    def internal_force_timestamp_callback(self, msg: Float64):
        """Receive force timestamp from force code"""
        self.latest_force_timestamp = msg.data

    def master_data_callback(self, msg: MasterToSlaveData):
        """
        Receives bundled data from master.
        """
        # Extract and store timestamp
        timestamp = msg.header.stamp.to_sec()
        self.latest_pose_timestamp = timestamp
        self.new_pose_available = True
        self.haptic_timestamps.append(timestamp)
        self.last_packet_received_time = rospy.get_time()
        
        # Process haptic data (your original working logic)
        euler_from_haptic = self.haptic_quat2rpy([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        if self.haptic_previous_angles is not None:
            euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
        self.haptic_previous_angles = euler_from_haptic

        haptic_stylus_position = np.array([
            [msg.pose.position.x],
            [msg.pose.position.y],
            [msg.pose.position.z],
            [euler_from_haptic[0]],
            [euler_from_haptic[1]],
            [euler_from_haptic[2]]
        ])

        # Apply initial offset if first message
        if self.haptic_pose_flag:
            self.haptic_stylus_initial_position = haptic_stylus_position
            self.haptic_pose_flag = False
            rospy.loginfo("Haptic device initialized!")

        self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

        if msg.force_timestamp > 0:
            echo_msg = Float64()
            echo_msg.data = msg.force_timestamp
            self.force_echo_pub.publish(echo_msg)            
            rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
        # Send bundled response back to master IMMEDIATELY
    


    def send_bundled_to_master(self):
        # """Send bundled data back to master"""
        bundled_msg = SlaveToMasterData()
        bundled_msg.force = self.latest_force_data
        bundled_msg.force_timestamp = self.latest_force_timestamp
        bundled_msg.header.stamp = rospy.Time.now()
        #bundled_msg.force_timestamp = self.latest_force_timestamp
        #bundled_msg.header.stamp = rospy.Time.now()
        if self.pose_timestamp_to_send > 0:
            bundled_msg.pose_timestamp = self.pose_timestamp_to_send
            self.pose_timestamp_to_send = 0.0  # prevent duplicates
        else:
            bundled_msg.pose_timestamp = 0.0   # master ignores
        
        self.slave_to_master_pub.publish(bundled_msg)

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

        c = 8  # Original working gain
        k = np.array([[c, 0, 0, 0, 0, 0],
                      [0, c, 0, 0, 0, 0],
                      [0, 0, c, 0, 0, 0],
                      [0, 0, 0, c, 0, 0],
                      [0, 0, 0, 0, c, 0],
                      [0, 0, 0, 0, 0, c]])
 
        ee_pose = self.current_ee_pose - self.initial_ee_pose
        joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
        # Log data
        current_time = rospy.get_time()
        if self.start_time is None:
            self.start_time = current_time
        self.time_stamps.append(current_time - self.start_time)
        self.ee_pose_plot.append(ee_pose.flatten())
        self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
        return joint_space_velocity.flatten()

    def pose_and_rotation(self,joint_position_robot):
        q1, q2, q3, q4, q5, q6 = joint_position_robot
        cos = np.cos
        sin = np.sin
        a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
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
            self.initial_ee_pose = self.current_ee_pose
            self.robot_pose_flag = False
            rospy.loginfo("Robot initialized!")

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

    def velocity_callback(self, event):
        if not self.new_pose_available:
            return
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.rpy2joint_space_vel()
        self.velocity_pub.publish(velocity_pub_msg)
        self.new_pose_available = False
        self.pose_timestamp_to_send = self.latest_pose_timestamp

    # Log processing delay (optional, for analysis)
        applied_time = rospy.get_time()
    
        if hasattr(self, "last_packet_received_time"):
            diff = applied_time - self.last_packet_received_time
            self.packet_times.append((
                self.last_packet_received_time, 
                applied_time, 
                diff))
        #self.velocity_pub.publish(velocity_pub_msg)

    def make_csv(self):
        times = np.array(self.haptic_timestamps)
        ee_pose_plot = np.array(self.ee_pose_plot)
        stylus_pose_plot = np.array(self.stylus_pose_plot)

        with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
                            'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
                            'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

            for i in range(len(times)):
                csvwriter.writerow([
                    times[i],                     
                    self.time_stamps[i],                  
                    ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
                    ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
                    stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
                    stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
                ])

    def make_csv_packet_times(self):
        with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

            for received, applied, diff in self.packet_times:
                csvwriter.writerow([received, applied, diff])

    # def main_loop(self):
    #     rate = 500
    #     rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
    #     rospy.spin()


    # def main_loop(self):
    #     rate = 500  # Hz

    # # Velocity control loop
    #     rospy.Timer(rospy.Duration(1.0 / rate), self.velocity_callback)

    # # FIXED 500Hz bundled transmission loop
    #     rospy.Timer(rospy.Duration(1.0 / rate), self.bundled_publish_callback)

    #     rospy.spin()

    def main_loop(self):

        velocity_rate = 500     # Hz (control loop)
        bundle_rate   = 500    # Hz (network transmission)

        # Velocity control loop (500 Hz)
        rospy.Timer(
            rospy.Duration(1.0 / velocity_rate),
            self.velocity_callback
        )

    # Bundled slave → master transmission (1000 Hz)
        rospy.Timer(
            rospy.Duration(1.0 / bundle_rate),
            lambda event: self.send_bundled_to_master()
        )

        rospy.spin()




if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        try:
            controller.make_csv()
        except Exception as e:
            rospy.logerr(f"Failed to save pose.csv: {e}")
        try:
            controller.make_csv_packet_times()
        except Exception as e:
            rospy.logerr(f"Failed to save pose_processing.csv: {e}")



#5
#ALL Bundled samples at 500Hz
#!/usr/bin/env python


# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Vector3
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# from omni_msgs.msg import OmniFeedback
# import csv
# import os


# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
        
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = []
        
#         # Storage for bundled data
#         self.latest_force_data = Vector3()
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0
        
#         # CRITICAL FIX: Track which pose timestamps have been sent
#         self.last_sent_pose_timestamp = 0.0  # Last pose_timestamp that was sent in a bundle
#         self.pose_timestamp_ready_to_send = False  # Flag: new pose available to send

#         # Track pose timestamp waiting time
#         self.pose_timestamp_wait_log = []  # (pose_ts, receive_time, send_time, wait_time)
#         self.pose_timestamp_receive_time = {}  # Maps pose_ts -> receive time

#         # Bundled transmission statistics
#         self.bundled_send_count = 0
#         self.bundled_send_times = []

#         # Publisher to forward force echo to force controller
#         self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)

#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # Bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # Subscribe to internal force data
#         rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
#         rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []
        
#         # Register shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
        
#         rospy.loginfo("Slave pose controller initialized with FIXED 500Hz bundled transmission")
#         rospy.loginfo("FIXED: Prevents duplicate pose_timestamp echoes")

#     def internal_force_callback(self, msg: Vector3):
#         """Receive force data from force code"""
#         self.latest_force_data = msg
    
#     def internal_force_timestamp_callback(self, msg: Float64):
#         """Receive force timestamp from force code"""
#         self.latest_force_timestamp = msg.data

#     def master_data_callback(self, msg: MasterToSlaveData):
#         """
#         Receives bundled data from master.
#         STORES data but does NOT send (sending happens at fixed 500Hz)
#         """
#         # Record receive time
#         receive_time = rospy.get_time()
        
#         # Extract and store timestamp
#         timestamp = msg.header.stamp.to_sec()
        
#         # CRITICAL: Only process NEW pose timestamps
#         if timestamp != self.latest_pose_timestamp:
#             # New pose received
#             self.latest_pose_timestamp = timestamp
#             self.pose_timestamp_ready_to_send = True  # Mark as ready to send
            
#             # Store receive time for this NEW pose timestamp
#             if timestamp not in self.pose_timestamp_receive_time:
#                 self.pose_timestamp_receive_time[timestamp] = receive_time
            
#             self.haptic_timestamps.append(timestamp)
            
#             rospy.loginfo_throttle(1.0, f"[SLAVE] NEW pose_ts received: {timestamp:.6f}")
        
#         self.last_packet_received_time = rospy.get_time()
        
#         # Process haptic data (ORIGINAL LOGIC - UNCHANGED)
#         euler_from_haptic = self.haptic_quat2rpy([
#             msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#             [msg.pose.position.x],
#             [msg.pose.position.y],
#             [msg.pose.position.z],
#             [euler_from_haptic[0]],
#             [euler_from_haptic[1]],
#             [euler_from_haptic[2]]
#         ])

#         # Apply initial offset if first message
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False
#             rospy.loginfo("Haptic device initialized!")

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#         # Forward force timestamp echo to force controller
#         if msg.force_timestamp > 0:
#             echo_msg = Float64()
#             echo_msg.data = msg.force_timestamp
#             self.force_echo_pub.publish(echo_msg)            
#             rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")

#     def bundled_transmission_callback(self, event):
#         """
#         Send bundled data at FIXED 500Hz (called by Timer)
#         CRITICAL: Only sends each pose_timestamp ONCE
#         """
#         send_time = rospy.get_time()
        
#         # Create bundled message
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()
        
#         # CRITICAL FIX: Only include pose_timestamp if it's NEW and hasn't been sent yet
#         if self.pose_timestamp_ready_to_send and self.latest_pose_timestamp != self.last_sent_pose_timestamp:
#             # This is a NEW pose timestamp that hasn't been echoed yet
#             bundled_msg.pose_timestamp = self.latest_pose_timestamp
            
#             # Log waiting time
#             if self.latest_pose_timestamp in self.pose_timestamp_receive_time:
#                 receive_time = self.pose_timestamp_receive_time[self.latest_pose_timestamp]
#                 wait_time = send_time - receive_time
                
#                 self.pose_timestamp_wait_log.append((
#                     self.latest_pose_timestamp,
#                     receive_time,
#                     send_time,
#                     wait_time
#                 ))
                
#                 rospy.loginfo_throttle(1.0, 
#                     f"[SLAVE@500Hz] Sending pose_ts {self.latest_pose_timestamp:.6f}, "
#                     f"wait: {wait_time*1000:.3f}ms")
                
#                 # Clean up
#                 del self.pose_timestamp_receive_time[self.latest_pose_timestamp]
            
#             # Mark this timestamp as sent
#             self.last_sent_pose_timestamp = self.latest_pose_timestamp
#             self.pose_timestamp_ready_to_send = False
#         else:
#             # No new pose timestamp available, send 0 (master will ignore)
#             bundled_msg.pose_timestamp = 0.0
#             rospy.loginfo_throttle(5.0, "[SLAVE@500Hz] No new pose_ts, sending bundle with pose_ts=0")
        
#         # Publish the bundle
#         self.slave_to_master_pub.publish(bundled_msg)
        
#         # Statistics
#         self.bundled_send_count += 1
#         self.bundled_send_times.append(send_time)

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):
#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     def pose_and_rotation(self, joint_position_robot):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles
#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], 
#                                               msg.position[3], msg.position[4], msg.position[5]])
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         if self.robot_pose_flag:
#             self.initial_ee_pose = self.current_ee_pose
#             self.robot_pose_flag = False
#             rospy.loginfo("Robot initialized!")

#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy = R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     def make_csv(self):
#         times = np.array(self.haptic_timestamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
#                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
#                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

#             for i in range(len(times)):
#                 csvwriter.writerow([
#                     times[i],                     
#                     self.time_stamps[i],                  
#                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
#                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
#                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
#                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
#                 ])

#     def make_csv_packet_times(self):
#         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#             for received, applied, diff in self.packet_times:
#                 csvwriter.writerow([received, applied, diff])

#     def save_pose_timestamp_wait_to_csv(self, filepath='/home/user/Desktop/delay/slave_pose_timestamp_wait.csv'):
#         """Save pose timestamp waiting time log"""
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow([
#                     'Pose Timestamp (from Master)',
#                     'Received at Slave (s)',
#                     'Sent in Bundle (s)',
#                     'Wait Time (s)',
#                     'Wait Time (ms)'
#                 ])
                
#                 for pose_ts, recv_time, send_time, wait_time in self.pose_timestamp_wait_log:
#                     writer.writerow([
#                         f"{pose_ts:.9f}",
#                         f"{recv_time:.9f}",
#                         f"{send_time:.9f}",
#                         f"{wait_time:.9f}",
#                         f"{wait_time*1000:.3f}"
#                     ])
            
#             rospy.loginfo(f"Pose timestamp wait log saved to: {filepath}")
            
#             # Print statistics
#             if self.pose_timestamp_wait_log:
#                 wait_times = [w[3] for w in self.pose_timestamp_wait_log]
#                 rospy.loginfo("="*70)
#                 rospy.loginfo("Pose Timestamp Wait Time Statistics (Slave @ 500Hz fixed):")
#                 rospy.loginfo(f"  Average:       {sum(wait_times)/len(wait_times)*1000:.3f} ms")
#                 rospy.loginfo(f"  Min:           {min(wait_times)*1000:.3f} ms")
#                 rospy.loginfo(f"  Max:           {max(wait_times)*1000:.3f} ms")
#                 rospy.loginfo(f"  Total samples: {len(wait_times)}")
#                 rospy.loginfo("="*70)
                
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose timestamp wait log to CSV: {e}")

#     def shutdown_hook(self):
#         """Handle shutdown and save all logs"""
#         rospy.loginfo("Shutting down slave pose controller...")
        
#         try:
#             self.make_csv()
#             rospy.loginfo("pose.csv saved successfully")
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")
        
#         try:
#             self.make_csv_packet_times()
#             rospy.loginfo("pose_processing.csv saved successfully")
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")
        
#         try:
#             self.save_pose_timestamp_wait_to_csv()
#             rospy.loginfo("slave_pose_timestamp_wait.csv saved successfully")
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose timestamp wait log: {e}")
        
#         rospy.loginfo("Shutdown complete.")

#     def main_loop(self):
#         """
#         Main loop with TWO 500Hz timers:
#         1. Velocity control
#         2. Bundled transmission (FIXED 500Hz)
#         """
#         rate = 500
        
#         # Timer 1: Velocity control
#         rospy.Timer(rospy.Duration(1.0/rate), self.velocity_callback)
        
#         # Timer 2: Bundled transmission at FIXED 500Hz
#         rospy.Timer(rospy.Duration(1.0/rate), self.bundled_transmission_callback)
        
#         rospy.loginfo(f"Started TWO 500Hz timers: velocity control + bundled transmission")
        
#         rospy.spin()


# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass



#4
#pose timestamp waiting timelogging has been added.

# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Vector3
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# from teleop_msgs.msg import MasterToSlaveData, SlaveToMasterData
# from omni_msgs.msg import OmniFeedback
# import csv
# import os


# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
        
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = []
        
#         # Storage for bundled data
#         self.latest_force_data = Vector3()
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0

#         # NEW: Pose timestamp waiting time tracking
#         self.pose_timestamp_wait_log = []  # Stores (pose_ts, receive_time, send_time, wait_time)

#         # Publisher to forward force echo to force controller
#         self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)
        
#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # Bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
#         # Subscribe to internal force data
#         rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
#         rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []
        
#         # Register shutdown hook
#         rospy.on_shutdown(self.shutdown_hook)
        
#         rospy.loginfo("Slave pose controller initialized with bundled messaging")
#         rospy.loginfo("Pose timestamp waiting time logging enabled")

#     def internal_force_callback(self, msg: Vector3):
#         """Receive force data from force code"""
#         self.latest_force_data = msg
    
#     def internal_force_timestamp_callback(self, msg: Float64):
#         """Receive force timestamp from force code"""
#         self.latest_force_timestamp = msg.data

#     def master_data_callback(self, msg: MasterToSlaveData):
#         """
#         Receives bundled data from master.
#         MODIFIED: Records pose timestamp receive time and send time for wait calculation
#         """
#         # NEW: Record receive time IMMEDIATELY (before any processing)
#         receive_time = rospy.get_time()
        
#         # Extract and store timestamp
#         timestamp = msg.header.stamp.to_sec()
#         self.latest_pose_timestamp = timestamp
#         self.haptic_timestamps.append(timestamp)
#         self.last_packet_received_time = rospy.get_time()
        
#         # Process haptic data (ORIGINAL LOGIC - UNCHANGED)
#         euler_from_haptic = self.haptic_quat2rpy([
#             msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#             [msg.pose.position.x],
#             [msg.pose.position.y],
#             [msg.pose.position.z],
#             [euler_from_haptic[0]],
#             [euler_from_haptic[1]],
#             [euler_from_haptic[2]]
#         ])

#         # Apply initial offset if first message
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False
#             rospy.loginfo("Haptic device initialized!")

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#         # Forward force timestamp echo to force controller
#         if msg.force_timestamp > 0:
#             echo_msg = Float64()
#             echo_msg.data = msg.force_timestamp
#             self.force_echo_pub.publish(echo_msg)            
#             rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
#         # MODIFIED: Send bundled response and get send time
#         send_time = self.send_bundled_to_master()
        
#         # NEW: Calculate and log pose timestamp waiting time
#         wait_time = send_time - receive_time
        
#         self.pose_timestamp_wait_log.append((
#             timestamp,       # pose_timestamp value (from master)
#             receive_time,    # when received at slave
#             send_time,       # when sent back in bundle
#             wait_time        # waiting time in seconds
#         ))
        
#         # Log periodically
#         rospy.loginfo_throttle(2.0, 
#             f"[SLAVE] Pose_ts {timestamp:.6f} wait: {wait_time*1000:.3f}ms "
#             f"(recv={receive_time:.6f}, send={send_time:.6f})")

#     def send_bundled_to_master(self):
#         """
#         Send bundled data back to master
#         MODIFIED: Returns send timestamp for wait time calculation
#         """
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.pose_timestamp = self.latest_pose_timestamp
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()
        
#         # NEW: Record send time BEFORE publishing
#         send_time = rospy.get_time()
        
#         self.slave_to_master_pub.publish(bundled_msg)
        
#         # NEW: Return send time for wait calculation
#         return send_time

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):
#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     def pose_and_rotation(self, joint_position_robot):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot
#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles
#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], 
#                                               msg.position[3], msg.position[4], msg.position[5]])
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         if self.robot_pose_flag:
#             self.initial_ee_pose = self.current_ee_pose
#             self.robot_pose_flag = False
#             rospy.loginfo("Robot initialized!")

#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy = R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     def make_csv(self):
#         times = np.array(self.haptic_timestamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
#                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
#                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

#             for i in range(len(times)):
#                 csvwriter.writerow([
#                     times[i],                     
#                     self.time_stamps[i],                  
#                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
#                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
#                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
#                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
#                 ])

#     def make_csv_packet_times(self):
#         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#             for received, applied, diff in self.packet_times:
#                 csvwriter.writerow([received, applied, diff])

#     def save_pose_timestamp_wait_to_csv(self, filepath='/home/user/Desktop/delay/pose_timestamp_wait.csv'):
#         """
#         NEW: Save pose timestamp waiting time log at slave side
#         This measures the time between receiving a pose timestamp and sending it back
#         """
#         try:
#             os.makedirs(os.path.dirname(filepath), exist_ok=True)
#             with open(filepath, 'w', newline='') as csvfile:
#                 writer = csv.writer(csvfile)
#                 writer.writerow([
#                     'Pose Timestamp (from Master)',
#                     'Received at Slave (s)',
#                     'Sent in Bundle (s)',
#                     'Wait Time (s)',
#                     'Wait Time (ms)'
#                 ])
                
#                 for pose_ts, recv_time, send_time, wait_time in self.pose_timestamp_wait_log:
#                     writer.writerow([
#                         f"{pose_ts:.9f}",
#                         f"{recv_time:.9f}",
#                         f"{send_time:.9f}",
#                         f"{wait_time:.9f}",
#                         f"{wait_time*1000:.3f}"
#                     ])
            
#             rospy.loginfo(f"Pose timestamp wait log saved to: {filepath}")
            
#             # Print statistics
#             if self.pose_timestamp_wait_log:
#                 wait_times = [w[3] for w in self.pose_timestamp_wait_log]
#                 rospy.loginfo("="*70)
#                 rospy.loginfo("Pose Timestamp Wait Time Statistics (at Slave):")
#                 rospy.loginfo(f"  Average:      {sum(wait_times)/len(wait_times)*1000:.3f} ms")
#                 rospy.loginfo(f"  Min:          {min(wait_times)*1000:.3f} ms")
#                 rospy.loginfo(f"  Max:          {max(wait_times)*1000:.3f} ms")
#                 rospy.loginfo(f"  Total samples: {len(wait_times)}")
#                 rospy.loginfo("="*70)
                
#         except Exception as e:
#             rospy.logerr(f"Failed to write pose timestamp wait log to CSV: {e}")

#     def shutdown_hook(self):
#         """Handle shutdown and save all logs"""
#         rospy.loginfo("Shutting down slave pose controller...")
        
#         # Save all CSV files
#         try:
#             self.make_csv()
#             rospy.loginfo("pose.csv saved successfully")
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")
        
#         try:
#             self.make_csv_packet_times()
#             rospy.loginfo("pose_processing.csv saved successfully")
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")
        
#         try:
#             self.save_pose_timestamp_wait_to_csv()
#             rospy.loginfo("slave_pose_timestamp_wait.csv saved successfully")
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose timestamp wait log: {e}")
        
#         rospy.loginfo("Shutdown complete.")

#     def main_loop(self):
#         rate = 500
#         rospy.Timer(rospy.Duration(1.0/rate), self.velocity_callback)
#         rospy.spin()


# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass



#Here we are making changes:now pose rtd is increasing(only changes seen are for 500 to 250Hz)
#Implement:time differnce=Time differnce when pose timestamps received from the master and when its get sent back again to the master side

# import rospy
# import numpy as np
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Float64MultiArray
# from scipy.spatial.transform import Rotation as R
# from sensor_msgs.msg import JointState
# import matplotlib.pyplot as plt
# from std_msgs.msg import Float64
# import csv

# class RobotEndEffectorController:

#     def __init__(self):
#         # Initializing node
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = [] 
#         self.last_received_master_timestamp = None


#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
#         self.time_back_to_master_pose_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=1)
#         self.time_back_to_master_force_pub = rospy.Publisher('time_from_slave_to_master_for_force', Float64, queue_size=1)
#         self.haptic_timestamps = []
#         #self.time_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=1, tcp_nodelay=True)
#         self.time_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=10)

        


#         # Subscribers
#         rospy.Subscriber('pose_from_master_to_slave', PoseStamped, self.haptic_callback)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
#         rospy.Subscriber('time_from_master_to_slave_for_pose', Float64, self.time_from_master_to_slave_for_pose_callback)
#         rospy.Subscriber('time_from_master_to_slave_for_force', Float64, self.time_from_master_to_slave_for_force_callback)



#         # For initialization parameter
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # For unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # For plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []

#     @staticmethod
#     def unwrap_angle(angle, previous_angle):
#         if previous_angle is None:
#             return angle
#         delta = angle - previous_angle
#         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
#         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
#         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
#         return previous_angle + delta
    
#     def time_from_master_to_slave_for_force_callback(self, msg):
#         self.last_received_master_force_timestamp = msg.data

    
#     # def time_from_master_to_slave_for_pose_callback(self, msg: Float64):
#     #     self.time_pub.publish(msg)
#     #     self.forwarded_time_stamps.append(msg.data)
    
#     def time_from_master_to_slave_for_pose_callback(self, msg):
#         #self.last_received_master_timestamp = msg.data
#         #self.time_back_to_master_pose_pub.publish(msg)
#         #self.time_pub.publish(msg.data)  # This sends the same timestamp back to the master:here msg has been changed to msg.data
#         #self.forwarded_time_stamps.append(msg.data)
#         self.last_received_master_timestamp = msg.data
#         self.forwarded_time_stamps.append(msg.data)

#     @staticmethod
#     def Jointspace2GeometricJacobian(joint_position_robot):
#         t1, t2, t3, t4, t5, t6 = joint_position_robot
#         cos, sin = np.cos, np.sin
#         J_geometrical = np.array([
#             [
#                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
#                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
#                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
#                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
#                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
#                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
#                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
#                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
#                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
#                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
#                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
#                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
#                 (1707*cos(t2 + t3)*cos(t4))/20000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
#                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
#                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
#                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
#                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
#                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
#                 0
#             ],
#             [
#                 0, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
#                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
#                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
#                 0
#             ],
#             [
#                 0, 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t1), 
#                 sin(t2 + t3 + t4)*cos(t1), 
#                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
#             ],
#             [
#                 0, 
#                 -cos(t1), 
#                 -cos(t1), 
#                 -cos(t1), 
#                 sin(t2 + t3 + t4)*sin(t1), 
#                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
#             ],
#             [
#                 1, 
#                 0, 
#                 0, 
#                 0, 
#                 -cos(t2 + t3 + t4), 
#                 -sin(t2 + t3 + t4)*sin(t5)
#             ]
#         ])
#         lamda = 0.001
#         return J_geometrical + lamda * np.eye(6)
    
#     def rpy2joint_space_vel(self):

#         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
#         cos, sin = np.cos, np.sin
        
#         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
#                               [0, 1, 0,                    0,           0, 0],
#                               [0, 0, 1,                    0,           0, 0],
#                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
#                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
#                               [0, 0, 0,           -sin(beta),           0, 1]])

#         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

#         c = 8  #Original c ws 8
#         k = np.array([[c, 0, 0, 0, 0, 0],
#                       [0, c, 0, 0, 0, 0],
#                       [0, 0, c, 0, 0, 0],
#                       [0, 0, 0, c, 0, 0],
#                       [0, 0, 0, 0, c, 0],
#                       [0, 0, 0, 0, 0, c]])
 
#         ee_pose = self.current_ee_pose - self.initial_ee_pose
#         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
#         # Log data
#         current_time = rospy.get_time()
#         if self.start_time is None:
#             self.start_time = current_time
#         self.time_stamps.append(current_time - self.start_time)
#         self.ee_pose_plot.append(ee_pose.flatten())
#         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
#         return joint_space_velocity.flatten()

#     def pose_and_rotation(self,joint_position_robot):
#         q1, q2, q3, q4, q5, q6 = joint_position_robot

#         cos = np.cos
#         sin = np.sin
#         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
#         # Calculate each component of the position
#         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
#             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
#             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
#             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

#         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
#             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
#             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
#             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

#         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
#             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

#         # Compute each element of the rotation matrix
#         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
#         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
#         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
#         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
#         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
#         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
#         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
#         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
#         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
#         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
#         rotation = R.from_matrix(R_matrix)
#         euler_angles = rotation.as_euler('xyz', degrees=False)
#         euler_angles = np.array(euler_angles)
#         if self.robot_previous_angles is not None:
#             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
#         self.robot_previous_angles = euler_angles

#         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])

#     def joint_callback_robot(self, msg: JointState):
#         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
#         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
#         if self.robot_pose_flag:
#             self.initial_ee_pose = self.current_ee_pose
#             self.robot_pose_flag = False

#     #def haptic_callback(self, msg: PoseStamped):
#     # Store header timestamp from incoming haptic pose
#     #     self.haptic_timestamps.append(msg.header.stamp.to_sec())

#     #     euler_from_haptic = self.haptic_quat2rpy([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
#     #     if self.haptic_previous_angles is not None:
#     #         euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#     #     self.haptic_previous_angles = euler_from_haptic

#     #     haptic_stylus_position = np.array([[msg.pose.position.x],[msg.pose.position.y],[msg.pose.position.z],
#     #                                    [euler_from_haptic[0]],[euler_from_haptic[1]],[euler_from_haptic[2]]])
    
#     #     if self.haptic_pose_flag:
#     #         self.haptic_stylus_initial_position = haptic_stylus_position
#     #         self.haptic_pose_flag = False

#     #     self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#     # # --- Send back the master timestamp for delay calculation ---
#     #     if self.last_received_master_timestamp is not None:
#     #         self.time_pub.publish(Float64(self.last_received_master_timestamp))

#     #     return self.haptic_stylus_position
 
# ##THIS IS THE ORIGINAL HAPTIC CALLBACK FUNCTION
 
#     def haptic_callback(self, msg: PoseStamped):
#     # --- Store and forward header timestamp from master ---
#         timestamp = msg.header.stamp.to_sec()
#         self.haptic_timestamps.append(timestamp)

# # New line
#         self.last_packet_received_time = rospy.get_time()
#     # Publish the same timestamp back to master for delay logging
#         #self.time_pub.publish(Float64(timestamp))

#     # --- Pose and orientation processing ---
#         euler_from_haptic = self.haptic_quat2rpy([
#         msg.pose.orientation.x,
#         msg.pose.orientation.y,
#         msg.pose.orientation.z,
#         msg.pose.orientation.w
#     ])

#         if self.haptic_previous_angles is not None:
#             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
#         self.haptic_previous_angles = euler_from_haptic

#         haptic_stylus_position = np.array([
#         [msg.pose.position.x],
#         [msg.pose.position.y],
#         [msg.pose.position.z],
#         [euler_from_haptic[0]],
#         [euler_from_haptic[1]],
#         [euler_from_haptic[2]]
#         ])

#     # --- Apply initial offset if first message ---
#         if self.haptic_pose_flag:
#             self.haptic_stylus_initial_position = haptic_stylus_position
#             self.haptic_pose_flag = False

#         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

#         return self.haptic_stylus_position
# ## ABOVE IS THE ORIGINAL HAPTIC CALL BACK FUNCTION
    

#     def pose_timestamp_callback(self, event):
#     # Publish the last timestamp from master at fixed 5000 Hz
#         if self.last_received_master_timestamp is not None:
#             self.time_pub.publish(Float64(self.last_received_master_timestamp))


#     @staticmethod
#     def haptic_quat2rpy(quaternion):
#         rotation = R.from_quat(quaternion)
#         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
#         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
#         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
#         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
#         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
#         euler_rpy= R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
#         return euler_rpy

#     def velocity_callback(self, event):
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))
#         self.velocity_pub.publish(velocity_pub_msg)

#     def plot_error_data(self):

#         times = np.array(self.time_stamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, stylus_pose_plot[:, 0] - ee_pose_plot[:, 0])
#         axs[0, 0].set_title('Error in x')
#         axs[0, 0].set_ylabel('Meters')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)

#         axs[1, 0].plot(times, stylus_pose_plot[:, 1] - ee_pose_plot[:, 1])
#         axs[1, 0].set_title('Error in y')
#         axs[1, 0].set_ylabel('Meters')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)

#         axs[2, 0].plot(times, stylus_pose_plot[:, 2] - ee_pose_plot[:, 2])
#         axs[2, 0].set_title('Error in z')
#         axs[2, 0].set_ylabel('Meters')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)

#         axs[0, 1].plot(times, stylus_pose_plot[:, 3] - ee_pose_plot[:, 3])
#         axs[0, 1].set_title('Error in roll')
#         axs[0, 1].set_ylabel('Radians')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)

#         axs[1, 1].plot(times, stylus_pose_plot[:, 4] - ee_pose_plot[:, 4])
#         axs[1, 1].set_title('Error in pitch')
#         axs[1, 1].set_ylabel('Radians')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)

#         axs[2, 1].plot(times, stylus_pose_plot[:, 5] - ee_pose_plot[:, 5])
#         axs[2, 1].set_title('Error in yaw')
#         axs[2, 1].set_ylabel('Radians')
#         axs[2, 1].set_xlabel('Seconds')
#         axs[2, 1].grid(True)

#         plt.tight_layout()
#         plt.show()

#     def plot_data(self):

#         times = np.array(self.time_stamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
#         axs[0, 0].plot(times, stylus_pose_plot[:, 0], label='haptic')
#         axs[0, 0].plot(times, ee_pose_plot[:, 0], label='robot')
#         axs[0, 0].set_title('x')
#         axs[0, 0].set_ylabel('Meters')
#         axs[0, 0].set_xlabel('Seconds')
#         axs[0, 0].grid(True)
#         axs[0, 0].legend()

#         axs[1, 0].plot(times, stylus_pose_plot[:, 1], label='haptic')
#         axs[1, 0].plot(times, ee_pose_plot[:, 1], label='robot')
#         axs[1, 0].set_title('y')
#         axs[1, 0].set_ylabel('Meters')
#         axs[1, 0].set_xlabel('Seconds')
#         axs[1, 0].grid(True)
#         axs[1, 0].legend()

#         axs[2, 0].plot(times, stylus_pose_plot[:, 2], label='haptic')
#         axs[2, 0].plot(times, ee_pose_plot[:, 2], label='robot')
#         axs[2, 0].set_title('z')
#         axs[2, 0].set_ylabel('Meters')
#         axs[2, 0].set_xlabel('Seconds')
#         axs[2, 0].grid(True)
#         axs[2, 0].legend()

#         axs[0, 1].plot(times, stylus_pose_plot[:, 3], label='haptic')
#         axs[0, 1].plot(times, ee_pose_plot[:, 3], label='robot')
#         axs[0, 1].set_title('roll')
#         axs[0, 1].set_ylabel('Radians')
#         axs[0, 1].set_xlabel('Seconds')
#         axs[0, 1].grid(True)
#         axs[0, 1].legend()

#         axs[1, 1].plot(times, stylus_pose_plot[:, 4], label='haptic')
#         axs[1, 1].plot(times, ee_pose_plot[:, 4], label='robot')
#         axs[1, 1].set_title('pitch')
#         axs[1, 1].set_ylabel('Radians')
#         axs[1, 1].set_xlabel('Seconds')
#         axs[1, 1].grid(True)
#         axs[1, 1].legend()

#         axs[2, 1].plot(times, stylus_pose_plot[:, 5], label='haptic')
#         axs[2, 1].plot(times, ee_pose_plot[:, 5], label='robot')
#         axs[2, 1].set_title('yaw')
#         axs[2, 1].set_ylabel('Radians')
#         axs[2, 1].set_xlabel('Seconds')
#         axs[2, 1].grid(True)
#         axs[2, 1].legend()

#         plt.tight_layout()
#         plt.show()

#     def plot_planes(self):

#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         fig, axs = plt.subplots(1, 3, figsize=(19, 4.5))
        
#         # Plot the xy plane
#         axs[0].plot(ee_pose_plot[:, 0], ee_pose_plot[:, 1], label='End-Effector')
#         axs[0].plot(stylus_pose_plot[:, 0], stylus_pose_plot[:, 1], label='Stylus')
#         axs[0].set_title('XY Plane')
#         axs[0].set_xlabel('X')
#         axs[0].set_ylabel('Y')
#         axs[0].axis('equal')
#         axs[0].grid(True)
#         axs[0].legend()

#         # Plot the yz plane
#         axs[1].plot(ee_pose_plot[:, 1], ee_pose_plot[:, 2], label='End-Effector')
#         axs[1].plot(stylus_pose_plot[:, 1], stylus_pose_plot[:, 2], label='Stylus')
#         axs[1].set_title('YZ Plane')
#         axs[1].set_xlabel('Y')
#         axs[1].set_ylabel('Z')
#         axs[1].axis('equal')
#         axs[1].grid(True)
#         axs[1].legend()

#         # Plot the zx plane
#         axs[2].plot(ee_pose_plot[:, 2], ee_pose_plot[:, 0], label='End-Effector')
#         axs[2].plot(stylus_pose_plot[:, 2], stylus_pose_plot[:, 0], label='Stylus')
#         axs[2].set_title('ZX Plane')
#         axs[2].set_xlabel('Z')
#         axs[2].set_ylabel('X')
#         axs[2].axis('equal')
#         axs[2].grid(True)
#         axs[2].legend()

#         plt.tight_layout()
#         plt.show()

#     def plot_all_data(self):
#         times = np.array(self.time_stamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         fig, axs = plt.subplots(6, 2, figsize=(20, 24))

#         # Plot error data
#         errors = ['Error in x', 'Error in y', 'Error in z', 'Error in roll', 'Error in pitch', 'Error in yaw']
#         units = ['Meters', 'Meters', 'Meters', 'Radians', 'Radians', 'Radians']
#         for i in range(6):
#             axs[i, 0].plot(times, stylus_pose_plot[:, i] - ee_pose_plot[:, i])
#             axs[i, 0].set_title(errors[i])
#             axs[i, 0].set_ylabel(units[i])
#             axs[i, 0].set_xlabel('Seconds')
#             axs[i, 0].grid(True)

#         # Plot pose data
#         poses = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
#         for i in range(6):
#             axs[i, 1].plot(times, stylus_pose_plot[:, i], label='haptic')
#             axs[i, 1].plot(times, ee_pose_plot[:, i], label='robot')
#             axs[i, 1].set_title(poses[i])
#             axs[i, 1].set_ylabel(units[i])
#             axs[i, 1].set_xlabel('Seconds')
#             axs[i, 1].grid(True)
#             axs[i, 1].legend()

#         plt.tight_layout()
#         plt.show()

#         # Plot planes
#         fig, axs = plt.subplots(1, 3, figsize=(19, 4.5))
        
#         # Plot the xy plane
#         axs[0].plot(ee_pose_plot[:, 0], ee_pose_plot[:, 1], label='End-Effector')
#         axs[0].plot(stylus_pose_plot[:, 0], stylus_pose_plot[:, 1], label='Stylus')
#         axs[0].set_title('XY Plane')
#         axs[0].set_xlabel('X')
#         axs[0].set_ylabel('Y')
#         axs[0].axis('equal')
#         axs[0].grid(True)
#         axs[0].legend()

#         # Plot the yz plane
#         axs[1].plot(ee_pose_plot[:, 1], ee_pose_plot[:, 2], label='End-Effector')
#         axs[1].plot(stylus_pose_plot[:, 1], stylus_pose_plot[:, 2], label='Stylus')
#         axs[1].set_title('YZ Plane')
#         axs[1].set_xlabel('Y')
#         axs[1].set_ylabel('Z')
#         axs[1].axis('equal')
#         axs[1].grid(True)
#         axs[1].legend()

#         # Plot the zx plane
#         axs[2].plot(ee_pose_plot[:, 2], ee_pose_plot[:, 0], label='End-Effector')
#         axs[2].plot(stylus_pose_plot[:, 2], stylus_pose_plot[:, 0], label='Stylus')
#         axs[2].set_title('ZX Plane')
#         axs[2].set_xlabel('Z')
#         axs[2].set_ylabel('X')
#         axs[2].axis('equal')
#         axs[2].grid(True)
#         axs[2].legend()

#         plt.tight_layout()
#         plt.show()


#     def make_csv(self):
#         times = np.array(self.haptic_timestamps)
#         ee_pose_plot = np.array(self.ee_pose_plot)
#         stylus_pose_plot = np.array(self.stylus_pose_plot)

#         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
#                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
#                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

#             for i in range(len(times)):
#                 current_time = self.start_time + times[i]  # Just the wall-clock time at which this was recorded

#                 csvwriter.writerow([
#                     times[i],                     
#                     self.time_stamps[i],                  
#                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
#                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
#                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
#                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
#                 ])


#     # def make_csv_packet_times(self):
#     # # Convert to numpy for easier handling
#     #     packet_times = np.array(self.packet_times)  # list of (received, applied, diff)

#     #     with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#     #         csvwriter = csv.writer(csvfile)
#     #         csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#     #         for i in range(len(packet_times)):
#     #             csvwriter.writerow([
#     #                 packet_times[i][0],   # Received_Time
#     #                 packet_times[i][1],   # Applied_Time
#     #                 packet_times[i][2]    # Difference
#     #             ])
    
#     def make_csv_packet_times(self):
#         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

#             for received, applied, diff in self.packet_times:
#                 csvwriter.writerow([received, applied, diff])

#     def main_loop(self):
#     # 500 Hz for robot velocity
#         rospy.Timer(rospy.Duration(1.0/500.0), self.velocity_callback)

#     # 5000 Hz for pose timestamp back to master
#         rospy.Timer(rospy.Duration(1.0/500.0), self.pose_timestamp_callback)

#         rospy.spin()

# # if __name__ == "__main__":
# #     try:
# #         controller = RobotEndEffectorController()
# #         controller.main_loop()
# #     except rospy.ROSInterruptException:
# #         pass
# #     finally:
# #         controller.make_csv()
# #         controller.make_csv_packet_times()

# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         try:
#             controller.make_csv()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose.csv: {e}")

#         try:
#             controller.make_csv_packet_times()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_processing.csv: {e}")

#         #controller.plot_error_data()
#         #controller.plot_data()
#         #controller.plot_planes()
#         #controller.plot_all_data()


# #Implement:time differnce=Time differnce when pose timestamps received from the master and when its get sent back again to the master side

# # import rospy
# # import numpy as np
# # from geometry_msgs.msg import PoseStamped
# # from std_msgs.msg import Float64MultiArray
# # from scipy.spatial.transform import Rotation as R

# # from sensor_msgs.msg import JointState
# # import matplotlib.pyplot as plt
# # from std_msgs.msg import Float64
# # import csv

# # class RobotEndEffectorController:

# #     def __init__(self):
# #         # Initializing node
# #         rospy.init_node('robot_end_effector_controller', anonymous=True)
# #         self.forwarded_time_stamps = []
# #         self.last_received_master_timestamp = None
# #         self.last_received_master_force_timestamp = None
# #         self.start_time = rospy.get_time()
# #         self.packet_times = [] 
# #         self.last_received_master_timestamp = None

# #         self.packet_times = [] 
# #         self.last_received_master_timestamp = None

# #         # --- NEW: for pose timestamp round-trip logging ---
# #         self.last_pose_ts_receive_time = None  # when Float64 ts is received at slave
# #         self.pose_ts_rtd = []  # list of (master_ts, recv_time, send_time, diff)


# #         # Publishers
# #         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
# #         self.time_back_to_master_pose_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=1)
# #         self.time_back_to_master_force_pub = rospy.Publisher('time_from_slave_to_master_for_force', Float64, queue_size=1)
# #         self.haptic_timestamps = []
# #         #self.time_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=1, tcp_nodelay=True)
# #         self.time_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=10)

        


# #         # Subscribers
# #         rospy.Subscriber('pose_from_master_to_slave', PoseStamped, self.haptic_callback)
# #         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
# #         rospy.Subscriber('time_from_master_to_slave_for_pose', Float64, self.time_from_master_to_slave_for_pose_callback)
# #         rospy.Subscriber('time_from_master_to_slave_for_force', Float64, self.time_from_master_to_slave_for_force_callback)



# #         # For initialization parameter
# #         self.robot_pose_flag = True
# #         self.haptic_pose_flag = True 
# #         self.initial_ee_pose = np.zeros((6, 1))
# #         self.current_ee_pose = np.zeros((6, 1))
# #         self.joint_position_robot = np.zeros(6)
# #         self.haptic_stylus_position = np.zeros((6, 1))
# #         self.haptic_stylus_initial_position = np.zeros((6,1))

# #         # For unwrapping
# #         self.robot_previous_angles = None
# #         self.haptic_previous_angles = None

# #         # For plotting
# #         self.start_time = None
# #         self.time_stamps = []
# #         self.ee_pose_plot = []
# #         self.stylus_pose_plot = []

# #     @staticmethod
# #     def unwrap_angle(angle, previous_angle):
# #         if previous_angle is None:
# #             return angle
# #         delta = angle - previous_angle
# #         delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
# #         delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
# #         delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
# #         return previous_angle + delta
    
# #     def time_from_master_to_slave_for_force_callback(self, msg):
# #         self.last_received_master_force_timestamp = msg.data

    
# #     # def time_from_master_to_slave_for_pose_callback(self, msg: Float64):
# #     #     self.time_pub.publish(msg)
# #     #     self.forwarded_time_stamps.append(msg.data)
    
# #     def time_from_master_to_slave_for_pose_callback(self, msg):
# #         # existing logic
# #         self.last_received_master_timestamp = msg.data
# #         self.forwarded_time_stamps.append(msg.data)

# #         # --- NEW: log when this timestamp was received at the slave (slave clock) ---
# #         self.last_pose_ts_receive_time = rospy.get_time()

# #     @staticmethod
# #     def Jointspace2GeometricJacobian(joint_position_robot):
# #         t1, t2, t3, t4, t5, t6 = joint_position_robot
# #         cos, sin = np.cos, np.sin
# #         J_geometrical = np.array([
# #             [
# #                 (2621*cos(t1))/20000 + (2371*cos(t1)*cos(t5))/10000 + (4871*cos(t2)*sin(t1))/20000 - 
# #                 (533*sin(t1)*sin(t2)*sin(t3))/2500 + (2371*cos(t2 + t3 + t4)*sin(t1)*sin(t5))/10000 - 
# #                 (1707*cos(t2 + t3)*sin(t1)*sin(t4))/20000 - (1707*sin(t2 + t3)*cos(t4)*sin(t1))/20000 + 
# #                 (533*cos(t2)*cos(t3)*sin(t1))/2500, 
# #                 cos(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
# #                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
# #                 (1707*cos(t2 + t3)*cos(t4))/20000), 
# #                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
# #                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
# #                 cos(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
# #                 (2371*cos(t1)*cos(t2)*cos(t5)*sin(t3)*sin(t4))/10000 - 
# #                 (2371*cos(t1)*cos(t2)*cos(t3)*cos(t4)*cos(t5))/10000 - (2371*sin(t1)*sin(t5))/10000 + 
# #                 (2371*cos(t1)*cos(t3)*cos(t5)*sin(t2)*sin(t4))/10000 + 
# #                 (2371*cos(t1)*cos(t4)*cos(t5)*sin(t2)*sin(t3))/10000, 
# #                 0
# #             ],
# #             [
# #                 (2621*sin(t1))/20000 - (4871*cos(t1)*cos(t2))/20000 + (2371*cos(t5)*sin(t1))/10000 + 
# #                 (533*cos(t1)*sin(t2)*sin(t3))/2500 - (2371*cos(t2 + t3 + t4)*cos(t1)*sin(t5))/10000 + 
# #                 (1707*cos(t2 + t3)*cos(t1)*sin(t4))/20000 + (1707*sin(t2 + t3)*cos(t1)*cos(t4))/20000 - 
# #                 (533*cos(t1)*cos(t2)*cos(t3))/2500, 
# #                 sin(t1)*((533*sin(t2 + t3))/2500 + (4871*sin(t2))/20000 - (1707*sin(t2 + t3)*sin(t4))/20000 + 
# #                 sin(t5)*((2371*cos(t2 + t3)*sin(t4))/10000 + (2371*sin(t2 + t3)*cos(t4))/10000) + 
# #                 (1707*cos(t2 + t3)*cos(t4))/20000), 
# #                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (533*sin(t2 + t3))/2500 + 
# #                 (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
# #                 sin(t1)*((1707*cos(t2 + t3 + t4))/20000 + (2371*sin(t2 + t3 + t4)*sin(t5))/10000), 
# #                 (2371*cos(t1)*sin(t5))/10000 - (2371*cos(t2)*cos(t3)*cos(t4)*cos(t5)*sin(t1))/10000 + 
# #                 (2371*cos(t2)*cos(t5)*sin(t1)*sin(t3)*sin(t4))/10000 + 
# #                 (2371*cos(t3)*cos(t5)*sin(t1)*sin(t2)*sin(t4))/10000 + 
# #                 (2371*cos(t4)*cos(t5)*sin(t1)*sin(t2)*sin(t3))/10000, 
# #                 0
# #             ],
# #             [
# #                 0, 
# #                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
# #                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500 - (4871*cos(t2))/20000, 
# #                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
# #                 (2371*sin(t2 + t3 + t4 + t5))/20000 - (533*cos(t2 + t3))/2500, 
# #                 (2371*sin(t2 + t3 + t4 - t5))/20000 + (1707*sin(t2 + t3 + t4))/20000 - 
# #                 (2371*sin(t2 + t3 + t4 + t5))/20000, 
# #                 - (2371*sin(t2 + t3 + t4 - t5))/20000 - (2371*sin(t2 + t3 + t4 + t5))/20000, 
# #                 0
# #             ],
# #             [
# #                 0, 
# #                 sin(t1), 
# #                 sin(t1), 
# #                 sin(t1), 
# #                 sin(t2 + t3 + t4)*cos(t1), 
# #                 cos(t5)*sin(t1) - cos(t2 + t3 + t4)*cos(t1)*sin(t5)
# #             ],
# #             [
# #                 0, 
# #                 -cos(t1), 
# #                 -cos(t1), 
# #                 -cos(t1), 
# #                 sin(t2 + t3 + t4)*sin(t1), 
# #                 - cos(t1)*cos(t5) - cos(t2 + t3 + t4)*sin(t1)*sin(t5)
# #             ],
# #             [
# #                 1, 
# #                 0, 
# #                 0, 
# #                 0, 
# #                 -cos(t2 + t3 + t4), 
# #                 -sin(t2 + t3 + t4)*sin(t5)
# #             ]
# #         ])
# #         lamda = 0.001
# #         return J_geometrical + lamda * np.eye(6)
    
# #     def rpy2joint_space_vel(self):

# #         alpha, beta, gamma = self.current_ee_pose[3:6, 0]
# #         cos, sin = np.cos, np.sin
        
# #         J_geo2ana = np.array([[1, 0, 0,                    0,           0, 0],
# #                               [0, 1, 0,                    0,           0, 0],
# #                               [0, 0, 1,                    0,           0, 0],
# #                               [0, 0, 0, cos(beta)*cos(gamma), -sin(gamma), 0],
# #                               [0, 0, 0, cos(beta)*sin(gamma),  cos(gamma), 0],
# #                               [0, 0, 0,           -sin(beta),           0, 1]])

# #         J_geometrical = self.Jointspace2GeometricJacobian(self.joint_position_robot)

# #         c = 8  #Original c ws 8
# #         k = np.array([[c, 0, 0, 0, 0, 0],
# #                       [0, c, 0, 0, 0, 0],
# #                       [0, 0, c, 0, 0, 0],
# #                       [0, 0, 0, c, 0, 0],
# #                       [0, 0, 0, 0, c, 0],
# #                       [0, 0, 0, 0, 0, c]])
 
# #         ee_pose = self.current_ee_pose - self.initial_ee_pose
# #         joint_space_velocity = k @ np.linalg.inv(J_geometrical) @ J_geo2ana @ (self.haptic_stylus_position - ee_pose)
        
# #         # Log data
# #         current_time = rospy.get_time()
# #         if self.start_time is None:
# #             self.start_time = current_time
# #         self.time_stamps.append(current_time - self.start_time)
# #         self.ee_pose_plot.append(ee_pose.flatten())
# #         self.stylus_pose_plot.append(self.haptic_stylus_position.flatten())
        
# #         return joint_space_velocity.flatten()

# #     def pose_and_rotation(self,joint_position_robot):
# #         q1, q2, q3, q4, q5, q6 = joint_position_robot

# #         cos = np.cos
# #         sin = np.sin
# #         a1, a2, a3, a4, a5, a6, a7, a8, a9 = 2621, 4871, 2371, 1707, 533, 3037, 20000, 2500, 10000
        
# #         # Calculate each component of the position
# #         x = (a1 * sin(q1)) / a7 - (a2 * cos(q1) * cos(q2)) / a7 + (a3 * cos(q5) * sin(q1)) / a9 - \
# #             (a3 * cos(q2 + q3 + q4) * cos(q1) * sin(q5)) / a9 + (a4 * cos(q2 + q3) * cos(q1) * sin(q4)) / a7 + \
# #             (a4 * sin(q2 + q3) * cos(q1) * cos(q4)) / a7 - (a5 * cos(q1) * cos(q2) * cos(q3)) / a8 + \
# #             (a5 * cos(q1) * sin(q2) * sin(q3)) / a8

# #         y = (a5 * sin(q1) * sin(q2) * sin(q3)) / a8 - (a3 * cos(q1) * cos(q5)) / a9 - \
# #             (a2 * cos(q2) * sin(q1)) / a7 - (a1 * cos(q1)) / a7 - (a3 * cos(q2 + q3 + q4) * sin(q1) * sin(q5)) / a9 + \
# #             (a4 * cos(q2 + q3) * sin(q1) * sin(q4)) / a7 + (a4 * sin(q2 + q3) * cos(q4) * sin(q1)) / a7 - \
# #             (a5 * cos(q2) * cos(q3) * sin(q1)) / a8

# #         z = (a4 * sin(q2 + q3) * sin(q4)) / a7 - (a2 * sin(q2)) / a7 - sin(q5) * ((a3 * cos(q2 + q3) * sin(q4)) / a9 + \
# #             (a3 * sin(q2 + q3) * cos(q4)) / a9) - (a4 * cos(q2 + q3) * cos(q4)) / a7 - (a5 * sin(q2 + q3)) / a8 + a6 / a7

# #         # Compute each element of the rotation matrix
# #         r11 = cos(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6)
# #         r12 = -sin(q6)*(sin(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6)
# #         r13 = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)
        
# #         r21 = -cos(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6)
# #         r22 = sin(q6)*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1)
# #         r23 = -cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)
        
# #         r31 = cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)
# #         r32 = cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)
# #         r33 = -sin(q2 + q3 + q4)*sin(q5)
        
# #         R_matrix = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
       
# #         rotation = R.from_matrix(R_matrix)
# #         euler_angles = rotation.as_euler('xyz', degrees=False)
# #         euler_angles = np.array(euler_angles)
# #         if self.robot_previous_angles is not None:
# #             euler_angles = self.unwrap_angle(euler_angles, self.robot_previous_angles)
# #         self.robot_previous_angles = euler_angles

# #         return np.array([[x], [y], [z], [euler_angles[0]], [euler_angles[1]], [euler_angles[2]]])

# #     def joint_callback_robot(self, msg: JointState):
# #         self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
# #         self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
# #         if self.robot_pose_flag:
# #             self.initial_ee_pose = self.current_ee_pose
# #             self.robot_pose_flag = False

# #     #def haptic_callback(self, msg: PoseStamped):
# #     # Store header timestamp from incoming haptic pose
# #     #     self.haptic_timestamps.append(msg.header.stamp.to_sec())

# #     #     euler_from_haptic = self.haptic_quat2rpy([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
# #     #     if self.haptic_previous_angles is not None:
# #     #         euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
# #     #     self.haptic_previous_angles = euler_from_haptic

# #     #     haptic_stylus_position = np.array([[msg.pose.position.x],[msg.pose.position.y],[msg.pose.position.z],
# #     #                                    [euler_from_haptic[0]],[euler_from_haptic[1]],[euler_from_haptic[2]]])
    
# #     #     if self.haptic_pose_flag:
# #     #         self.haptic_stylus_initial_position = haptic_stylus_position
# #     #         self.haptic_pose_flag = False

# #     #     self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

# #     # # --- Send back the master timestamp for delay calculation ---
# #     #     if self.last_received_master_timestamp is not None:
# #     #         self.time_pub.publish(Float64(self.last_received_master_timestamp))

# #     #     return self.haptic_stylus_position
 
# # ##THIS IS THE ORIGINAL HAPTIC CALLBACK FUNCTION
 
# #     def haptic_callback(self, msg: PoseStamped):
# #     # --- Store and forward header timestamp from master ---
# #         timestamp = msg.header.stamp.to_sec()
# #         self.haptic_timestamps.append(timestamp)

# # # New line
# #         self.last_packet_received_time = rospy.get_time()
# #     # Publish the same timestamp back to master for delay logging
# #         #self.time_pub.publish(Float64(timestamp))

# #     # --- Pose and orientation processing ---
# #         euler_from_haptic = self.haptic_quat2rpy([
# #         msg.pose.orientation.x,
# #         msg.pose.orientation.y,
# #         msg.pose.orientation.z,
# #         msg.pose.orientation.w
# #     ])

# #         if self.haptic_previous_angles is not None:
# #             euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
# #         self.haptic_previous_angles = euler_from_haptic

# #         haptic_stylus_position = np.array([
# #         [msg.pose.position.x],
# #         [msg.pose.position.y],
# #         [msg.pose.position.z],
# #         [euler_from_haptic[0]],
# #         [euler_from_haptic[1]],
# #         [euler_from_haptic[2]]
# #         ])

# #     # --- Apply initial offset if first message ---
# #         if self.haptic_pose_flag:
# #             self.haptic_stylus_initial_position = haptic_stylus_position
# #             self.haptic_pose_flag = False

# #         self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

# #         return self.haptic_stylus_position
# # ## ABOVE IS THE ORIGINAL HAPTIC CALL BACK FUNCTION
    

# #     def pose_timestamp_callback(self, event):
# #         # Publish the last timestamp from master at fixed 500 Hz
# #         if self.last_received_master_timestamp is not None:
# #             send_time = rospy.get_time()  # --- NEW: when we send back to master (slave clock)

# #             # existing behaviour: publish back to master
# #             self.time_pub.publish(Float64(self.last_received_master_timestamp))

# #             # --- NEW: log RTD-on-slave for this timestamp ---
# #             if self.last_pose_ts_receive_time is not None:
# #                 diff = send_time - self.last_pose_ts_receive_time
# #                 self.pose_ts_rtd.append((
# #                     self.last_received_master_timestamp,  # timestamp value from master
# #                     self.last_pose_ts_receive_time,       # when received at slave
# #                     send_time,                            # when sent back from slave
# #                     diff                                  # difference (seconds)
# #                 ))

# #     @staticmethod
# #     def haptic_quat2rpy(quaternion):
# #         rotation = R.from_quat(quaternion)
# #         resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_matrix()
# #         resulting_matrix[0, 1] = -resulting_matrix[0, 1]
# #         resulting_matrix[0, 2] = -resulting_matrix[0, 2]
# #         resulting_matrix[1, 0] = -resulting_matrix[1, 0]
# #         resulting_matrix[2, 0] = -resulting_matrix[2, 0]
# #         euler_rpy= R.from_matrix(resulting_matrix).as_euler('xyz', degrees=False)
# #         return euler_rpy

# #     def velocity_callback(self, event):
# #         velocity_pub_msg = Float64MultiArray()
# #         velocity_pub_msg.data = self.rpy2joint_space_vel()
# #         applied_time = rospy.get_time()
# #         if hasattr(self, "last_packet_received_time"):
# #             diff = applied_time - self.last_packet_received_time
# #             self.packet_times.append((
# #                 self.last_packet_received_time, 
# #                 applied_time, 
# #                 diff))
# #         self.velocity_pub.publish(velocity_pub_msg)

# #     def plot_error_data(self):

# #         times = np.array(self.time_stamps)
# #         ee_pose_plot = np.array(self.ee_pose_plot)
# #         stylus_pose_plot = np.array(self.stylus_pose_plot)

# #         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
# #         axs[0, 0].plot(times, stylus_pose_plot[:, 0] - ee_pose_plot[:, 0])
# #         axs[0, 0].set_title('Error in x')
# #         axs[0, 0].set_ylabel('Meters')
# #         axs[0, 0].set_xlabel('Seconds')
# #         axs[0, 0].grid(True)

# #         axs[1, 0].plot(times, stylus_pose_plot[:, 1] - ee_pose_plot[:, 1])
# #         axs[1, 0].set_title('Error in y')
# #         axs[1, 0].set_ylabel('Meters')
# #         axs[1, 0].set_xlabel('Seconds')
# #         axs[1, 0].grid(True)

# #         axs[2, 0].plot(times, stylus_pose_plot[:, 2] - ee_pose_plot[:, 2])
# #         axs[2, 0].set_title('Error in z')
# #         axs[2, 0].set_ylabel('Meters')
# #         axs[2, 0].set_xlabel('Seconds')
# #         axs[2, 0].grid(True)

# #         axs[0, 1].plot(times, stylus_pose_plot[:, 3] - ee_pose_plot[:, 3])
# #         axs[0, 1].set_title('Error in roll')
# #         axs[0, 1].set_ylabel('Radians')
# #         axs[0, 1].set_xlabel('Seconds')
# #         axs[0, 1].grid(True)

# #         axs[1, 1].plot(times, stylus_pose_plot[:, 4] - ee_pose_plot[:, 4])
# #         axs[1, 1].set_title('Error in pitch')
# #         axs[1, 1].set_ylabel('Radians')
# #         axs[1, 1].set_xlabel('Seconds')
# #         axs[1, 1].grid(True)

# #         axs[2, 1].plot(times, stylus_pose_plot[:, 5] - ee_pose_plot[:, 5])
# #         axs[2, 1].set_title('Error in yaw')
# #         axs[2, 1].set_ylabel('Radians')
# #         axs[2, 1].set_xlabel('Seconds')
# #         axs[2, 1].grid(True)

# #         plt.tight_layout()
# #         plt.show()

# #     def plot_data(self):

# #         times = np.array(self.time_stamps)
# #         ee_pose_plot = np.array(self.ee_pose_plot)
# #         stylus_pose_plot = np.array(self.stylus_pose_plot)

# #         fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
# #         axs[0, 0].plot(times, stylus_pose_plot[:, 0], label='haptic')
# #         axs[0, 0].plot(times, ee_pose_plot[:, 0], label='robot')
# #         axs[0, 0].set_title('x')
# #         axs[0, 0].set_ylabel('Meters')
# #         axs[0, 0].set_xlabel('Seconds')
# #         axs[0, 0].grid(True)
# #         axs[0, 0].legend()

# #         axs[1, 0].plot(times, stylus_pose_plot[:, 1], label='haptic')
# #         axs[1, 0].plot(times, ee_pose_plot[:, 1], label='robot')
# #         axs[1, 0].set_title('y')
# #         axs[1, 0].set_ylabel('Meters')
# #         axs[1, 0].set_xlabel('Seconds')
# #         axs[1, 0].grid(True)
# #         axs[1, 0].legend()

# #         axs[2, 0].plot(times, stylus_pose_plot[:, 2], label='haptic')
# #         axs[2, 0].plot(times, ee_pose_plot[:, 2], label='robot')
# #         axs[2, 0].set_title('z')
# #         axs[2, 0].set_ylabel('Meters')
# #         axs[2, 0].set_xlabel('Seconds')
# #         axs[2, 0].grid(True)
# #         axs[2, 0].legend()

# #         axs[0, 1].plot(times, stylus_pose_plot[:, 3], label='haptic')
# #         axs[0, 1].plot(times, ee_pose_plot[:, 3], label='robot')
# #         axs[0, 1].set_title('roll')
# #         axs[0, 1].set_ylabel('Radians')
# #         axs[0, 1].set_xlabel('Seconds')
# #         axs[0, 1].grid(True)
# #         axs[0, 1].legend()

# #         axs[1, 1].plot(times, stylus_pose_plot[:, 4], label='haptic')
# #         axs[1, 1].plot(times, ee_pose_plot[:, 4], label='robot')
# #         axs[1, 1].set_title('pitch')
# #         axs[1, 1].set_ylabel('Radians')
# #         axs[1, 1].set_xlabel('Seconds')
# #         axs[1, 1].grid(True)
# #         axs[1, 1].legend()

# #         axs[2, 1].plot(times, stylus_pose_plot[:, 5], label='haptic')
# #         axs[2, 1].plot(times, ee_pose_plot[:, 5], label='robot')
# #         axs[2, 1].set_title('yaw')
# #         axs[2, 1].set_ylabel('Radians')
# #         axs[2, 1].set_xlabel('Seconds')
# #         axs[2, 1].grid(True)
# #         axs[2, 1].legend()

# #         plt.tight_layout()
# #         plt.show()

# #     def plot_planes(self):

# #         ee_pose_plot = np.array(self.ee_pose_plot)
# #         stylus_pose_plot = np.array(self.stylus_pose_plot)

# #         fig, axs = plt.subplots(1, 3, figsize=(19, 4.5))
        
# #         # Plot the xy plane
# #         axs[0].plot(ee_pose_plot[:, 0], ee_pose_plot[:, 1], label='End-Effector')
# #         axs[0].plot(stylus_pose_plot[:, 0], stylus_pose_plot[:, 1], label='Stylus')
# #         axs[0].set_title('XY Plane')
# #         axs[0].set_xlabel('X')
# #         axs[0].set_ylabel('Y')
# #         axs[0].axis('equal')
# #         axs[0].grid(True)
# #         axs[0].legend()

# #         # Plot the yz plane
# #         axs[1].plot(ee_pose_plot[:, 1], ee_pose_plot[:, 2], label='End-Effector')
# #         axs[1].plot(stylus_pose_plot[:, 1], stylus_pose_plot[:, 2], label='Stylus')
# #         axs[1].set_title('YZ Plane')
# #         axs[1].set_xlabel('Y')
# #         axs[1].set_ylabel('Z')
# #         axs[1].axis('equal')
# #         axs[1].grid(True)
# #         axs[1].legend()

# #         # Plot the zx plane
# #         axs[2].plot(ee_pose_plot[:, 2], ee_pose_plot[:, 0], label='End-Effector')
# #         axs[2].plot(stylus_pose_plot[:, 2], stylus_pose_plot[:, 0], label='Stylus')
# #         axs[2].set_title('ZX Plane')
# #         axs[2].set_xlabel('Z')
# #         axs[2].set_ylabel('X')
# #         axs[2].axis('equal')
# #         axs[2].grid(True)
# #         axs[2].legend()

# #         plt.tight_layout()
# #         plt.show()

# #     def plot_all_data(self):
# #         times = np.array(self.time_stamps)
# #         ee_pose_plot = np.array(self.ee_pose_plot)
# #         stylus_pose_plot = np.array(self.stylus_pose_plot)

# #         fig, axs = plt.subplots(6, 2, figsize=(20, 24))

# #         # Plot error data
# #         errors = ['Error in x', 'Error in y', 'Error in z', 'Error in roll', 'Error in pitch', 'Error in yaw']
# #         units = ['Meters', 'Meters', 'Meters', 'Radians', 'Radians', 'Radians']
# #         for i in range(6):
# #             axs[i, 0].plot(times, stylus_pose_plot[:, i] - ee_pose_plot[:, i])
# #             axs[i, 0].set_title(errors[i])
# #             axs[i, 0].set_ylabel(units[i])
# #             axs[i, 0].set_xlabel('Seconds')
# #             axs[i, 0].grid(True)

# #         # Plot pose data
# #         poses = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
# #         for i in range(6):
# #             axs[i, 1].plot(times, stylus_pose_plot[:, i], label='haptic')
# #             axs[i, 1].plot(times, ee_pose_plot[:, i], label='robot')
# #             axs[i, 1].set_title(poses[i])
# #             axs[i, 1].set_ylabel(units[i])
# #             axs[i, 1].set_xlabel('Seconds')
# #             axs[i, 1].grid(True)
# #             axs[i, 1].legend()

# #         plt.tight_layout()
# #         plt.show()

# #         # Plot planes
# #         fig, axs = plt.subplots(1, 3, figsize=(19, 4.5))
        
# #         # Plot the xy plane
# #         axs[0].plot(ee_pose_plot[:, 0], ee_pose_plot[:, 1], label='End-Effector')
# #         axs[0].plot(stylus_pose_plot[:, 0], stylus_pose_plot[:, 1], label='Stylus')
# #         axs[0].set_title('XY Plane')
# #         axs[0].set_xlabel('X')
# #         axs[0].set_ylabel('Y')
# #         axs[0].axis('equal')
# #         axs[0].grid(True)
# #         axs[0].legend()

# #         # Plot the yz plane
# #         axs[1].plot(ee_pose_plot[:, 1], ee_pose_plot[:, 2], label='End-Effector')
# #         axs[1].plot(stylus_pose_plot[:, 1], stylus_pose_plot[:, 2], label='Stylus')
# #         axs[1].set_title('YZ Plane')
# #         axs[1].set_xlabel('Y')
# #         axs[1].set_ylabel('Z')
# #         axs[1].axis('equal')
# #         axs[1].grid(True)
# #         axs[1].legend()

# #         # Plot the zx plane
# #         axs[2].plot(ee_pose_plot[:, 2], ee_pose_plot[:, 0], label='End-Effector')
# #         axs[2].plot(stylus_pose_plot[:, 2], stylus_pose_plot[:, 0], label='Stylus')
# #         axs[2].set_title('ZX Plane')
# #         axs[2].set_xlabel('Z')
# #         axs[2].set_ylabel('X')
# #         axs[2].axis('equal')
# #         axs[2].grid(True)
# #         axs[2].legend()

# #         plt.tight_layout()
# #         plt.show()


# #     def make_csv(self):
# #         times = np.array(self.haptic_timestamps)
# #         ee_pose_plot = np.array(self.ee_pose_plot)
# #         stylus_pose_plot = np.array(self.stylus_pose_plot)

# #         with open('/home/user/Desktop/delay/pose.csv', 'w', newline='') as csvfile:
# #             csvwriter = csv.writer(csvfile)
# #             csvwriter.writerow(['Time Stamps Received(pose)from Master', 'Time', 
# #                             'EE Pose X', 'EE Pose Y', 'EE Pose Z', 'EE Pose Roll', 'EE Pose Pitch', 'EE Pose Yaw',
# #                             'Stylus Pose X', 'Stylus Pose Y', 'Stylus Pose Z', 'Stylus Pose Roll', 'Stylus Pose Pitch', 'Stylus Pose Yaw'])

# #             for i in range(len(times)):
# #                 current_time = self.start_time + times[i]  # Just the wall-clock time at which this was recorded

# #                 csvwriter.writerow([
# #                     times[i],                     
# #                     self.time_stamps[i],                  
# #                     ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
# #                     ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
# #                     stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
# #                     stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
# #                 ])


# #     # def make_csv_packet_times(self):
# #     # # Convert to numpy for easier handling
# #     #     packet_times = np.array(self.packet_times)  # list of (received, applied, diff)

# #     #     with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
# #     #         csvwriter = csv.writer(csvfile)
# #     #         csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

# #     #         for i in range(len(packet_times)):
# #     #             csvwriter.writerow([
# #     #                 packet_times[i][0],   # Received_Time
# #     #                 packet_times[i][1],   # Applied_Time
# #     #                 packet_times[i][2]    # Difference
# #     #             ])
    
# #     def make_csv_packet_times(self):
# #         with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
# #             csvwriter = csv.writer(csvfile)
# #             csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

# #             for received, applied, diff in self.packet_times:
# #                 csvwriter.writerow([received, applied, diff])

# #     def make_csv_pose_timestamp_rtd(self):
# #         with open('/home/user/Desktop/delay/pose_timestamp_processing_slave_2.csv', 'w', newline='') as csvfile:
# #             csvwriter = csv.writer(csvfile)
# #             csvwriter.writerow([
# #                 'Master_Timestamp_Value',
# #                 'Received_Time_Slave',
# #                 'Sent_Back_Time_Slave',
# #                 'Difference_SentMinusReceived'
# #             ])

# #             for master_ts, recv_time, send_time, diff in self.pose_ts_rtd:
# #                 csvwriter.writerow([master_ts, recv_time, send_time, diff])                

# #     def main_loop(self):
# #     # 500 Hz for robot velocity
# #         rospy.Timer(rospy.Duration(1.0/500.0), self.velocity_callback)

# #     # 5000 Hz for pose timestamp back to master
# #         rospy.Timer(rospy.Duration(1.0/500.0), self.pose_timestamp_callback)

# #         rospy.spin()

# # # if __name__ == "__main__":
# # #     try:
# # #         controller = RobotEndEffectorController()
# # #         controller.main_loop()
# # #     except rospy.ROSInterruptException:
# # #         pass
# # #     finally:
# # #         controller.make_csv()
# # #         controller.make_csv_packet_times()

# # if __name__ == "__main__":
# #     try:
# #         controller = RobotEndEffectorController()
# #         controller.main_loop()
# #     except rospy.ROSInterruptException:
# #         pass
# #     finally:
# #         try:
# #             controller.make_csv()
# #         except Exception as e:
# #             rospy.logerr(f"Failed to save pose.csv: {e}")

# #         try:
# #             controller.make_csv_packet_times()
# #         except Exception as e:
# #             rospy.logerr(f"Failed to save pose_processing.csv: {e}")
# #         # --- NEW: save pose timestamp RTD CSV ---
# #         try:
# #             controller.make_csv_pose_timestamp_rtd()
# #         except Exception as e:
# #             rospy.logerr(f"Failed to save pose_timestamp_rtd_slave.csv: {e}")            

# #         #controller.plot_error_data()
# #         #controller.plot_data()
# #         #controller.plot_planes()
# #         #controller.plot_all_data()