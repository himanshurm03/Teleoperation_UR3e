#!/usr/bin/env python3
#9
#Potentially Correct
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
#         #self.velocity_pub.publish(velocity_pub_msg)

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
#         bundle_rate   = 500    # Hz (network transmission)

#         # Velocity control loop (500 Hz)
#         rospy.Timer(
#             rospy.Duration(1.0 / velocity_rate),
#             self.velocity_callback
#         )

#     # Bundled slave → master transmission (1000 Hz)
#         rospy.Timer(
#             rospy.Duration(1.0 / bundle_rate),
#             lambda event: self.send_bundled_to_master()
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


#10
#Some Improvement

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
#         Immediately echoes pose timestamp in bundled form.
#         """
#         # Extract and store timestamp
#         timestamp = msg.header.stamp.to_sec()
#         self.latest_pose_timestamp = timestamp
#         self.new_pose_available = True
#         self.haptic_timestamps.append(timestamp)
#         self.last_packet_received_time = rospy.get_time()
        
#         # Process haptic data
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

#         # Echo force timestamp if present
#         if msg.force_timestamp > 0:
#             echo_msg = Float64()
#             echo_msg.data = msg.force_timestamp
#             self.force_echo_pub.publish(echo_msg)            
#             rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
#         # ✅ IMMEDIATELY echo pose timestamp in bundled form
#         self.pose_timestamp_to_send = timestamp
#         self.send_bundled_to_master()
#         rospy.loginfo_throttle(2.0, f"[POSE] Immediate echo sent for pose_ts: {timestamp:.6f}")

#     def send_bundled_to_master(self):
#         """Send bundled data back to master"""
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()
        
#         if self.pose_timestamp_to_send > 0:
#             bundled_msg.pose_timestamp = self.pose_timestamp_to_send
#             self.pose_timestamp_to_send = 0.0  # Prevent duplicates
#         else:
#             bundled_msg.pose_timestamp = 0.0   # Master ignores
        
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
#         """Velocity control loop - no longer handles pose echo"""
#         if not self.new_pose_available:
#             return
        
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         self.velocity_pub.publish(velocity_pub_msg)
#         self.new_pose_available = False

#         # Log processing delay (optional, for analysis)
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))

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
#         """
#         Main control loop with:
#         - 500 Hz velocity control
#         - 500 Hz regular bundled updates (for force data)
#         - Immediate bundled echo on pose receipt (handled in master_data_callback)
#         """
#         velocity_rate = 500     # Hz (control loop)
#         bundle_rate   = 500     # Hz (regular force updates)

#         # Velocity control loop (500 Hz)
#         rospy.Timer(
#             rospy.Duration(1.0 / velocity_rate),
#             self.velocity_callback
#         )

#         # Regular bundled slave → master transmission (500 Hz)
#         # Mainly for continuous force feedback updates
#         rospy.Timer(
#             rospy.Duration(1.0 / bundle_rate),
#             lambda event: self.send_bundled_to_master()
#         )

#         rospy.loginfo("Control loops started: 500Hz velocity + 500Hz bundled updates + immediate pose echo")
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



#some fix-1
#This has solved the purpose to some extent
#working fine


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

#         # Publisher to forward force echo to force controller
#         self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)

#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # Bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers with INCREASED queue size to prevent backup
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback, queue_size=1)  # Only process latest
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
#         Echoes back EXACTLY the timestamp from msg.header (master's send time).
#         """
#         # ✅ Measure callback timing
#         callback_start = rospy.get_time()
        
#         # Extract master's send timestamp from message header
#         master_send_timestamp = msg.header.stamp.to_sec()
        
#         # ✅ DIAGNOSTIC: Check message age (time between send and receive)
#         message_age = callback_start - master_send_timestamp
#         if message_age > 0.010:  # More than 10ms old
#             rospy.logwarn_throttle(1.0, f"⚠️  OLD MESSAGE! Age: {message_age*1000:.1f}ms - Queue backup detected!")
        
#         self.new_pose_available = True
#         self.haptic_timestamps.append(master_send_timestamp)
#         self.last_packet_received_time = callback_start
        
#         # Process haptic data
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

#         # Echo force timestamp if present
#         if msg.force_timestamp > 0:
#             echo_msg = Float64()
#             echo_msg.data = msg.force_timestamp
#             self.force_echo_pub.publish(echo_msg)            
#             rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
#         # Echo back EXACTLY what master sent (master's send timestamp)
#         self.pose_timestamp_to_send = master_send_timestamp
            
#         self.send_bundled_to_master()
        
#         #DIAGNOSTIC: Measure callback duration
#         callback_duration = rospy.get_time() - callback_start
#         rospy.loginfo_throttle(2.0, f"[POSE] Echo sent | msg_age: {message_age*1000:.2f}ms | callback: {callback_duration*1000:.2f}ms")

#     def send_bundled_to_master(self):
#         """Send bundled data back to master"""
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()
        
#         if self.pose_timestamp_to_send > 0:
#             bundled_msg.pose_timestamp = self.pose_timestamp_to_send
#             self.pose_timestamp_to_send = 0.0  # Prevent duplicates
#         else:
#             bundled_msg.pose_timestamp = 0.0   # Master ignores
        
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
#         """Velocity control loop - no longer handles pose echo"""
#         if not self.new_pose_available:
#             return
        
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         self.velocity_pub.publish(velocity_pub_msg)
#         self.new_pose_available = False

#         # Log processing delay (optional, for analysis)
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))

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
#         """
#         Main control loop with:
#         - 500 Hz velocity control
#         - 500 Hz regular bundled updates (for force data)
#         - Immediate bundled echo on pose receipt (handled in master_data_callback)
#         """
#         velocity_rate = 500     # Hz (control loop)
#         bundle_rate   = 500     # Hz (regular force updates)

#         # Velocity control loop (500 Hz)
#         rospy.Timer(
#             rospy.Duration(1.0 / velocity_rate),
#             self.velocity_callback
#         )

#         # Regular bundled slave → master transmission (500 Hz)
#         # Mainly for continuous force feedback updates
#         rospy.Timer(
#             rospy.Duration(1.0 / bundle_rate),
#             lambda event: self.send_bundled_to_master()
#         )

#         rospy.loginfo("Control loops started: 500Hz velocity + 500Hz bundled updates + immediate pose echo")
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


#5
#some bug fix-2
#working still fine
#FIFO implementation

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
from collections import deque

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

        self.pose_ts_queue = deque()   # (pose_timestamp, arrival_time)
        self.MAX_POSE_WAIT = 1.0 / 500.0  

        
        # Storage for bundled data
        self.latest_force_data = Vector3()
        self.latest_force_data.x = 0.0
        self.latest_force_data.y = 0.0
        self.latest_force_data.z = 0.0
        self.latest_pose_timestamp = 0.0
        self.latest_force_timestamp = 0.0

        # Publisher to forward force echo to force controller
        self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)

        # Publishers
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
        # Bundled publisher to master
        self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
        self.haptic_timestamps = []

        # Subscribers with INCREASED queue size to prevent backup
        rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback, queue_size=1)  # Only process latest
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        
        # Subscribe to internal force data
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
        Echoes back EXACTLY the timestamp from msg.header (master's send time).
        """
        # Measure callback timing
        callback_start = rospy.get_time()
        
        # Extract master's send timestamp from message header
        master_send_timestamp = msg.header.stamp.to_sec()
        
        #  DIAGNOSTIC: Check message age (time between send and receive)
        message_age = callback_start - master_send_timestamp
        if message_age > 0.010:  # More than 10ms old
            rospy.logwarn_throttle(1.0, f"⚠️  OLD MESSAGE! Age: {message_age*1000:.1f}ms - Queue backup detected!")
            
        
        self.new_pose_available = True
        self.haptic_timestamps.append(master_send_timestamp)
        self.last_packet_received_time = callback_start
        
        # Process haptic data
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

        # Echo force timestamp if present
        if msg.force_timestamp > 0:
            echo_msg = Float64()
            echo_msg.data = msg.force_timestamp
            self.force_echo_pub.publish(echo_msg)            
            rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
        # Echo back EXACTLY what master sent (master's send timestamp)
        #self.pose_timestamp_to_send = master_send_timestamp
            
        #self.send_bundled_to_master()

        arrival_time = rospy.get_time()
        self.pose_ts_queue.append((master_send_timestamp, arrival_time))
        
        #DIAGNOSTIC: Measure callback duration
        callback_duration = rospy.get_time() - callback_start
        rospy.loginfo_throttle(2.0, f"[POSE] Echo sent | msg_age: {message_age*1000:.2f}ms | callback: {callback_duration*1000:.2f}ms")

    # def send_bundled_to_master(self):
    #     """Send bundled data back to master"""
    #     bundled_msg = SlaveToMasterData()
    #     bundled_msg.force = self.latest_force_data
    #     bundled_msg.force_timestamp = self.latest_force_timestamp
    #     bundled_msg.header.stamp = rospy.Time.now()
        
    #     if self.pose_timestamp_to_send > 0:
    #         bundled_msg.pose_timestamp = self.pose_timestamp_to_send
    #         self.pose_timestamp_to_send = 0.0  # Prevent duplicates
    #     else:
    #         bundled_msg.pose_timestamp = 0.0   # Master ignores
        
    #     self.slave_to_master_pub.publish(bundled_msg)

    def send_bundled_to_master(self):
        bundled_msg = SlaveToMasterData()
        bundled_msg.force = self.latest_force_data
        bundled_msg.force_timestamp = self.latest_force_timestamp
        bundled_msg.header.stamp = rospy.Time.now()

        now = rospy.get_time()
        pose_ts_to_send = 0.0

    # Drop expired pose timestamps
        while self.pose_ts_queue:
            ts, arrival = self.pose_ts_queue[0]
            if now - arrival > self.MAX_POSE_WAIT:
                self.pose_ts_queue.popleft()  # deadline miss
            else:
                break

    # Pop oldest valid pose timestamp
        if self.pose_ts_queue:
            ts, arrival = self.pose_ts_queue.popleft()
            pose_ts_to_send = ts

        bundled_msg.pose_timestamp = pose_ts_to_send
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
        """Velocity control loop - no longer handles pose echo"""
        if not self.new_pose_available:
            return
        
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.rpy2joint_space_vel()
        self.velocity_pub.publish(velocity_pub_msg)
        self.new_pose_available = False

        # Log processing delay (optional, for analysis)
        applied_time = rospy.get_time()
        if hasattr(self, "last_packet_received_time"):
            diff = applied_time - self.last_packet_received_time
            self.packet_times.append((
                self.last_packet_received_time, 
                applied_time, 
                diff))

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

    def main_loop(self):
        """
        Main control loop with:
        - 500 Hz velocity control
        - 500 Hz regular bundled updates (for force data)
        - Immediate bundled echo on pose receipt (handled in master_data_callback)
        """
        velocity_rate = 500     # Hz (control loop)
        bundle_rate   = 500     # Hz (regular force updates)

        # Velocity control loop (500 Hz)
        rospy.Timer(
            rospy.Duration(1.0 / velocity_rate),
            self.velocity_callback
        )

        # Regular bundled slave → master transmission (500 Hz)
        # Mainly for continuous force feedback updates
        rospy.Timer(
            rospy.Duration(1.0 / bundle_rate),
            lambda event: self.send_bundled_to_master()
        )

        rospy.loginfo("Control loops started: 500Hz velocity + 500Hz bundled updates + immediate pose echo")
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



#Above code is perfectly fine
#added the pose timestamp waiting time


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
# from collections import deque

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

#         self.pose_ts_queue = deque()   # (pose_timestamp, arrival_time)
#         self.MAX_POSE_WAIT = 1.0 / 500.0  

#         # NEW: Storage for timestamp logging
#         self.timestamp_log = []  # Will store (pose_timestamp, arrival_time, echo_time, waiting_time)

        
#         # Storage for bundled data
#         self.latest_force_data = Vector3()
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0

#         # Publisher to forward force echo to force controller
#         self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)

#         # Publishers
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        
#         # Bundled publisher to master
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers with INCREASED queue size to prevent backup
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback, queue_size=1)  # Only process latest
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
#         Echoes back EXACTLY the timestamp from msg.header (master's send time).
#         """
#         # Measure callback timing
#         callback_start = rospy.get_time()
        
#         # Extract master's send timestamp from message header
#         master_send_timestamp = msg.header.stamp.to_sec()
        
#         #  DIAGNOSTIC: Check message age (time between send and receive)
#         message_age = callback_start - master_send_timestamp
#         if message_age > 0.010:  # More than 10ms old
#             rospy.logwarn_throttle(1.0, f"⚠️  OLD MESSAGE! Age: {message_age*1000:.1f}ms - Queue backup detected!")
            
        
#         self.new_pose_available = True
#         self.haptic_timestamps.append(master_send_timestamp)
#         self.last_packet_received_time = callback_start
        
#         # Process haptic data
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

#         # Echo force timestamp if present
#         if msg.force_timestamp > 0:
#             echo_msg = Float64()
#             echo_msg.data = msg.force_timestamp
#             self.force_echo_pub.publish(echo_msg)            
#             rospy.loginfo_throttle(2.0, f"[POSE] Forwarding force_ts echo: {msg.force_timestamp:.6f}")
        
#         # Echo back EXACTLY what master sent (master's send timestamp)
#         #self.pose_timestamp_to_send = master_send_timestamp
            
#         #self.send_bundled_to_master()

#         arrival_time = rospy.get_time()
#         self.pose_ts_queue.append((master_send_timestamp, arrival_time))
        
#         #DIAGNOSTIC: Measure callback duration
#         callback_duration = rospy.get_time() - callback_start
#         rospy.loginfo_throttle(2.0, f"[POSE] Echo sent | msg_age: {message_age*1000:.2f}ms | callback: {callback_duration*1000:.2f}ms")

#     def send_bundled_to_master(self):
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()

#         now = rospy.get_time()
#         pose_ts_to_send = 0.0

        
#         while self.pose_ts_queue:
#             ts, arrival = self.pose_ts_queue[0]
#             if now - arrival > self.MAX_POSE_WAIT:
#                 self.pose_ts_queue.popleft()  
#             else:
#                 break

#         # Pop oldest valid pose timestamp
#         if self.pose_ts_queue:
#             ts, arrival = self.pose_ts_queue.popleft()
#             pose_ts_to_send = ts
            
#             # NEW: Log timestamp information
#             echo_time = now
#             waiting_time = echo_time - arrival
#             self.timestamp_log.append((ts, arrival, echo_time, waiting_time))

#         bundled_msg.pose_timestamp = pose_ts_to_send
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
#         """Velocity control loop - no longer handles pose echo"""
#         if not self.new_pose_available:
#             return
        
#         velocity_pub_msg = Float64MultiArray()
#         velocity_pub_msg.data = self.rpy2joint_space_vel()
#         self.velocity_pub.publish(velocity_pub_msg)
#         self.new_pose_available = False

#         # Log processing delay (optional, for analysis)
#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))

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

#     # NEW: CSV writer for timestamp waiting times
#     def make_csv_timestamp_waiting(self):
#         """Log pose timestamp arrival, echo, and waiting times"""
#         with open('/home/user/Desktop/delay/pose_timestamp_waiting.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Pose_Timestamp', 'Arrival_Time', 'Echo_Time', 'Waiting_Time_seconds', 'Waiting_Time_ms'])

#             for pose_ts, arrival, echo, waiting in self.timestamp_log:
#                 csvwriter.writerow([
#                     pose_ts,
#                     arrival,
#                     echo,
#                     waiting,
#                     waiting * 1000.0  # Convert to milliseconds
#                 ])

#     def main_loop(self):
#         """
#         Main control loop with:
#         - 500 Hz velocity control
#         - 500 Hz regular bundled updates (for force data)
#         - Immediate bundled echo on pose receipt (handled in master_data_callback)
#         """
#         velocity_rate = 500     # Hz (control loop)
#         bundle_rate   = 500     # Hz (regular force updates)

#         # Velocity control loop (500 Hz)
#         rospy.Timer(
#             rospy.Duration(1.0 / velocity_rate),
#             self.velocity_callback
#         )

#         # Regular bundled slave → master transmission (500 Hz)
#         # Mainly for continuous force feedback updates
#         rospy.Timer(
#             rospy.Duration(1.0 / bundle_rate),
#             lambda event: self.send_bundled_to_master()
#         )

#         rospy.loginfo("Control loops started: 500Hz velocity + 500Hz bundled updates + immediate pose echo")
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
#         try:
#             controller.make_csv_timestamp_waiting()
#         except Exception as e:
#             rospy.logerr(f"Failed to save pose_timestamp_waiting.csv: {e}")


#Above code is perfect
#this code is just for logging purpose.


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
# from collections import deque

# class RobotEndEffectorController:

#     def __init__(self):
#         rospy.init_node('robot_end_effector_controller', anonymous=True)
        
#         self.forwarded_time_stamps = []
#         self.last_received_master_timestamp = None
#         self.last_received_master_force_timestamp = None
#         self.start_time = rospy.get_time()
#         self.packet_times = []
#         self.new_pose_available = False
#         self.pose_timestamp_to_send = 0.0

#         self.pose_ts_queue = deque()
#         self.MAX_POSE_WAIT = 1.0 / 500.0

#         # Timestamp logging storage
#         self.timestamp_log = []
        
#         # NEW: Consecutive pose timestamp differences
#         self.pose_timestamp_diffs = []
#         self.previous_pose_timestamp = None
        
#         # NEW: Force timestamp publishing differences
#         self.force_timestamp_publish_log = []
#         self.previous_force_publish_time = None
        
#         # Bundled data storage
#         self.latest_force_data = Vector3()
#         self.latest_force_data.x = 0.0
#         self.latest_force_data.y = 0.0
#         self.latest_force_data.z = 0.0
#         self.latest_pose_timestamp = 0.0
#         self.latest_force_timestamp = 0.0

#         # Publishers
#         self.force_echo_pub = rospy.Publisher('internal_force_timestamp_echo', Float64, queue_size=1)
#         self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
#         self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
#         self.haptic_timestamps = []

#         # Subscribers
#         rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback, queue_size=1)
#         rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
#         rospy.Subscriber('internal_force_data', Vector3, self.internal_force_callback)
#         rospy.Subscriber('internal_force_timestamp', Float64, self.internal_force_timestamp_callback)

#         # Initialization parameters
#         self.robot_pose_flag = True
#         self.haptic_pose_flag = True 
#         self.initial_ee_pose = np.zeros((6, 1))
#         self.current_ee_pose = np.zeros((6, 1))
#         self.joint_position_robot = np.zeros(6)
#         self.haptic_stylus_position = np.zeros((6, 1))
#         self.haptic_stylus_initial_position = np.zeros((6,1))

#         # Unwrapping
#         self.robot_previous_angles = None
#         self.haptic_previous_angles = None

#         # Plotting
#         self.start_time = None
#         self.time_stamps = []
#         self.ee_pose_plot = []
#         self.stylus_pose_plot = []
        
#         rospy.loginfo("Slave pose controller initialized with enhanced timestamp logging")

#     def internal_force_callback(self, msg: Vector3):
#         self.latest_force_data = msg
    
#     def internal_force_timestamp_callback(self, msg: Float64):
#         self.latest_force_timestamp = msg.data

#     def master_data_callback(self, msg: MasterToSlaveData):
#         callback_start = rospy.get_time()
#         master_send_timestamp = msg.header.stamp.to_sec()
        
#         # NEW: Log consecutive pose timestamp differences
#         if self.previous_pose_timestamp is not None:
#             time_diff = master_send_timestamp - self.previous_pose_timestamp
#             self.pose_timestamp_diffs.append((
#                 master_send_timestamp,
#                 self.previous_pose_timestamp,
#                 time_diff
#             ))
#         self.previous_pose_timestamp = master_send_timestamp
        
#         message_age = callback_start - master_send_timestamp
#         if message_age > 0.010:
#             rospy.logwarn_throttle(1.0, f"⚠️  OLD MESSAGE! Age: {message_age*1000:.1f}ms")
        
#         self.new_pose_available = True
#         self.haptic_timestamps.append(master_send_timestamp)
#         self.last_packet_received_time = callback_start
        
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
        
#         arrival_time = rospy.get_time()
#         self.pose_ts_queue.append((master_send_timestamp, arrival_time))
        
#         callback_duration = rospy.get_time() - callback_start
#         rospy.loginfo_throttle(2.0, f"[POSE] msg_age: {message_age*1000:.2f}ms | callback: {callback_duration*1000:.2f}ms")

#     def send_bundled_to_master(self):
#         bundled_msg = SlaveToMasterData()
#         bundled_msg.force = self.latest_force_data
#         bundled_msg.force_timestamp = self.latest_force_timestamp
#         bundled_msg.header.stamp = rospy.Time.now()

#         now = rospy.get_time()
#         pose_ts_to_send = 0.0
        
#         while self.pose_ts_queue:
#             ts, arrival = self.pose_ts_queue[0]
#             if now - arrival > self.MAX_POSE_WAIT:
#                 self.pose_ts_queue.popleft()
#             else:
#                 break

#         if self.pose_ts_queue:
#             ts, arrival = self.pose_ts_queue.popleft()
#             pose_ts_to_send = ts
            
#             echo_time = now
#             waiting_time = echo_time - arrival
#             self.timestamp_log.append((ts, arrival, echo_time, waiting_time))

#         bundled_msg.pose_timestamp = pose_ts_to_send
#         self.slave_to_master_pub.publish(bundled_msg)
        
#         # NEW: Log force timestamp publishing information
#         if self.latest_force_timestamp > 0:
#             publish_time = now
#             time_since_last = 0.0
#             if self.previous_force_publish_time is not None:
#                 time_since_last = publish_time - self.previous_force_publish_time
            
#             self.force_timestamp_publish_log.append((
#                 self.latest_force_timestamp,
#                 publish_time,
#                 time_since_last
#             ))
#             self.previous_force_publish_time = publish_time

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

#         applied_time = rospy.get_time()
#         if hasattr(self, "last_packet_received_time"):
#             diff = applied_time - self.last_packet_received_time
#             self.packet_times.append((
#                 self.last_packet_received_time, 
#                 applied_time, 
#                 diff))

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

#     def make_csv_timestamp_waiting(self):
#         """Log pose timestamp waiting times with missed packet detection"""
        
#         # Main detailed log
#         with open('/home/user/Desktop/delay/pose_timestamp_waiting.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Pose_Timestamp', 'Arrival_Time', 'Echo_Time', 'Waiting_Time_seconds', 
#                               'Waiting_Time_ms', 'Actual_Bundle_Interval_ms', 'Missed_Immediate_Packet'])

#             previous_echo_time = None
#             missed_packet_cases = []  # Store cases where timestamp missed immediate packet
            
#             for pose_ts, arrival, echo, waiting in self.timestamp_log:
#                 # Calculate actual bundle interval (time between consecutive packet sends)
#                 bundle_interval_ms = 0.0
#                 if previous_echo_time is not None:
#                     bundle_interval_ms = (echo - previous_echo_time) * 1000.0
                
#                 # Detect if this timestamp missed its immediate packet
#                 # Threshold: if waiting > 1.5ms, it likely waited for next packet (considering scheduling jitter)
#                 missed_packet = "NO"
#                 if waiting > 0.0018:  # 1.5ms threshold
#                     missed_packet = "YES"
#                     missed_packet_cases.append({
#                         'pose_ts': pose_ts,
#                         'arrival': arrival,
#                         'echo': echo,
#                         'waiting_ms': waiting * 1000.0,
#                         'bundle_interval_ms': bundle_interval_ms
#                     })
                
#                 csvwriter.writerow([
#                     f"{pose_ts:.6f}",
#                     f"{arrival:.6f}",
#                     f"{echo:.6f}",
#                     f"{waiting:.6f}",
#                     f"{waiting * 1000.0:.3f}",
#                     f"{bundle_interval_ms:.3f}",
#                     missed_packet
#                 ])
                
#                 previous_echo_time = echo
        
#         # Separate CSV specifically for missed packet cases (for histogram analysis)
#         with open('/home/user/Desktop/delay/missed_packet_analysis.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Case_Number', 'Pose_Timestamp', 'Arrival_Time', 'Echo_Time', 
#                               'Waiting_Time_ms', 'Bundle_Interval_ms', 'Extra_Wait_Beyond_Expected_ms'])
            
#             for idx, case in enumerate(missed_packet_cases, 1):
#                 # Extra wait = how much longer than expected ~0.5ms typical wait
#                 extra_wait = case['waiting_ms'] - 0.5  # Assuming typical wait ~0.5ms
#                 csvwriter.writerow([
#                     idx,
#                     f"{case['pose_ts']:.6f}",
#                     f"{case['arrival']:.6f}",
#                     f"{case['echo']:.6f}",
#                     f"{case['waiting_ms']:.3f}",
#                     f"{case['bundle_interval_ms']:.3f}",
#                     f"{extra_wait:.3f}"
#                 ])
        
#         if self.timestamp_log:
#             total = len(self.timestamp_log)
#             waiting_times_ms = [w * 1000.0 for _, _, _, w in self.timestamp_log]
            
#             # Calculate statistics
#             avg_wait = np.mean(waiting_times_ms)
#             max_wait = np.max(waiting_times_ms)
#             min_wait = np.min(waiting_times_ms)
#             std_wait = np.std(waiting_times_ms)
            
#             # Missed packet statistics
#             num_missed = len(missed_packet_cases)
#             percentage = (num_missed / total) * 100 if total > 0 else 0
            
#             rospy.loginfo(f"📊 Pose waiting stats: avg={avg_wait:.3f}ms, max={max_wait:.3f}ms, "
#                          f"min={min_wait:.3f}ms, std={std_wait:.3f}ms")
#             rospy.loginfo(f"📊 Missed immediate packet: {num_missed}/{total} ({percentage:.1f}%)")
            
#             if missed_packet_cases:
#                 missed_waits = [c['waiting_ms'] for c in missed_packet_cases]
#                 rospy.loginfo(f"📊 Missed packet wait times: avg={np.mean(missed_waits):.3f}ms, "
#                              f"max={np.max(missed_waits):.3f}ms, min={np.min(missed_waits):.3f}ms")

#     def make_csv_pose_timestamp_diffs(self):
#         """NEW: Log consecutive pose timestamp differences (master sampling rate analysis)"""
#         with open('/home/user/Desktop/delay/pose_timestamp_differences.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Current_Pose_Timestamp', 'Previous_Pose_Timestamp', 'Time_Difference_seconds', 
#                               'Time_Difference_ms', 'Sampling_Rate_Hz'])

#             for current_ts, previous_ts, time_diff in self.pose_timestamp_diffs:
#                 sampling_rate = 1.0 / time_diff if time_diff > 0 else 0.0
#                 csvwriter.writerow([
#                     f"{current_ts:.6f}",
#                     f"{previous_ts:.6f}",
#                     f"{time_diff:.6f}",
#                     f"{time_diff * 1000.0:.3f}",
#                     f"{sampling_rate:.2f}"
#                 ])
        
#         if self.pose_timestamp_diffs:
#             avg_diff = np.mean([td for _, _, td in self.pose_timestamp_diffs])
#             avg_rate = 1.0 / avg_diff if avg_diff > 0 else 0.0
#             rospy.loginfo(f"📊 Master pose sampling: avg={avg_diff*1000:.3f}ms, rate={avg_rate:.1f}Hz")

#     def make_csv_force_timestamp_publish(self):
#         """NEW: Log force timestamp publishing intervals"""
#         with open('/home/user/Desktop/delay/force_timestamp_publish_intervals.csv', 'w', newline='') as csvfile:
#             csvwriter = csv.writer(csvfile)
#             csvwriter.writerow(['Force_Timestamp', 'Publish_Time', 'Time_Since_Last_Publish_seconds', 
#                               'Time_Since_Last_Publish_ms', 'Publishing_Rate_Hz'])

#             for force_ts, publish_time, time_since_last in self.force_timestamp_publish_log:
#                 publish_rate = 1.0 / time_since_last if time_since_last > 0 else 0.0
#                 csvwriter.writerow([
#                     f"{force_ts:.6f}",
#                     f"{publish_time:.6f}",
#                     f"{time_since_last:.6f}",
#                     f"{time_since_last * 1000.0:.3f}",
#                     f"{publish_rate:.2f}"
#                 ])
        
#         if len(self.force_timestamp_publish_log) > 1:
#             intervals = [tsl for _, _, tsl in self.force_timestamp_publish_log if tsl > 0]
#             if intervals:
#                 avg_interval = np.mean(intervals)
#                 avg_rate = 1.0 / avg_interval if avg_interval > 0 else 0.0
#                 rospy.loginfo(f"📊 Force publish: avg={avg_interval*1000:.3f}ms, rate={avg_rate:.1f}Hz")

#     def main_loop(self):
#         velocity_rate = 500
#         bundle_rate = 500

#         rospy.Timer(rospy.Duration(1.0 / velocity_rate), self.velocity_callback)
#         rospy.Timer(rospy.Duration(1.0 / bundle_rate), lambda event: self.send_bundled_to_master())

#         rospy.loginfo("Control loops: 500Hz velocity + 500Hz bundled + immediate pose echo")
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
#             rospy.loginfo("✅ Saved pose.csv")
#         except Exception as e:
#             rospy.logerr(f"❌ Failed to save pose.csv: {e}")
#         try:
#             controller.make_csv_packet_times()
#             rospy.loginfo("✅ Saved pose_processing.csv")
#         except Exception as e:
#             rospy.logerr(f"❌ Failed to save pose_processing.csv: {e}")
#         try:
#             controller.make_csv_timestamp_waiting()
#             rospy.loginfo("✅ Saved pose_timestamp_waiting.csv")
#         except Exception as e:
#             rospy.logerr(f"❌ Failed to save pose_timestamp_waiting.csv: {e}")
#         try:
#             controller.make_csv_pose_timestamp_diffs()
#             rospy.loginfo("✅ Saved pose_timestamp_differences.csv")
#         except Exception as e:
#             rospy.logerr(f"❌ Failed to save pose_timestamp_differences.csv: {e}")
#         try:
#             controller.make_csv_force_timestamp_publish()
#             rospy.loginfo("✅ Saved force_timestamp_publish_intervals.csv")
#         except Exception as e:
#             rospy.logerr(f"❌ Failed to save force_timestamp_publish_intervals.csv: {e}")