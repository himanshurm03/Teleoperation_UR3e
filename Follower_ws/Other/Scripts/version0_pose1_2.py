#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
import csv

class RobotEndEffectorController:

    def __init__(self):
        # Initializing node
        rospy.init_node('robot_end_effector_controller', anonymous=True)
        self.start_time = rospy.get_time()

        # Open CSV file
        self.csv_file = open("/home/user/Desktop/delay/velocities_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "v1", "v2", "v3", "v4", "v5", "v6"])  # header row #rad/sec
        self.forwarded_time_stamps = []
        self.last_received_master_timestamp = None
        self.last_received_master_force_timestamp = None
        self.start_time = rospy.get_time()
        self.packet_times = [] 

        # CSV for actual EE velocities
        self.ee_velocity_csv_file = open("/home/user/Desktop/delay/ee_velocities.csv", "w", newline="") 
        self.ee_velocity_csv_writer = csv.writer(self.ee_velocity_csv_file)
        self.ee_velocity_csv_writer.writerow(["time", "vx", "vy", "vz", "v_roll", "v_pitch", "v_yaw"])




        # Publishers
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)
        self.time_back_to_master_pose_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=1)
        self.time_back_to_master_force_pub = rospy.Publisher('time_from_slave_to_master_for_force', Float64, queue_size=1)
        self.haptic_timestamps = []
        #self.time_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=1, tcp_nodelay=True)
        self.time_pub = rospy.Publisher('time_from_slave_to_master_for_pose', Float64, queue_size=10)

        


        # Subscribers
        rospy.Subscriber('pose_from_master_to_slave', PoseStamped, self.haptic_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback_robot)
        rospy.Subscriber('time_from_master_to_slave_for_pose', Float64, self.time_from_master_to_slave_for_pose_callback)
        rospy.Subscriber('time_from_master_to_slave_for_force', Float64, self.time_from_master_to_slave_for_force_callback)





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

    @staticmethod
    def unwrap_angle(angle, previous_angle):
        if previous_angle is None:
            return angle
        delta = angle - previous_angle
        delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
        delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
        delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
        return previous_angle + delta
    
    def time_from_master_to_slave_for_force_callback(self, msg):
        self.last_received_master_force_timestamp = msg.data

    
    # def time_from_master_to_slave_for_pose_callback(self, msg: Float64):
    #     self.time_pub.publish(msg)
    #     self.forwarded_time_stamps.append(msg.data)
    
    def time_from_master_to_slave_for_pose_callback(self, msg):
        #self.last_received_master_timestamp = msg.data
        #self.time_back_to_master_pose_pub.publish(msg)
        self.time_pub.publish(msg.data)  # This sends the same timestamp back to the master:here msg has been changed to msg.data
        self.forwarded_time_stamps.append(msg.data)

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

        c = 8  #Original c ws 8
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

    # def joint_callback_robot(self, msg: JointState):
    #     self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
    #     self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
    #     if self.robot_pose_flag:
    #         self.initial_ee_pose = self.current_ee_pose
    #         self.robot_pose_flag = False

    def joint_callback_robot(self, msg: JointState):
    # Joint positions (already exists)
        self.joint_position_robot = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4], msg.position[5]])
        self.current_ee_pose = self.pose_and_rotation(self.joint_position_robot)
    
    # --- Log initial EE pose ---
        if self.robot_pose_flag:
            self.initial_ee_pose = self.current_ee_pose
            self.robot_pose_flag = False

    # --- Compute EE velocity from joint velocities ---
        joint_velocities = np.array([msg.velocity[2], msg.velocity[1], msg.velocity[0], msg.velocity[3], msg.velocity[4], msg.velocity[5]])
        J_geo = self.Jointspace2GeometricJacobian(self.joint_position_robot)
        J_geo2ana = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])  # optional if you want geometric to analytical conversion

        ee_velocity = J_geo2ana @ J_geo @ joint_velocities  # 6x1 end-effector velocity
        current_time = rospy.get_time() - self.start_time
        self.ee_velocity_csv_writer.writerow([current_time] + list(ee_velocity))
        self.ee_velocity_csv_file.flush()


    #def haptic_callback(self, msg: PoseStamped):
    # Store header timestamp from incoming haptic pose
    #     self.haptic_timestamps.append(msg.header.stamp.to_sec())

    #     euler_from_haptic = self.haptic_quat2rpy([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    #     if self.haptic_previous_angles is not None:
    #         euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
    #     self.haptic_previous_angles = euler_from_haptic

    #     haptic_stylus_position = np.array([[msg.pose.position.x],[msg.pose.position.y],[msg.pose.position.z],
    #                                    [euler_from_haptic[0]],[euler_from_haptic[1]],[euler_from_haptic[2]]])
    
    #     if self.haptic_pose_flag:
    #         self.haptic_stylus_initial_position = haptic_stylus_position
    #         self.haptic_pose_flag = False

    #     self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

    # # --- Send back the master timestamp for delay calculation ---
    #     if self.last_received_master_timestamp is not None:
    #         self.time_pub.publish(Float64(self.last_received_master_timestamp))

    #     return self.haptic_stylus_position
 
##THIS IS THE ORIGINAL HAPTIC CALLBACK FUNCTION
 
    def haptic_callback(self, msg: PoseStamped):
    # --- Store and forward header timestamp from master ---
        timestamp = msg.header.stamp.to_sec()
        self.haptic_timestamps.append(timestamp)

# New line
        self.last_packet_received_time = rospy.get_time()
    # Publish the same timestamp back to master for delay logging
        self.time_pub.publish(Float64(timestamp))

    # --- Pose and orientation processing ---
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

    # --- Apply initial offset if first message ---
        if self.haptic_pose_flag:
            self.haptic_stylus_initial_position = haptic_stylus_position
            self.haptic_pose_flag = False

        self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position

        return self.haptic_stylus_position
## ABOVE IS THE ORIGINAL HAPTIC CALL BACK FUNCTION
 


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

    # def velocity_callback(self, event):
    #     velocity_pub_msg = Float64MultiArray()
    #     velocity_pub_msg.data = self.rpy2joint_space_vel()
    #     applied_time = rospy.get_time()
    #     if hasattr(self, "last_packet_received_time"):
    #         diff = applied_time - self.last_packet_received_time
    #         self.packet_times.append((
    #             self.last_packet_received_time, 
    #             applied_time, 
    #             diff))
    #     self.velocity_pub.publish(velocity_pub_msg)

    def velocity_callback(self, event):
        velocity = self.rpy2joint_space_vel()   # numpy array of 6 values
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = velocity

        applied_time = rospy.get_time() - self.start_time 
        if hasattr(self, "last_packet_received_time"):
            diff = applied_time - self.last_packet_received_time
            self.packet_times.append((
                self.last_packet_received_time, 
                applied_time, 
                diff))

        # Publish to robot
        self.velocity_pub.publish(velocity_pub_msg)

        # --- Log to CSV ---
        self.csv_writer.writerow([applied_time] + list(velocity))
        self.csv_file.flush()  # ensures data is written immediately



    def __del__(self):
        # Close file when object is destroyed
        if hasattr(self, "csv_file"):
            self.csv_file.close()
        if hasattr(self, "ee_velocity_csv_file"):
            self.ee_velocity_csv_file.close()

    def plot_error_data(self):

        times = np.array(self.time_stamps)
        ee_pose_plot = np.array(self.ee_pose_plot)
        stylus_pose_plot = np.array(self.stylus_pose_plot)

        fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
        axs[0, 0].plot(times, stylus_pose_plot[:, 0] - ee_pose_plot[:, 0])
        axs[0, 0].set_title('Error in x')
        axs[0, 0].set_ylabel('Meters')
        axs[0, 0].set_xlabel('Seconds')
        axs[0, 0].grid(True)

        axs[1, 0].plot(times, stylus_pose_plot[:, 1] - ee_pose_plot[:, 1])
        axs[1, 0].set_title('Error in y')
        axs[1, 0].set_ylabel('Meters')
        axs[1, 0].set_xlabel('Seconds')
        axs[1, 0].grid(True)

        axs[2, 0].plot(times, stylus_pose_plot[:, 2] - ee_pose_plot[:, 2])
        axs[2, 0].set_title('Error in z')
        axs[2, 0].set_ylabel('Meters')
        axs[2, 0].set_xlabel('Seconds')
        axs[2, 0].grid(True)

        axs[0, 1].plot(times, stylus_pose_plot[:, 3] - ee_pose_plot[:, 3])
        axs[0, 1].set_title('Error in roll')
        axs[0, 1].set_ylabel('Radians')
        axs[0, 1].set_xlabel('Seconds')
        axs[0, 1].grid(True)

        axs[1, 1].plot(times, stylus_pose_plot[:, 4] - ee_pose_plot[:, 4])
        axs[1, 1].set_title('Error in pitch')
        axs[1, 1].set_ylabel('Radians')
        axs[1, 1].set_xlabel('Seconds')
        axs[1, 1].grid(True)

        axs[2, 1].plot(times, stylus_pose_plot[:, 5] - ee_pose_plot[:, 5])
        axs[2, 1].set_title('Error in yaw')
        axs[2, 1].set_ylabel('Radians')
        axs[2, 1].set_xlabel('Seconds')
        axs[2, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_data(self):

        times = np.array(self.time_stamps)
        ee_pose_plot = np.array(self.ee_pose_plot)
        stylus_pose_plot = np.array(self.stylus_pose_plot)

        fig, axs = plt.subplots(3, 2, figsize=(16, 9))
        
        axs[0, 0].plot(times, stylus_pose_plot[:, 0], label='haptic')
        axs[0, 0].plot(times, ee_pose_plot[:, 0], label='robot')
        axs[0, 0].set_title('x')
        axs[0, 0].set_ylabel('Meters')
        axs[0, 0].set_xlabel('Seconds')
        axs[0, 0].grid(True)
        axs[0, 0].legend()

        axs[1, 0].plot(times, stylus_pose_plot[:, 1], label='haptic')
        axs[1, 0].plot(times, ee_pose_plot[:, 1], label='robot')
        axs[1, 0].set_title('y')
        axs[1, 0].set_ylabel('Meters')
        axs[1, 0].set_xlabel('Seconds')
        axs[1, 0].grid(True)
        axs[1, 0].legend()

        axs[2, 0].plot(times, stylus_pose_plot[:, 2], label='haptic')
        axs[2, 0].plot(times, ee_pose_plot[:, 2], label='robot')
        axs[2, 0].set_title('z')
        axs[2, 0].set_ylabel('Meters')
        axs[2, 0].set_xlabel('Seconds')
        axs[2, 0].grid(True)
        axs[2, 0].legend()

        axs[0, 1].plot(times, stylus_pose_plot[:, 3], label='haptic')
        axs[0, 1].plot(times, ee_pose_plot[:, 3], label='robot')
        axs[0, 1].set_title('roll')
        axs[0, 1].set_ylabel('Radians')
        axs[0, 1].set_xlabel('Seconds')
        axs[0, 1].grid(True)
        axs[0, 1].legend()

        axs[1, 1].plot(times, stylus_pose_plot[:, 4], label='haptic')
        axs[1, 1].plot(times, ee_pose_plot[:, 4], label='robot')
        axs[1, 1].set_title('pitch')
        axs[1, 1].set_ylabel('Radians')
        axs[1, 1].set_xlabel('Seconds')
        axs[1, 1].grid(True)
        axs[1, 1].legend()

        axs[2, 1].plot(times, stylus_pose_plot[:, 5], label='haptic')
        axs[2, 1].plot(times, ee_pose_plot[:, 5], label='robot')
        axs[2, 1].set_title('yaw')
        axs[2, 1].set_ylabel('Radians')
        axs[2, 1].set_xlabel('Seconds')
        axs[2, 1].grid(True)
        axs[2, 1].legend()

        plt.tight_layout()
        plt.show()

    def plot_planes(self):

        ee_pose_plot = np.array(self.ee_pose_plot)
        stylus_pose_plot = np.array(self.stylus_pose_plot)

        fig, axs = plt.subplots(1, 3, figsize=(19, 4.5))
        
        # Plot the xy plane
        axs[0].plot(ee_pose_plot[:, 0], ee_pose_plot[:, 1], label='End-Effector')
        axs[0].plot(stylus_pose_plot[:, 0], stylus_pose_plot[:, 1], label='Stylus')
        axs[0].set_title('XY Plane')
        axs[0].set_xlabel('X')
        axs[0].set_ylabel('Y')
        axs[0].axis('equal')
        axs[0].grid(True)
        axs[0].legend()

        # Plot the yz plane
        axs[1].plot(ee_pose_plot[:, 1], ee_pose_plot[:, 2], label='End-Effector')
        axs[1].plot(stylus_pose_plot[:, 1], stylus_pose_plot[:, 2], label='Stylus')
        axs[1].set_title('YZ Plane')
        axs[1].set_xlabel('Y')
        axs[1].set_ylabel('Z')
        axs[1].axis('equal')
        axs[1].grid(True)
        axs[1].legend()

        # Plot the zx plane
        axs[2].plot(ee_pose_plot[:, 2], ee_pose_plot[:, 0], label='End-Effector')
        axs[2].plot(stylus_pose_plot[:, 2], stylus_pose_plot[:, 0], label='Stylus')
        axs[2].set_title('ZX Plane')
        axs[2].set_xlabel('Z')
        axs[2].set_ylabel('X')
        axs[2].axis('equal')
        axs[2].grid(True)
        axs[2].legend()

        plt.tight_layout()
        plt.show()

    def plot_all_data(self):
        times = np.array(self.time_stamps)
        ee_pose_plot = np.array(self.ee_pose_plot)
        stylus_pose_plot = np.array(self.stylus_pose_plot)

        fig, axs = plt.subplots(6, 2, figsize=(20, 24))

        # Plot error data
        errors = ['Error in x', 'Error in y', 'Error in z', 'Error in roll', 'Error in pitch', 'Error in yaw']
        units = ['Meters', 'Meters', 'Meters', 'Radians', 'Radians', 'Radians']
        for i in range(6):
            axs[i, 0].plot(times, stylus_pose_plot[:, i] - ee_pose_plot[:, i])
            axs[i, 0].set_title(errors[i])
            axs[i, 0].set_ylabel(units[i])
            axs[i, 0].set_xlabel('Seconds')
            axs[i, 0].grid(True)

        # Plot pose data
        poses = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for i in range(6):
            axs[i, 1].plot(times, stylus_pose_plot[:, i], label='haptic')
            axs[i, 1].plot(times, ee_pose_plot[:, i], label='robot')
            axs[i, 1].set_title(poses[i])
            axs[i, 1].set_ylabel(units[i])
            axs[i, 1].set_xlabel('Seconds')
            axs[i, 1].grid(True)
            axs[i, 1].legend()

        plt.tight_layout()
        plt.show()

        # Plot planes
        fig, axs = plt.subplots(1, 3, figsize=(19, 4.5))
        
        # Plot the xy plane
        axs[0].plot(ee_pose_plot[:, 0], ee_pose_plot[:, 1], label='End-Effector')
        axs[0].plot(stylus_pose_plot[:, 0], stylus_pose_plot[:, 1], label='Stylus')
        axs[0].set_title('XY Plane')
        axs[0].set_xlabel('X')
        axs[0].set_ylabel('Y')
        axs[0].axis('equal')
        axs[0].grid(True)
        axs[0].legend()

        # Plot the yz plane
        axs[1].plot(ee_pose_plot[:, 1], ee_pose_plot[:, 2], label='End-Effector')
        axs[1].plot(stylus_pose_plot[:, 1], stylus_pose_plot[:, 2], label='Stylus')
        axs[1].set_title('YZ Plane')
        axs[1].set_xlabel('Y')
        axs[1].set_ylabel('Z')
        axs[1].axis('equal')
        axs[1].grid(True)
        axs[1].legend()

        # Plot the zx plane
        axs[2].plot(ee_pose_plot[:, 2], ee_pose_plot[:, 0], label='End-Effector')
        axs[2].plot(stylus_pose_plot[:, 2], stylus_pose_plot[:, 0], label='Stylus')
        axs[2].set_title('ZX Plane')
        axs[2].set_xlabel('Z')
        axs[2].set_ylabel('X')
        axs[2].axis('equal')
        axs[2].grid(True)
        axs[2].legend()

        plt.tight_layout()
        plt.show()


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
                current_time = self.start_time + times[i]  # Just the wall-clock time at which this was recorded

                csvwriter.writerow([
                    times[i],                     
                    self.time_stamps[i],                  
                    ee_pose_plot[i][0], ee_pose_plot[i][1], ee_pose_plot[i][2],
                    ee_pose_plot[i][3], ee_pose_plot[i][4], ee_pose_plot[i][5],
                    stylus_pose_plot[i][0], stylus_pose_plot[i][1], stylus_pose_plot[i][2],
                    stylus_pose_plot[i][3], stylus_pose_plot[i][4], stylus_pose_plot[i][5]
                ])


    # def make_csv_packet_times(self):
    # # Convert to numpy for easier handling
    #     packet_times = np.array(self.packet_times)  # list of (received, applied, diff)

    #     with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
    #         csvwriter = csv.writer(csvfile)
    #         csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

    #         for i in range(len(packet_times)):
    #             csvwriter.writerow([
    #                 packet_times[i][0],   # Received_Time
    #                 packet_times[i][1],   # Applied_Time
    #                 packet_times[i][2]    # Difference
    #             ])
    
    def make_csv_packet_times(self):
        with open('/home/user/Desktop/delay/pose_processing.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['Received_Time', 'Applied_Time', 'Difference'])

            for received, applied, diff in self.packet_times:
                csvwriter.writerow([received, applied, diff])

    def main_loop(self):
        rate = 500
        rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
        rospy.spin()

# if __name__ == "__main__":
#     try:
#         controller = RobotEndEffectorController()
#         controller.main_loop()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         controller.make_csv()
#         controller.make_csv_packet_times()

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

        #controller.plot_error_data()
        #controller.plot_data()
        #controller.plot_planes()
        #controller.plot_all_data()