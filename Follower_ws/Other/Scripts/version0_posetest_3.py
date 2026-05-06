#!/usr/bin/env python3


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
        self.pose_ts_buffer = []   # stores all pose timestamps since last echo


        
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
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)#
        
        # Bundled publisher to master
        self.slave_to_master_pub = rospy.Publisher('slave_to_master_data', SlaveToMasterData, queue_size=1)
        
        self.haptic_timestamps = []

        # Subscribers with INCREASED queue size to prevent backup
        rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback, queue_size=1)  # TCP:Only process latest
        # rospy.Subscriber('master_to_slave_data', MasterToSlaveData, self.master_data_callback,
        #                  queue_size=1,
        #                  transport_hints=rospy.TransportHints().udp())
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
        self.pose_ts_buffer.append(master_send_timestamp)
        
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

    # Send ONLY the latest pose timestamp (if available)
        if self.pose_ts_buffer:
            latest_ts = self.pose_ts_buffer[-1]      # take newest
            bundled_msg.pose_timestamps = [latest_ts]
            self.pose_ts_buffer.clear()               # clear buffer after sending
        else:
            bundled_msg.pose_timestamps = []

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

