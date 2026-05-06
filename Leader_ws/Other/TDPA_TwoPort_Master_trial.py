#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from omni_msgs.msg import OmniFeedback
from scipy.spatial.transform import Rotation as R

class TDPAmaster:

    def __init__(self):
        
        # Initialize node
        rospy.init_node('Master', anonymous=True)

        # Initialize flags and variables
        self.haptic_pose_flag = True
        self.haptic_previous_angles = None
        self.haptic_stylus_position = np.zeros((6, 1))
        self.haptic_stylus_initial_position = np.zeros((6, 1))
        self.fmd = np.zeros((3, 1))
        self.Es_in_delayed = np.zeros((3, 1))
        self.Em_out = np.zeros((3, 1))
        self.Em_in = np.zeros((3, 1))

        self.previous_haptic_stylus_position = np.zeros((6, 1))
        self.previous_time = rospy.get_time()

        # Make publishers

        # velocity published to slave (= vm)
        self.velocity_publisher = rospy.Publisher("velocity_from_master", Float64MultiArray, queue_size=10)

        # energy published to slave (= Em_in)
        self.energy_m_in_publisher = rospy.Publisher("energy_from_master", Float64MultiArray, queue_size=10)

        # force published to haptic device
        self.force_publisher = rospy.Publisher("/phantom/phantom/force_feedback", OmniFeedback, queue_size=10)

        # force published to haptic device
        self.pose_publisher = rospy.Publisher("pose_from_master", Float64MultiArray, queue_size=10)

        # Make subscribers

        # subscribed pose from haptic device
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.haptic_pose_callback)

        # subscribed force from slave (= fmd)
        rospy.Subscriber('force_from_slave', Float64MultiArray, self.force_slave_callback)

        # subscribed energy from slave (= Es_in(k - Dsm(k)))
        rospy.Subscriber('energy_from_slave', Float64MultiArray, self.energy_slave_callback)

    def haptic_pose_callback(self, msg: PoseStamped):
        euler_from_haptic = self.haptic_quat2rpy([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        if self.haptic_previous_angles is not None:
            euler_from_haptic = self.unwrap_angle(euler_from_haptic, self.haptic_previous_angles)
        self.haptic_previous_angles = euler_from_haptic
        haptic_stylus_position = np.array([[msg.pose.position.x],[msg.pose.position.y],[msg.pose.position.z],[euler_from_haptic[0]],[euler_from_haptic[1]],[euler_from_haptic[2]]])
        if self.haptic_pose_flag:
            self.haptic_stylus_initial_position = haptic_stylus_position
            self.haptic_pose_flag = False
        self.haptic_stylus_position = haptic_stylus_position - self.haptic_stylus_initial_position
        # print(f"self.haptic_stylus_position: {self.haptic_stylus_position}, shape: {self.haptic_stylus_position.shape}")
        return self.haptic_stylus_position #shape (6,1)

    def force_slave_callback(self, msg: Float64MultiArray):
        self.fmd = np.array(msg.data).reshape(3, 1)
        #print(f"self.fmd: {self.fmd.flatten()}")

    def energy_slave_callback(self, msg: Float64MultiArray):
        self.Es_in_delayed = np.array(msg.data).reshape(3, 1)
        #print(f"self.Es_in_delayed: {self.Es_in_delayed}, shape: {self.Es_in_delayed.shape}")

    @staticmethod
    def haptic_quat2rpy(quaternion):
        rotation = R.from_quat(quaternion)
        resulting_matrix = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ rotation.as_dcm()
        resulting_matrix[0, 1] = -resulting_matrix[0, 1]
        resulting_matrix[0, 2] = -resulting_matrix[0, 2]
        resulting_matrix[1, 0] = -resulting_matrix[1, 0]
        resulting_matrix[2, 0] = -resulting_matrix[2, 0]
        euler_rpy= R.from_dcm(resulting_matrix).as_euler('xyz', degrees=False)
        return euler_rpy

    @staticmethod
    def unwrap_angle(angle, previous_angle):
        if previous_angle is None:
            return angle
        delta = angle - previous_angle
        delta[0] = (delta[0] + np.pi) % (2 * np.pi) - np.pi
        delta[1] = (delta[1] + np.pi/2) % np.pi - np.pi/2
        delta[2] = (delta[2] + np.pi) % (2 * np.pi) - np.pi
        return previous_angle + delta

    def velocity_fn(self, haptic_stylus_position, current_time):
        haptic_displacement = haptic_stylus_position - self.previous_haptic_stylus_position
        delta_time = current_time - self.previous_time
        self.previous_time = current_time
        self.previous_haptic_stylus_position = haptic_stylus_position
        haptic_velocity = haptic_displacement/delta_time
        #print(f"velocity: {haptic_velocity.flatten()}")
        #print(f"delta_time: {delta_time}")
        return haptic_velocity, delta_time

    def publisher_callback(self, event):
        fm = np.zeros(3)
        velocity, delta_time = self.velocity_fn(self.haptic_stylus_position, rospy.get_time())
        for i in range(3):
            vm = velocity[i, 0]
            fmd = self.fmd[i, 0]
            P = vm * fmd

            if P < 0:
                self.Em_out[i, 0] -= P*delta_time

            if P > 0:
                self.Em_in[i, 0] += P*delta_time
            
            if self.Em_out[i, 0] > self.Es_in_delayed[i, 0] and vm != 0:
                alpha = (self.Em_out[i, 0] - self.Es_in_delayed[i, 0]) / ((vm ** 2)*delta_time)
            elif self.Em_out[i, 0] <= self.Es_in_delayed[i, 0]:
                alpha = 0

            fm[i] = fmd + alpha * vm

        print(f"self.fm: {fm.flatten()}")

        pose_msg = Float64MultiArray(data=self.haptic_stylus_position.flatten().tolist())
        velocity_msg = Float64MultiArray(data=self.velocity.flatten().tolist())
        energy_msg = Float64MultiArray(data=self.Em_in.flatten().tolist())
        force_msg = OmniFeedback()
        force_msg.force.x, force_msg.force.y, force_msg.force.z = fm

        # print(f"velocity_msg: {velocity_msg}, energy_msg: {energy_msg}, force_msg: {force_msg}")
        #print(f"energy_msg: {energy_msg}")
        
        
        self.pose_publisher.publish(pose_msg)
        self.velocity_publisher.publish(velocity_msg)
        self.energy_m_in_publisher.publish(energy_msg)
        self.force_publisher.publish(force_msg)

        print(f"velocity_msg: {velocity_msg.data}")

    def main_loop(self):
        rate = 1000
        rospy.Timer(rospy.Duration(1.0 / rate), self.publisher_callback)
        rospy.spin()

if __name__ == "__main__":
    try:
        node = TDPAmaster()
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
