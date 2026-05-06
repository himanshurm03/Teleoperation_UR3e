#!/usr/bin/env python3
import rospy
import time
import numpy as np
from omni_msgs.msg import OmniFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64 
from collections import deque

class MasterController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('master', anonymous=True)

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
        self.pose_pub = rospy.Publisher('pose_from_master_to_slave', PoseStamped, queue_size=1)
        self.time_pub = rospy.Publisher('time_from_master_to_slave', Float64, queue_size=1)

        # Subscribers
        rospy.Subscriber('force_from_slave_to_master', OmniFeedback, self.force_from_slave_to_master_callback)
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_from_master_to_slave_callback)
        rospy.Subscriber('time_from_slave_to_master', Float64, self.time_from_slave_to_master_callback)

        # Shutdown management
        self.shutdown_flag = False
        rospy.on_shutdown(self.shutdown_hook)

        self.velocity = np.zeros((3, 1)) 
        self.previous_time_of_msg = None
        self.current_time_of_msg = rospy.get_time()
        self.energy_without_tdpa = np.zeros((3, 1))
        self.energy_obs = np.zeros((3, 1))
        self.alpha_previous = np.zeros((3, 1))
        self.previous_haptic_stylus_velocity = np.zeros((3, 1))
        self.force_history = deque(maxlen=100)
        self.actual_force = np.zeros((3, 1))

    def time_from_slave_to_master_callback(self, msg: Float64):
        delay = (rospy.get_time() - msg.data) / 2
        # rospy.loginfo(f"Delay for force: {delay}")

    def force_from_slave_to_master_callback(self, msg: OmniFeedback):
        if self.shutdown_flag:
            return
        force = np.array([[msg.force.x], [msg.force.y], [msg.force.z]])
        force = self.TDPA_twoport(force, self.velocity)
        feedback_msg = OmniFeedback()
        feedback_msg.force.x = force[0, 0]
        feedback_msg.force.y = force[1, 0]
        feedback_msg.force.z = force[2, 0]
        self.force_pub.publish(feedback_msg)
        time_msg = Float64()
        time_msg.data = rospy.get_time()
        self.time_pub.publish(time_msg)

    def TDPA_twoport(self, force, velocity):
        if self.previous_time_of_msg is None:
            self.previous_time_of_msg = self.current_time_of_msg
            return np.zeros((3, 1))
        self.current_time_of_msg = rospy.get_time()
        alpha = np.zeros((3, 1))
        velocity = -velocity
        delta_t = (self.current_time_of_msg - self.previous_time_of_msg)
        self.energy_without_tdpa = self.energy_without_tdpa + np.multiply(force, velocity) * delta_t
        self.energy_obs = self.energy_obs + (np.multiply(force, velocity) + np.multiply(self.alpha_previous, (self.previous_haptic_stylus_velocity ** 2))) * delta_t
        for i in range(3):
            if self.energy_obs[i, 0] < 0:
                alpha[i, 0] = -self.energy_obs[i, 0] / (delta_t * (velocity[i, 0] ** 2) + 0.0001)
                alpha[i, 0] = min(alpha[i, 0], 100000)
            else:
                alpha[i, 0] = 0.0
        rospy.loginfo(f"Value of alpha: {alpha}")
        self.previous_time_of_msg = self.current_time_of_msg
        self.alpha_previous = alpha
        self.previous_haptic_stylus_velocity = velocity
        total_force = force + np.multiply(alpha, velocity)
        self.force_history.append(total_force)
        smoothed_force = np.mean(self.force_history, axis=0)
        self.actual_force = force
        return smoothed_force

    def pose_from_master_to_slave_callback(self, msg: PoseStamped):
        self.pose_pub.publish(msg)
        self.velocity = 0.001*np.array([[msg.pose.position.x],[msg.pose.position.y],[msg.pose.position.z]])

    def shutdown_hook(self):
        self.shutdown_flag = True
        zero_force_msg = OmniFeedback()
        zero_force_msg.force.x = 0
        zero_force_msg.force.y = 0
        zero_force_msg.force.z = 0

        rospy.loginfo("Sending zero force to haptic device...")
        for _ in range(10):
            self.force_pub.publish(zero_force_msg)
            time.sleep(0.1)
        rospy.loginfo("Shutdown complete.")

    def main_loop(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = MasterController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
