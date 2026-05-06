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
import time
from collections import deque
from omni_msgs.msg import OmniButtonEvent

class RobotEndEffectorController:

    def __init__(self):

        # Initialising node
        rospy.init_node('robot_end_effector_controller', anonymous=True)

        # Publisher that publish joint angle velocity to the manipulator
        self.velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

        # Provides joint angles of haptic device
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.haptic_jointstate_callback)

        # Provides joint angles of the manipulator
        rospy.Subscriber('/joint_states', JointState, self.robot_jointstate_callback)

        # Provides button reponse of haptic device
        rospy.Subscriber('/phantom/phantom/button', OmniButtonEvent, self.button_callback)

        # Record the start time of the experiment
        self.start_time = time.time()

        # Initialising variables
        self.robot_jointstate = np.zeros((6,1))
        self.intial_robot_jointstate = np.zeros((6,1))
        self.inital_robot_jointstate_flag = True
        self.haptic_jointstate = np.zeros((6, 1))
        self.intial_haptic_jointstate = np.zeros((6,1))
        self.inital_haptic_jointstate_flag = True
        self.constant = 1

        # For moving average filter
        self.window_size = 100  # Adjust the window size for smoothing
        self.velocity_window = deque(maxlen=self.window_size)
        
        # Lists to store data for plotting
        self.time_stamps = []
        self.jointspace_velocity_plot = []
        self.robot_jointstate_plot = []
        self.haptic_jointstate_plot = []

        # Running flag
        self.running = True
        
    def jointspace_velocity_calculator(self):

        # c1 = 1 #5
        # c2 = 1
        # c3 = 1
        # c4 = 1
        # c5 = 1
        # c6 = 1
        # k = np.array([[c1, 0, 0, 0, 0, 0],
        #               [0, c2, 0, 0, 0, 0],
        #               [0, 0, c3, 0, 0, 0],
        #               [0, 0, 0, c4, 0, 0],
        #               [0, 0, 0, 0, c5, 0],
        #               [0, 0, 0, 0, 0, c6]])
        
        c = 2
        k = np.array([[c, 0, 0, 0, 0, 0],
                      [0, c, 0, 0, 0, 0],
                      [0, 0, c, 0, 0, 0],
                      [0, 0, 0, c, 0, 0],
                      [0, 0, 0, 0, c, 0],
                      [0, 0, 0, 0, 0, 0]])
        
        c1 = self.constant
        k1 = np.array([[c1, 0, 0, 0, 0, 0],
                      [0, c1, 0, 0, 0, 0],
                      [0, 0, c1, 0, 0, 0],
                      [0, 0, 0, c1, 0, 0],
                      [0, 0, 0, 0, c1, 0],
                      [0, 0, 0, 0, 0, c1]])
        
        jointspace_velocity = k@(self.haptic_jointstate - self.robot_jointstate)
        # print("jointspace_velocity:",jointspace_velocity)
        #print("haptic joint space",self.haptic_jointstate+self.intial_robot_jointstate)

        # Add the new velocity to the window
        self.velocity_window.append(jointspace_velocity.flatten())
        
        # Calculate the moving average
        filtered_velocity = np.mean(self.velocity_window, axis=0)
        print("jointspace_velocity:",filtered_velocity.reshape((6, 1)))

        current_time = rospy.get_time()
        self.time_stamps.append(current_time)
        # self.jointspace_velocity_plot.append(jointspace_velocity.flatten())
        self.jointspace_velocity_plot.append(filtered_velocity)
        self.robot_jointstate_plot.append(self.robot_jointstate.flatten())
        self.haptic_jointstate_plot.append(self.haptic_jointstate.flatten())

        # return jointspace_velocity.flatten()
        return filtered_velocity
    
    def robot_jointstate_callback(self, msg: JointState):
        if time.time() - self.start_time < 1.0:
            return
        if self.inital_robot_jointstate_flag is True:
            self.intial_robot_jointstate = np.array([[msg.position[2]], [msg.position[1]], [msg.position[0]], [msg.position[3]], [msg.position[4]], [msg.position[5]]])
            self.inital_robot_jointstate_flag = False
        self.robot_jointstate = np.array([[msg.position[2]], [msg.position[1]], [msg.position[0]], [msg.position[3]], [msg.position[4]], [msg.position[5]]]) - self.intial_robot_jointstate
    
    def haptic_jointstate_callback(self, msg: JointState):
        if time.time() - self.start_time < 1.0:
            return
        if self.inital_haptic_jointstate_flag is True:
            self.intial_haptic_jointstate = np.array([[msg.position[0]], [msg.position[1]], [msg.position[2]], [msg.position[4]], [msg.position[3]], [msg.position[5]]])
            self.inital_haptic_jointstate_flag = False
        self.haptic_jointstate = np.array([[msg.position[0]], [msg.position[1]], [msg.position[2]], [msg.position[4]], [msg.position[3]], [msg.position[5]]]) - self.intial_haptic_jointstate

    def button_callback(self, msg: OmniButtonEvent):
            if msg.grey_button:
                rospy.loginfo("Grey button pressed")
                if self.constant < 1:
                    self.constant = self.constant + 0.25
            if msg.white_button:
                rospy.loginfo("White button pressed")
                if self.constant > 0:
                    self.constant = self.constant - 0.25

    def velocity_callback(self, event):
        self.running = False
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = self.jointspace_velocity_calculator()
        self.velocity_pub.publish(velocity_pub_msg)

    def plot_data(self):
        """ Plot the logged data. """

        times = np.array(self.time_stamps)
        jointspace_velocity_plot = np.array(self.jointspace_velocity_plot)
        robot_jointstate_plot = np.array(self.robot_jointstate_plot)
        haptic_jointstate_plot = np.array(self.haptic_jointstate_plot)

        fig, axs = plt.subplots(4, 1, figsize=(10, 15))
        
        axs[0].plot(times, jointspace_velocity_plot)
        axs[0].set_title('Joint Space Velocities')
        axs[0].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        axs[1].plot(times, robot_jointstate_plot)
        axs[1].set_title('Robot Joint State')
        axs[1].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        axs[2].plot(times, haptic_jointstate_plot)
        axs[2].set_title('Haptic Joint State')
        axs[2].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        axs[3].plot(times, haptic_jointstate_plot-robot_jointstate_plot)
        axs[3].set_title('Difference')
        axs[3].legend(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])

        plt.tight_layout()
        plt.show()

    def interruption(self):
        velocity_pub_msg = Float64MultiArray()
        velocity_pub_msg.data = np.array([0, 0, 0, 0, 0, 0])
        self.velocity_pub.publish(velocity_pub_msg)

    def main_loop(self): 
        rate = 500
        rospy.Timer(rospy.Duration(1.0/rate),self.velocity_callback)
        rospy.spin()

if __name__ == "__main__":
    controller = RobotEndEffectorController()
    try:
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.interruption()
        controller.plot_data()