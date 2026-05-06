#!/usr/bin/env python3
import rospy
import time
import numpy as np
from omni_msgs.msg import OmniFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray 
from collections import deque
from omni_msgs.msg import OmniState

class MasterController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('master', anonymous=True)

         # Initialize instance variables
        self.fmd = np.zeros(3)
        self.fmd_corrected = np.zeros(3)
        self.vmc = np.zeros(3)
        self.E_M_out = 0.0
        self.E_M_in = 0.0
        self.E_S_in = 0.0
        self.prev_time = None

        # For velocity filtering
        self.velocity_buffer = deque(maxlen=25)

        # Publishers
        self.force_pub = rospy.Publisher('/phantom/phantom/force_feedback', OmniFeedback, queue_size=1)
        self.msg_pub = rospy.Publisher('message_from_master_to_slave', Float64MultiArray, queue_size=1)

        # Subscribers
        rospy.Subscriber('force_from_slave_to_master', Float64MultiArray, self.force_from_slave_to_master_callback)
        rospy.Subscriber('energy_from_slave_to_master', Float64MultiArray, self.energy_from_slave_to_master_callback)
        rospy.Subscriber('/phantom/phantom/state', OmniState, self.velocity_callback)

    @staticmethod
    def E_M_calculation(fmd, vmc, E_M_out, E_M_in, delta_t):
        P1 = fmd[2] * vmc[2]
        #print(f'fmd: {fmd[2]:.3f}')
        #print(f"P1: {P1:.3f}, fmd: {fmd[2]:.3f}, vmc: {vmc[2]:.3f}")
        if P1 > 0:
            E_M_in += P1 * delta_t
        elif P1 < 0:
            E_M_out -= P1 * delta_t
        #print(f"E_M_out: {E_M_out}, E_M_in: {E_M_in}")
        return E_M_out, E_M_in

    # @staticmethod
    # def alpha_calculation(E_M_out, E_S_in, vmc, delta_t):
    #     if delta_t <= 0 or vmc[2] == 0:
    #         return 0.0
    #     if E_M_out > E_S_in:
    #         #print(f"E_M_out: {E_M_out}, E_S_in: {E_S_in}, delta_t: {delta_t}")
    #         #print(f"numerator: {E_M_out - E_S_in}, denominator: {delta_t * (vmc[2] ** 2)}")
    #         return (E_M_out - E_S_in) / (delta_t * (vmc[2] ** 2))
    #     else:
    #         return 0.0

    '''this is for testing, the upper one is the original'''
    @staticmethod
    def alpha_calculation(E_M_out, E_S_in, vmc, delta_t):
        if delta_t <= 0:
            return 0.0
        if abs(vmc[2]) < 0.001:
            return 0.0
        if E_M_out > E_S_in:
            #print(f"E_M_out: {E_M_out}, E_S_in: {E_S_in}, delta_t: {delta_t}")
            #print(f"numerator: {E_M_out - E_S_in}, denominator: {delta_t * (vmc[2] ** 2)}")
            return (E_M_out - E_S_in) / (delta_t * (vmc[2] ** 2))
        else:
            return 0.0

    def force_from_slave_to_master_callback(self, msg: Float64MultiArray):
        self.fmd = np.array(msg.data)
        #print(f"fmd: {self.fmd}")

    def energy_from_slave_to_master_callback(self, msg: Float64MultiArray):
        self.E_S_in = msg.data[0] if msg.data else 0.0
        #print(f"E_S_in: {self.E_S_in}")

    def velocity_callback(self, msg: OmniState):
        # self.vmc = 0.001 * np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        new_velocity = 0.001 * np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        self.velocity_buffer.append(new_velocity)
        self.vmc = np.mean(self.velocity_buffer, axis=0)
        #print(f"vmc: {self.vmc}")

        current_time = time.time()
        if self.prev_time is not None:
            delta_t = current_time - self.prev_time
            self.E_M_out, self.E_M_in = self.E_M_calculation(self.fmd, self.vmc, self.E_M_out, self.E_M_in, delta_t)
            alpha = self.alpha_calculation(self.E_M_out, self.E_S_in, self.vmc, delta_t)
            print(f"alpha: {alpha:.3f}, E_M_out: {self.E_M_out:.3f}, E_M_in: {self.E_M_in:.3f}, E_S_in: {self.E_S_in:.3f}, delta_t: {delta_t:.3f}, fmd: {self.fmd[2]:.3f}, vmc: {self.vmc[2]:.3f}")
        else:
            alpha = 0.0
        self.prev_time = current_time

        self.fmd_corrected[2] = self.fmd[2] + alpha * self.vmc[2]

        force_pub_msg = OmniFeedback()
        # force_pub_msg.force.x, force_pub_msg.force.y, force_pub_msg.force.z = self.fmd

        force_pub_msg.force.x = 0.0
        force_pub_msg.force.y = 0.0
        force_pub_msg.force.z = self.fmd_corrected[2]


        #print(f"force: {self.fmd_corrected[2]:.3f}, alpha: {alpha}")
        #self.force_pub.publish(force_pub_msg)

        message = Float64MultiArray()
        combined_message = np.concatenate(([self.E_M_in], self.vmc.flatten())).tolist()
        message.data = combined_message
        self.msg_pub.publish(message)

    def main_loop(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = MasterController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
