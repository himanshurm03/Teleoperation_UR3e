#!/usr/bin/env python3
import rospy
import time
from omni_msgs.msg import OmniFeedback
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64 

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

    def time_from_slave_to_master_callback(self, msg: Float64):
        delay = (rospy.get_time() - msg.data) / 2
        rospy.loginfo(f"Delay for force: {delay}")

    def force_from_slave_to_master_callback(self, msg: OmniFeedback):
        if not self.shutdown_flag:
            self.force_pub.publish(msg)
        time_msg = Float64()
        time_msg.data = rospy.get_time()
        self.time_pub.publish(time_msg)

    def pose_from_master_to_slave_callback(self, msg: PoseStamped):
        self.pose_pub.publish(msg)

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