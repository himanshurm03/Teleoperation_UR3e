#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

class RobotEndEffectorController:

    def __init__(self):
        rospy.init_node('version0_pose', anonymous=True)
        self.pose_pub = rospy.Publisher("pose_from_master_to_slave", PoseStamped, queue_size=1)
        rospy.Subscriber('/phantom/phantom/pose', PoseStamped, self.pose_from_master_to_slave_callback)

    def pose_from_master_to_slave_callback(self, msg: PoseStamped):
        self.pose_pub.publish(msg)

    def main_loop(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = RobotEndEffectorController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass
