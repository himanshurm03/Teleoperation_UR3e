#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

last_position = None
last_time = None

def joint_state_callback(msg):
    global last_position, last_time

    if last_position is not None:
        dt = (msg.header.stamp - last_time).to_sec()
        if dt > 0:
            velocities = [(p - lp)/dt for p, lp in zip(msg.position, last_position)]
            rospy.loginfo("Estimated velocities: %s", velocities)

    last_position = msg.position
    last_time = msg.header.stamp

if __name__ == "__main__":
    rospy.init_node("phantom_velocity_estimator")
    rospy.Subscriber("/phantom/phantom/joint_states", JointState, joint_state_callback)
    rospy.spin()
