#!/usr/bin/env python3

import rospy
from cartesian_control_msgs.msg import FollowCartesianTrajectoryGoal, FollowCartesianTrajectoryAction, CartesianTrajectoryPoint
import numpy as np
import actionlib
from geometry_msgs.msg import Point, Quaternion
import sys
from tf2_msgs.msg import TFMessage

global linear_pos, cartesian_trajectory_controller, quaternion

cartesian_trajectory_controller = "pose_based_cartesian_traj_controller"

linear_pos = np.empty(3)
quaternion = np.empty(4)
end_eff = np.empty(3)

def send_cartesian_trajectory():

    global linear_pos, cartesian_trajectory_controller, quaternion
    
    linear_pos = np.array([0.387,-0.259,0.558])
    #linear_pos = np.array([0.250,-0.250,0.250])
    quaternion = np.array([0.0,0.0,0,1])
    goal = FollowCartesianTrajectoryGoal()
    trajectory_client = actionlib.SimpleActionClient(
        "{}/follow_cartesian_trajectory".format(cartesian_trajectory_controller),
        FollowCartesianTrajectoryAction,
    )
    timeout = rospy.Duration(5)
    if not trajectory_client.wait_for_server(timeout):
        rospy.logerr("Could not reach controller action server.")
        sys.exit(-1)
    if linear_pos is None or linear_pos.size != 3:
        rospy.logerr("Linear position is not set correctly.")
        sys.exit(-1)
    point = CartesianTrajectoryPoint()
    point.pose.position = Point(*linear_pos)
    point.pose.orientation = Quaternion(*quaternion)
    point.time_from_start = rospy.Duration(6)
    goal.trajectory.points.append(point)
    
    #Send the goal and wait for the result
    result = trajectory_client.send_goal_and_wait(goal, rospy.Duration(5))
    state = trajectory_client.get_state()
    # if state == actionlib.GoalStatus.SUCCEEDED:
    #     rospy.loginfo("Goal succeeded!")
    # else:
    #     rospy.logwarn("Goal failed with state: %s", state)

def tf_callback(msg):
    global end_eff
    Xee = msg.transforms[0].transform.translation.x
    Yee = msg.transforms[0].transform.translation.y
    Zee = msg.transforms[0].transform.translation.z
    end_eff = np.array([Xee, Yee, Zee])
    rospy.loginfo(end_eff)
    # print(end_eff)

def main():
    rospy.init_node('task_space', anonymous=True)
    rospy.Subscriber('/tf', TFMessage, tf_callback)
    send_cartesian_trajectory()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass