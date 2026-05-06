#!/usr/bin/env python3

import rospy
import numpy as np
import sys
import time
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal,JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from cartesian_control_msgs.msg import FollowCartesianTrajectoryActionFeedback
from controller_manager_msgs.srv import LoadControllerRequest, LoadController







global J,joint_position,joint_trajectory_controller
J = []
joint_position = np.empty(6)


# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

# List of all joint trajectory controllers
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# List of all Cartesian trajectory controllers
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# List of conflicting controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[1]

# def load_controllers():
   
#     # Initialize the load controller service proxy
#     load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
#     timeout = rospy.Duration(5)
#     try:
#         load_srv.wait_for_service(timeout.to_sec())
#     except rospy.exceptions.ROSException as err:
#         rospy.logerr("Could not reach controller load service. Msg: {}".format(err))
#         sys.exit(-1)

#     # Combine all controllers except conflicting ones
#     controllers_to_load = (
#         JOINT_TRAJECTORY_CONTROLLERS + CARTESIAN_TRAJECTORY_CONTROLLERS + CONFLICTING_CONTROLLERS
#     )

#     # Load each controller
#     for controller in controllers_to_load:
#         # if controller not in CONFLICTING_CONTROLLERS:
#         rospy.loginfo(f"Loading controller: {controller}")
#         srv = LoadControllerRequest()
#         srv.name = controller
#         response = load_srv(srv)
#         if response.ok:
#             rospy.loginfo(f"Successfully loaded controller: {controller}")
#         else:
#             rospy.logerr(f"Failed to load controller: {controller}")

# def joint_callback():
#     global linear_pos,quaternion
#     n = 6
#     alp = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
#     a = [0, -0.24355, -0.2132, 0, 0, 0]
#     d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
#     th = np.deg2rad([9.80, -225.01, 107.47, -172.69, -100.40, 270.80])
#     d_E_n = np.array([0, 0, 0.145])
#     T_i = np.eye(4)
#     for i in range(n):
#         T_i_i_m = np.array([
#             [np.cos(th[i]), -np.sin(th[i]) * np.cos(alp[i]), np.sin(th[i]) * np.sin(alp[i]), a[i] * np.cos(th[i])],
#             [np.sin(th[i]), np.cos(th[i]) * np.cos(alp[i]), -np.cos(th[i]) * np.sin(alp[i]), a[i] * np.sin(th[i])],
#             [0, np.sin(alp[i]), np.cos(alp[i]), d[i]],
#             [0, 0, 0, 1]
#         ])
#         T_i_0 = T_i @ T_i_i_m
#         T_i = T_i_0
#     T_n_0 = T_i_0
#     P_E_0 = T_n_0 @ np.append(d_E_n, 1)
#     p_E_0 = P_E_0[:3]

#     # Extract rotation matrix from T_n_0
#     R_n_0 = T_n_0[:3, :3]

#     # Compute roll, pitch, and yaw from the rotation matrix
#     sy = np.sqrt(R_n_0[2, 2] ** 2 + R_n_0[2, 1] ** 2)
#     singular = sy < 1e-6

#     if not singular:
#         roll = np.arctan2(R_n_0[2, 1], R_n_0[2, 2])
#         pitch = np.arctan2(-R_n_0[2, 0], sy)
#         yaw = np.arctan2(R_n_0[1, 0], R_n_0[0, 0])
#     else:
#         roll = np.arctan2(-R_n_0[1, 2], R_n_0[1, 1])
#         pitch = np.arctan2(-R_n_0[2, 0], sy)
#         yaw = 0





def joint_vel_cb(msg):

    joint_velocity = msg.actual.velocities
    print("joint velocity: ",joint_velocity)

def end_eff_cb(msg):

    X_vel_ee = msg.feedback.actual.twist.linear.x
    Y_vel_ee = msg.feedback.actual.twist.linear.y
    Z_vel_ee = msg.feedback.actual.twist.linear.z

    X_angvel_ee = msg.feedback.actual.twist.angular.x
    Y_angvel_ee = msg.feedback.actual.twist.angular.y
    Z_angvel_ee = msg.feedback.actual.twist.angular.z

    print("X_vel_lin: ",X_vel_ee)
    print("X_vel_ang: ",X_angvel_ee)
    # print("a")


def joint_callback(msg):
    global linear_pos, quaternion,J,joint_position

    joint_position = msg.position

    n = 6
    alp = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    a = [0, -0.24355, -0.2132, 0, 0, 0]
    d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
    #th = np.deg2rad([9.80, -225.01, 107.47, -172.69, -100.40, 270.80])

    th = ([joint_position[2], joint_position[1],joint_position[0],joint_position[3], joint_position[4], joint_position[5]])

    d_E_n = np.array([0, 0, 0.145])
    T_i = np.eye(4)
    Ts = [T_i]  # Store transformation matrices

    for i in range(n):
        T_i_i_m = np.array([
            [np.cos(th[i]), -np.sin(th[i]) * np.cos(alp[i]), np.sin(th[i]) * np.sin(alp[i]), a[i] * np.cos(th[i])],
            [np.sin(th[i]), np.cos(th[i]) * np.cos(alp[i]), -np.cos(th[i]) * np.sin(alp[i]), a[i] * np.sin(th[i])],
            [0, np.sin(alp[i]), np.cos(alp[i]), d[i]],
            [0, 0, 0, 1]
        ])
        T_i_0 = T_i @ T_i_i_m
        Ts.append(T_i_0)  # Append transformation matrix to list
        T_i = T_i_0

    T_n_0 = T_i_0
    P_E_0 = T_n_0 @ np.append(d_E_n, 1)
    p_E_0 = P_E_0[:3]

    # Jacobian Calculation
    J = np.zeros((6, n))
    for i in range(n):
        T_i_0 = Ts[i]
        R_i_0 = T_i_0[:3, :3]
        p_i_0 = T_i_0[:3, 3]

        # z-axis of each joint in base frame
        z_i_0 = R_i_0[:, 2]

        # Linear velocity part
        J_p = np.cross(z_i_0, (p_E_0 - p_i_0))

        # Angular velocity part
        J_o = z_i_0

        # Assemble Jacobian
        J[:3, i] = J_p
        J[3:, i] = J_o

    print("Transformation matrix: ")
    print(T_n_0)
    print("Jacobian Matrix:")
    print(J)
    

    return J, p_E_0


def joint_joint_trajectory():
    global joint_trajectory_controller, joint_position, JOINT_NAMES,J
    global js_start, t_init, t_now

    rospy.sleep(5)  # Wait for action server to start
    goal = FollowJointTrajectoryGoal()
    trajectory_client = actionlib.SimpleActionClient(
        "{}/follow_joint_trajectory".format(joint_trajectory_controller),
        FollowJointTrajectoryAction,
    )
    timeout = rospy.Duration(5)

    if not trajectory_client.wait_for_server(timeout):
        rospy.logerr("Could not reach controller action server.")
        sys.exit(-1)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = JOINT_NAMES

    # Ensure that joint_position has been initialized by the callback
    if joint_position is None or len(joint_position) != 6:
        rospy.logerr("Joint position is not set correctly.")
        sys.exit(-1)

    # Initialize positions and velocities lists
    #position_list = list(joint_position)
    position_list = list(joint_position)
    velocity_list = [0.0,0.0,-0.0,0.0,0.0,0.0]

    end_eff_vel = [0.1,0,0,0,0,0]
    velocity_list = np.matmul(np.linalg.pinv(J),end_eff_vel)

    # Modify the position and velocity for the joint you want to move (e.g., wrist_2_joint)
    joint_index = 1  # Index for wrist_2_joint in JOINT_NAMES
    if js_start:
        js_start = False
        t_init = time.time()

    t_now = time.time() - t_init
    print(t_now)

    # Example: Moving wrist_2_joint with a certain velocity
    #velocity_list[joint_index] = 0.2  # Set desired velocity for the joint
    duration_list = [3.0, 7.0, 10.0]
    
    for duration in duration_list:
        point = JointTrajectoryPoint()
        #point.positions = position_list  # Maintain current positions
        point.positions = joint_position
        point.velocities = velocity_list  # Apply the velocity
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)

    trajectory_client.send_goal(goal)
    trajectory_client.wait_for_result()






def main():
    rospy.init_node('Vel_Control_2',anonymous=True)
    # joint_callback()
    #load_controllers()
    rospy.Subscriber('/joint_states', JointState, joint_callback)
    #rospy.Subscriber('/joint_states', JointState, joint_callback)
    rospy.Subscriber('/pose_based_cartesian_traj_controller/follow_cartesian_trajectory/feedback',FollowCartesianTrajectoryActionFeedback,end_eff_cb)
    rospy.Subscriber('/scaled_vel_joint_traj_controller/state',JointTrajectoryControllerState,joint_vel_cb)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
       pass