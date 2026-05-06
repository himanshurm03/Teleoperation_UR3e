#!/usr/bin/env python3

# import sys
# import rospy
# from controller_manager_msgs.srv import LoadController, LoadControllerRequest

# # Compatibility for python2 and python3
# if sys.version_info[0] < 3:
#     input = raw_input

# # List of all joint trajectory controllers
# JOINT_TRAJECTORY_CONTROLLERS = [
#     "scaled_pos_joint_traj_controller",
#     "scaled_vel_joint_traj_controller",
#     "pos_joint_traj_controller",
#     "vel_joint_traj_controller",
#     "forward_joint_traj_controller",
# ]

# # List of all Cartesian trajectory controllers
# CARTESIAN_TRAJECTORY_CONTROLLERS = [
#     "pose_based_cartesian_traj_controller",
#     "joint_based_cartesian_traj_controller",
#     "forward_cartesian_traj_controller",
# ]

# # List of conflicting controllers
# CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

# def load_controllers():
#     rospy.init_node("load_controllers")

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
#         JOINT_TRAJECTORY_CONTROLLERS + CARTESIAN_TRAJECTORY_CONTROLLERS
#     )

#     # Load each controller
#     for controller in controllers_to_load:
#         if controller not in CONFLICTING_CONTROLLERS:
#             rospy.loginfo(f"Loading controller: {controller}")
#             srv = LoadControllerRequest()
#             srv.name = controller
#             response = load_srv(srv)
#             if response.ok:
#                 rospy.loginfo(f"Successfully loaded controller: {controller}")
#             else:
#                 rospy.logerr(f"Failed to load controller: {controller}")

# if __name__ == "__main__":
#     load_controllers()




########################################    ALL CONTROLLERS INCLUDING CONFLICTING   ######################



import sys
import rospy
from controller_manager_msgs.srv import LoadController, LoadControllerRequest

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

def load_controllers():
    rospy.init_node("load_controllers")

    # Initialize the load controller service proxy
    load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
    timeout = rospy.Duration(5)
    try:
        load_srv.wait_for_service(timeout.to_sec())
    except rospy.exceptions.ROSException as err:
        rospy.logerr("Could not reach controller load service. Msg: {}".format(err))
        sys.exit(-1)

    # Combine all controllers except conflicting ones
    # controllers_to_load = (
    #     JOINT_TRAJECTORY_CONTROLLERS + CARTESIAN_TRAJECTORY_CONTROLLERS[0] + CONFLICTING_CONTROLLERS
    # )

    controllers_to_load = (
        CARTESIAN_TRAJECTORY_CONTROLLERS + JOINT_TRAJECTORY_CONTROLLERS + CONFLICTING_CONTROLLERS
    )

    # Load each controller
    for controller in controllers_to_load:
        # if controller not in CONFLICTING_CONTROLLERS:
        rospy.loginfo(f"Loading controller: {controller}")
        srv = LoadControllerRequest()
        srv.name = controller
        response = load_srv(srv)
        if response.ok:
            rospy.loginfo(f"Successfully loaded controller: {controller}")
        else:
            rospy.logerr(f"Failed to load controller: {controller}")

if __name__ == "__main__":
    load_controllers()
    rospy.spin()