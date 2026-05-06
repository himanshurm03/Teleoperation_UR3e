#!/usr/bin/env python3

import rospy
import numpy as np
import sys
import actionlib
from tf2_msgs.msg import TFMessage
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R    #python lib



global linear_pos, cartesian_trajectory_controller,end_eff
# CARTESIAN_TRAJECTORY_CONTROLLERS = [
#     "pose_based_cartesian_traj_controller",
#     "joint_based_cartesian_traj_controller",
#     "forward_cartesian_traj_controller",
# ]
# cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
cartesian_trajectory_controller = "pose_based_cartesian_traj_controller"

linear_pos = np.empty(3)
quaternion = np.empty(4)
end_eff = np.empty(3)

def euler_to_quat(roll_arg,pitch_arg,yaw_arg):
    r = R.from_euler('xyz',[roll_arg,pitch_arg,yaw_arg],degrees=False)
    quat = r.as_quat()
    return quat

def joint_callback():
    global linear_pos,quaternion
    n = 6
    alp = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
    a = [0, -0.24355, -0.2132, 0, 0, 0]
    d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
    
 
    

    th = [
        np.array([ -0.06086141267885381, -3.904510875741476,1.840231720601217, 3.5767227846333007, -1.7778661886798304, 4.164084434509277]),
        np.array([2.694200038909912, -0.053385929470398,  -1.5220253467559814, 3.267535849208496, -2.20729905763735, -3.7819090525256556]),
        np.array([2.6942670345306396, -0.053393201237060595, -1.522813320159912, 3.2673379617878417, -1.2618702093707483, -3.7819982210742396]),
        np.array([2.694455146789551, -1.0988002282432099, -1.6221636533737183, 4.671582861537598, -1.4588177839862269, -3.821932379399435]),
        np.array([ 4.273403167724609, -1.4483108085444947,-1.5330666303634644, 4.568957968349121, -1.7348936239825647, -3.903097454701559])
    ]
    for j in th:
        
        th_for_forward_kin = j
        
        d_E_n = np.array([0, 0, 0.145])
        T_i = np.eye(4)
        for i in range(n):
            T_i_i_m = np.array([
                [np.cos(th_for_forward_kin[i]), -np.sin(th_for_forward_kin[i]) * np.cos(th_for_forward_kin[i]), np.sin(th_for_forward_kin[i]) * np.sin(alp[i]), a[i] * np.cos(th_for_forward_kin[i])],
                [np.sin(th_for_forward_kin[i]), np.cos(th_for_forward_kin[i]) * np.cos(alp[i]), -np.cos(th_for_forward_kin[i]) * np.sin(alp[i]), a[i] * np.sin(th_for_forward_kin[i])],
                [0, np.sin(alp[i]), np.cos(alp[i]), d[i]],
                [0, 0, 0, 1]
            ])
            T_i_0 = T_i @ T_i_i_m
            T_i = T_i_0
        T_n_0 = T_i_0
        P_E_0 = T_n_0 @ np.append(d_E_n, 1)
        p_E_0 = P_E_0[:3]

        # Extract rotation matrix from T_n_0
        R_n_0 = T_n_0[:3, :3]

        # Compute roll, pitch, and yaw from the rotation matrix
        sy = np.sqrt(R_n_0[2, 2] ** 2 + R_n_0[2, 1] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R_n_0[2, 1], R_n_0[2, 2])
            pitch = np.arctan2(-R_n_0[2, 0], sy)
            yaw = np.arctan2(R_n_0[1, 0], R_n_0[0, 0])
        else:
            roll = np.arctan2(-R_n_0[1, 2], R_n_0[1, 1])
            pitch = np.arctan2(-R_n_0[2, 0], sy)
            yaw = 0
        
        linear_pos = p_E_0

        quaternion = euler_to_quat(roll,pitch,yaw)
        #quaternion = np.array([0,0,0,1])
        #quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        print("Linear pose: ",linear_pos)
        print(f"Roll: {roll}, pitch: {pitch}, yaw: {yaw}")
        print("Quaternions: ",quaternion)



def send_cartesian_trajectory():
    global linear_pos, cartesian_trajectory_controller, quaternion
    

    linear_pos = np.array([0.387,-0.259,0.558])
    quaternion = np.array([0.0,0.0,0,1])
    # rospy.sleep(5)  # Wait for action server to start
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
    
    # Send the goal and wait for the result
    result = trajectory_client.send_goal_and_wait(goal, rospy.Duration(5))
    state = trajectory_client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal succeeded!")
    else:
        rospy.logwarn("Goal failed with state: %s", state)

def tf_callback(msg):
    global end_eff
    Xee = msg.transforms[0].transform.translation.x
    Yee = msg.transforms[0].transform.translation.y
    Zee = msg.transforms[0].transform.translation.z
    end_eff = np.array([Xee, Yee, Zee])
    print(end_eff)

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


############################################################   Giving 10 points  ###############################
#################################################################################################

# import rospy
# import numpy as np
# import sys
# import actionlib
# from tf2_msgs.msg import TFMessage
# from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint
# from geometry_msgs.msg import Pose, Point, Quaternion
# from scipy.spatial.transform import Rotation as R    #python lib



# global linear_pos, cartesian_trajectory_controller,end_eff
# CARTESIAN_TRAJECTORY_CONTROLLERS = [
#     "pose_based_cartesian_traj_controller",
#     "joint_based_cartesian_traj_controller",
#     "forward_cartesian_traj_controller",
# ]
# cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
# linear_pos = np.empty(3)
# quaternion = np.empty(4)
# end_eff = np.empty(3)


# def euler_to_quat(roll_arg,pitch_arg,yaw_arg):
#     r = R.from_euler('xyz',[roll_arg,pitch_arg,yaw_arg],degrees=False)
#     quat = r.as_quat()
#     return quat

# def joint_callback():
#     global linear_pos,quaternion
#     # n = 6
#     # alp = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
#     # a = [0, -0.24355, -0.2132, 0, 0, 0]
#     # d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
    
 
    

#     # th = [
#     #     np.array([-0.2905386129962366, -3.1567350826659144, 1.019900147114889, 3.7073137003132324, -1.5686028639422815, -1.278116528187887]),
#     #     np.array([-0.29051524797548467, -3.115457674066061, 0.9783290068255823, 3.707613154048584, -1.5687029997455042, -1.2781804243670862]),
#     #     np.array([-0.2904909292804163, -3.0733829937376917, 0.933575455342428, 3.710282488460205, -1.5688141028033655, -1.2781804243670862]),
#     #     np.array([-0.29041892686952764, -3.0294486484923304, 0.8843029181109827, 3.7156669336506347, -1.5689070860492151, -1.278231445943014]),
#     #     np.array([-0.2903826872455042, -2.984099050561422, 0.8306553999530237, 3.7239438730427246, -1.5689995924579065, -1.2782452742206019]),
#     #     np.array([-0.29035884538759404, -2.9364806614317835, 0.7712515036212366, 3.735712690944336, -1.5691388289081019, -1.278320614491598]),
#     #     np.array([-0.290302578602926, -2.8863774738707484, 0.705367390309469, 3.751509352321289, -1.5692361036883753, -1.2784083525287073]),
#     #     np.array([-0.29028207460512334, -2.832578798333639, 0.6305516401873987, 3.7725106912800292, -1.5693839232074183, -1.278480354939596]),
#     #     np.array([-0.29020387331117803, -2.773736616174215, 0.5438569227801722, 3.8003999429890136, -1.569552246724264, -1.2786195913897913]),
#     #     np.array([-0.2901557127581995, -2.705784936944479, 0.4369733969317835, 3.8393236833759765, -1.5697358290301722, -1.2786863485919397])
#     # ]
#     # for j in th:
        
#     #     th_for_forward_kin = j
        
#     #     d_E_n = np.array([0, 0, 0.145])
#     #     T_i = np.eye(4)
#     #     for i in range(n):
#     #         T_i_i_m = np.array([
#     #             [np.cos(th_for_forward_kin[i]), -np.sin(th_for_forward_kin[i]) * np.cos(th_for_forward_kin[i]), np.sin(th_for_forward_kin[i]) * np.sin(alp[i]), a[i] * np.cos(th_for_forward_kin[i])],
#     #             [np.sin(th_for_forward_kin[i]), np.cos(th_for_forward_kin[i]) * np.cos(alp[i]), -np.cos(th_for_forward_kin[i]) * np.sin(alp[i]), a[i] * np.sin(th_for_forward_kin[i])],
#     #             [0, np.sin(alp[i]), np.cos(alp[i]), d[i]],
#     #             [0, 0, 0, 1]
#     #         ])
#     #         T_i_0 = T_i @ T_i_i_m
#     #         T_i = T_i_0
#     #     T_n_0 = T_i_0
#     #     P_E_0 = T_n_0 @ np.append(d_E_n, 1)
#     #     p_E_0 = P_E_0[:3]

#     #     # Extract rotation matrix from T_n_0
#     #     R_n_0 = T_n_0[:3, :3]

#     #     # Compute roll, pitch, and yaw from the rotation matrix
#     #     sy = np.sqrt(R_n_0[2, 2] ** 2 + R_n_0[2, 1] ** 2)
#     #     singular = sy < 1e-6

#     #     if not singular:
#     #         roll = np.arctan2(R_n_0[2, 1], R_n_0[2, 2])
#     #         pitch = np.arctan2(-R_n_0[2, 0], sy)
#     #         yaw = np.arctan2(R_n_0[1, 0], R_n_0[0, 0])
#     #     else:
#     #         roll = np.arctan2(-R_n_0[1, 2], R_n_0[1, 1])
#     #         pitch = np.arctan2(-R_n_0[2, 0], sy)
#     #         yaw = 0
        
#     #     linear_pos = p_E_0

#     #     quaternion = euler_to_quat(roll,pitch,yaw)
#     #     #quaternion = np.array([0,0,0,1])
#     #     #quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
#     #     print("Linear pose: ",linear_pos)
#     #     print(f"Roll: {roll}, pitch: {pitch}, yaw: {yaw}")
#     #     print("Quaternions: ",quaternion)

#     pose_list = [np.array([0.387,-0.253,0.558,0,0,0,1]),
#                 np.array([0.387,-0.253,0.568,0,0,0,1]),
#                 np.array([0.387,-0.253,0.578,0,0,0,1]),
#                 np.array([0.387,-0.253,0.588,0,0,0,1]),
#                 np.array([0.387,-0.253,0.598,0,0,0,1]),
#                 np.array([0.387,-0.253,0.608,0,0,0,1]),
#                 np.array([0.387,-0.253,0.618,0,0,0,1]),
#                 np.array([0.387,-0.253,0.628,0,0,0,1]),
#                 np.array([0.387,-0.253,0.638,0,0,0,1]),
#                 np.array([0.387,-0.253,0.648,0,0,0,1]),
#                  ]
        
#     for k in pose_list:

#         l_pos = k
#         linear_pos = np.array([l_pos[0],l_pos[1],l_pos[2]])
#         quaternion = np.array([l_pos[3],l_pos[4],l_pos[5],l_pos[6]])
    
#         send_cartesian_trajectory()



# def send_cartesian_trajectory():
#     global linear_pos, cartesian_trajectory_controller, quaternion

#     rospy.sleep(1)  # Wait for action server to start
#     goal = FollowCartesianTrajectoryGoal()
#     trajectory_client = actionlib.SimpleActionClient(
#         "{}/follow_cartesian_trajectory".format(cartesian_trajectory_controller),
#         FollowCartesianTrajectoryAction,
#     )
#     timeout = rospy.Duration(1)
#     if not trajectory_client.wait_for_server(timeout):
#         rospy.logerr("Could not reach controller action server.")
#         sys.exit(-1)
#     if linear_pos is None or linear_pos.size != 3:
#         rospy.logerr("Linear position is not set correctly.")
#         sys.exit(-1)
#     point = CartesianTrajectoryPoint()
#     point.pose.position = Point(*linear_pos)
#     point.pose.orientation = Quaternion(*quaternion)
#     point.time_from_start = rospy.Duration(2)
#     goal.trajectory.points.append(point)
    
#     # Send the goal and wait for the result
#     result = trajectory_client.send_goal_and_wait(goal, rospy.Duration(2))
#     state = trajectory_client.get_state()
#     if state == actionlib.GoalStatus.SUCCEEDED:
#         rospy.loginfo("Goal succeeded!")
#     else:
#         rospy.logwarn("Goal failed with state: %s", state)

# def tf_callback(msg):
#     global end_eff
#     Xee = msg.transforms[0].transform.translation.x
#     Yee = msg.transforms[0].transform.translation.y
#     Zee = msg.transforms[0].transform.translation.z
#     end_eff = np.array([Xee, Yee, Zee])
#     print(end_eff)

# def main():
#     rospy.init_node('task_space_multiple_points', anonymous=True)
#     rospy.Subscriber('/tf', TFMessage, tf_callback)
#     joint_callback()
#     rospy.spin()

# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass



###############################################################################################
#######################################  Running action client once   ###########################
##########################################


# import rospy
# import numpy as np
# import sys
# import actionlib
# from tf2_msgs.msg import TFMessage
# from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint,FollowCartesianTrajectoryActionFeedback
# from geometry_msgs.msg import Point, Quaternion
# from scipy.spatial.transform import Rotation as R
# import pandas as pd
# from datetime import datetime 

# global linear_pos, cartesian_trajectory_controller, end_eff, trajectory_client,end_eff_vel, start_time
# CARTESIAN_TRAJECTORY_CONTROLLERS = [
#     "pose_based_cartesian_traj_controller",
#     "joint_based_cartesian_traj_controller",
#     "forward_cartesian_traj_controller",
# ]
# cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
# linear_pos = np.empty(3)
# quaternion = np.empty(4)
# end_eff = np.empty(3)
# end_eff_vel = np.empty(6)
# start_time = 0.0

# # Initialize a DataFrame to store the end-effector velocity data
# global df
# df = pd.DataFrame(columns=["timestamp", "x_vel", "y_vel", "z_vel", "x_ang_vel", "y_ang_vel", "z_ang_vel"])


# def euler_to_quat(roll_arg, pitch_arg, yaw_arg):
#     r = R.from_euler('xyz', [roll_arg, pitch_arg, yaw_arg], degrees=False)
#     quat = r.as_quat()
#     return quat

# def initialize_action_client():
#     global trajectory_client
#     trajectory_client = actionlib.SimpleActionClient(
#         "{}/follow_cartesian_trajectory".format(cartesian_trajectory_controller),
#         FollowCartesianTrajectoryAction,
#     )
#     timeout = rospy.Duration(5)
#     if not trajectory_client.wait_for_server(timeout):
#         rospy.logerr("Could not reach controller action server.")
#         sys.exit(-1)

# def joint_callback():
#     global linear_pos, quaternion

#     pose_list = [
#         np.array([0.387, -0.253, 0.558, 0, 0, 0, 1]),
#         np.array([0.387, -0.253, 0.568, 0, 0, 0, 1]),
#         np.array([0.387, -0.253, 0.578, 0, 0, 0, 1]),
#         np.array([0.387, -0.253, 0.588, 0, 0, 0, 1]),
#         np.array([0.387, -0.253, 0.598, 0, 0, 0, 1]),
#         np.array([0.387, -0.253, 0.608, 0, 0, 0, 1]),
#         np.array([0.387, -0.253, 0.618, 0, 0, 0, 1]),
#         np.array([0.387, -0.253, 0.628, 0, 0, 0, 1]),
#         np.array([0.387, -0.253, 0.638, 0, 0, 0, 1]),
#         np.array([0.387, -0.253, 0.648, 0, 0, 0, 1]),
#     ]
    
#     for idx, pose in enumerate(pose_list):
#         linear_pos = pose[:3]
#         quaternion = pose[3:]
        
#         send_cartesian_trajectory(idx * 1)

# def send_cartesian_trajectory(time_from_start):
#     global linear_pos, quaternion, trajectory_client

#     if linear_pos is None or linear_pos.size != 3:
#         rospy.logerr("Linear position is not set correctly.")
#         sys.exit(-1)

#     goal = FollowCartesianTrajectoryGoal()  # Initialize goal here to clear previous points
#     point = CartesianTrajectoryPoint()
#     point.pose.position = Point(*linear_pos)
#     point.pose.orientation = Quaternion(*quaternion)
#     point.time_from_start = rospy.Duration(time_from_start)
#     goal.trajectory.points.append(point)
    
#     # Send the goal and wait for the result
#     trajectory_client.send_goal(goal)
#     #trajectory_client.wait_for_result()

#     state = trajectory_client.get_state()
#     if state == actionlib.GoalStatus.SUCCEEDED:
#         rospy.loginfo("Goal succeeded!")
#     else:
#         rospy.logwarn("Goal failed with state: %s", state)

# def tf_callback(msg):
#     global end_eff
#     Xee = msg.transforms[0].transform.translation.x
#     Yee = msg.transforms[0].transform.translation.y
#     Zee = msg.transforms[0].transform.translation.z
#     end_eff = np.array([Xee, Yee, Zee])
#     print(end_eff)

# def end_eff_cb(msg):

#     global end_eff_vel,df,  start_time 
    
#     X_vel_ee = msg.feedback.actual.twist.linear.x
#     Y_vel_ee = msg.feedback.actual.twist.linear.y
#     Z_vel_ee = msg.feedback.actual.twist.linear.z

#     X_angvel_ee = msg.feedback.actual.twist.angular.x
#     Y_angvel_ee = msg.feedback.actual.twist.angular.y
#     Z_angvel_ee = msg.feedback.actual.twist.angular.z

#     print("X_vel_lin: ",X_vel_ee)
#     print("X_vel_ang: ",X_angvel_ee)

#     end_eff_vel = np.array([X_vel_ee,Y_vel_ee,Z_vel_ee,X_angvel_ee,Y_angvel_ee,Z_angvel_ee])
    

#     # Get the elapsed time in seconds
    
#     elapsed_time = (datetime.now() - start_time).total_seconds()
    
#     # Append the data to the DataFrame
#     new_data = pd.DataFrame({
#         "timestamp": [elapsed_time],
#         "x_vel": [X_vel_ee],
#         "y_vel": [Y_vel_ee],
#         "z_vel": [Z_vel_ee],
#         "x_ang_vel": [X_angvel_ee],
#         "y_ang_vel": [Y_angvel_ee],
#         "z_ang_vel": [Z_angvel_ee]
#     })
#     df = pd.concat([df, new_data], ignore_index=True)
    
#     # Print the DataFrame to see the data in real-time
#     print(df)

# def save_data():
#     global df
#     save_path = '/home/user/catkin_UR_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/data_files/end_eff_vel.csv'
#     df.to_csv(save_path, index=False)


# def main():
    
#     global start_time
#     rospy.init_node('task_space_multiple_points', anonymous=True)

#     start_time = datetime.now()  # Initialize the start time

#     rospy.Subscriber('/tf', TFMessage, tf_callback)
#     rospy.Subscriber('/pose_based_cartesian_traj_controller/follow_cartesian_trajectory/feedback',FollowCartesianTrajectoryActionFeedback,end_eff_cb)
#     initialize_action_client()
#     joint_callback()

#     # Set up a timer to periodically save the data
#     save_interval = rospy.Duration(1)  # Save every 1 seconds
#     rospy.Timer(save_interval, lambda event: save_data())
    
#     rospy.spin()

  
# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass
