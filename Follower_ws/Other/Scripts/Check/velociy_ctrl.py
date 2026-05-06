#!/usr/bin/env python3

# import rospy
# import numpy as np
# import sys
# import time
# import actionlib
# from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# import geometry_msgs.msg as geometry_msgs
# from sensor_msgs.msg  import JointState

# JOINT_NAMES = [
#     "shoulder_pan_joint",
#     "shoulder_lift_joint",
#     "elbow_joint",
#     "wrist_1_joint",
#     "wrist_2_joint",
#     "wrist_3_joint",
# ]

# JOINT_TRAJECTORY_CONTROLLERS = [
#     "scaled_pos_joint_traj_controller",
#     "scaled_vel_joint_traj_controller",
#     "pos_joint_traj_controller",
#     "vel_joint_traj_controller",
#     "forward_joint_traj_controller",
# ]

# global joint_trajectory_controller, joint_postion
# global js_start,t_init,t_now

# js_start = True
# t_init = 0
# t_now = 0

# joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[1]
# joint_postion = np.empty(6)


# def joint_callback(msg):
#     global joint_postion
#     joint_postion = msg.position


# def joint_cartesian_trajectory():

#     global joint_trajectory_controller,joint_postion,JOINT_NAMES
#     global js_start,t_init,t_now

#     rospy.sleep(5)  # Wait for action server to start
#     goal = FollowJointTrajectoryGoal()
#     trajectory_client = actionlib.SimpleActionClient(
#         "{}/follow_joint_trajectory".format(joint_trajectory_controller),
#         FollowJointTrajectoryAction,
#     )
#     timeout = rospy.Duration(5)

    
#     if not trajectory_client.wait_for_server(timeout):
#         rospy.logerr("Could not reach controller action server.")
#         sys.exit(-1)

#     goal = FollowJointTrajectoryGoal()
#     goal.trajectory.joint_names = JOINT_NAMES

# ############## my method   #################
#     # position_list = joint_postion


#     # if(js_start==True):
                
#     #     js_start = False
#     #     t_init = time.time()


#     # t_now = time.time() - t_init          
#     # print(t_now)

#     # #velocity_list = [[0.2*np.sin(2*np.pi*t_now), 0, 0, 0, 0, 0]]

#     # duration_list = [3.0, 7.0, 10.0] 

#     # point = JointTrajectoryPoint()
#     # point.positions = position_list
#     # point.velocities = velocity_list
#     # point.time_from_start = rospy.Duration(3)
# #######################################################

# ###################################  Test move method #####################

#     position_list = [joint_postion[0],joint_postion[1],joint_postion[2],joint_postion[3],joint_postion[4],joint_postion[5]]
#     position_list.append([joint_postion[0],joint_postion[1],joint_postion[2],joint_postion[3],joint_postion[4],joint_postion[5]])

#     position_list.append([joint_postion[0],joint_postion[1],joint_postion[2],joint_postion[3],joint_postion[4],joint_postion[5]])

#     velocity_list = [[0.0, 0, 0, 0, 0.2, 0]]
#     velocity_list.append([0.0, 0, 0, 0, -0.2, 0])
#     velocity_list.append([0, 0, 0, 0, 0, 0])
#     duration_list = [3.0, 7.0, 10.0]
#     for i, position in enumerate(position_list):
#         point = JointTrajectoryPoint()
#         point.positions = position
#         point.velocities = velocity_list[i]
#         point.time_from_start = rospy.Duration(duration_list[i])
#         goal.trajectory.points.append(point)

# ###########################################


#     goal.trajectory.points.append(point)


#     trajectory_client.send_goal(goal)
#     trajectory_client.wait_for_result()

    
    


# def main():
#     rospy.init_node('velocity_ctrl',anonymous=True)
#     #joint_callback()
#     rospy.Subscriber('/joint_states', JointState, joint_callback)
#     joint_cartesian_trajectory()
#     rospy.spin()

# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#        pass



############################################################  VIBRATION METHOD   ###########################################


# # #!/usr/bin/env python3

# import rospy
# import numpy as np
# import sys
# import time
# import actionlib
# from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from sensor_msgs.msg import JointState

# JOINT_NAMES = [
#     "shoulder_pan_joint",
#     "shoulder_lift_joint",
#     "elbow_joint",
#     "wrist_1_joint",
#     "wrist_2_joint",
#     "wrist_3_joint",
# ]

# JOINT_TRAJECTORY_CONTROLLERS = [
#     "scaled_pos_joint_traj_controller",
#     "scaled_vel_joint_traj_controller",
#     "pos_joint_traj_controller",
#     "vel_joint_traj_controller",
#     "forward_joint_traj_controller",
# ]

# global joint_trajectory_controller, joint_position,joint_position_1
# global js_start, t_init, t_now,jp

# js_start = True
# jp = True
# t_init = 0
# t_now = 0

# joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[1]
# joint_position = None  # Initialize as None
# joint_position_1 = None

# def joint_callback(msg):
#     global joint_position,joint_position_1,jp
#     if jp == True:
#         joint_position_1 = msg.position
#         jp = False
#     joint_position = joint_position_1
#     joint_cartesian_trajectory() 



# def joint_cartesian_trajectory():
#     global joint_trajectory_controller, joint_position, JOINT_NAMES
#     global js_start, t_init, t_now

#     rospy.sleep(5)  # Wait for action server to start
#     goal = FollowJointTrajectoryGoal()
#     trajectory_client = actionlib.SimpleActionClient(
#         "{}/follow_joint_trajectory".format(joint_trajectory_controller),
#         FollowJointTrajectoryAction,
#     )
#     timeout = rospy.Duration(5)

#     if not trajectory_client.wait_for_server(timeout):
#         rospy.logerr("Could not reach controller action server.")
#         sys.exit(-1)

#     goal = FollowJointTrajectoryGoal()
#     goal.trajectory.joint_names = JOINT_NAMES

#     # Ensure that joint_position has been initialized by the callback
#     if joint_position is None or len(joint_position) != 6:
#         rospy.logerr("Joint position is not set correctly.")
#         sys.exit(-1)

#     # Initialize positions and velocities lists
#     #position_list = list(joint_position)
#     position_list = list(joint_position)
#     velocity_list = [0.0,0.0,-0.0,0.0,0.0,0.0]

#     # Modify the position and velocity for the joint you want to move (e.g., wrist_2_joint)
#     joint_index = 0 # Index for wrist_2_joint in JOINT_NAMES
#     if js_start:
#         js_start = False
#         t_init = time.time()

#     t_now = time.time() - t_init
#     print(t_now)

#     # Example: Moving wrist_2_joint with a certain velocity
#     velocity_list[joint_index] = 0.2  # Set desired velocity for the joint
#     duration_list = [3.0, 7.0, 10.0]
    
#     for duration in duration_list:
#         point = JointTrajectoryPoint()
#         #point.positions = position_list  # Maintain current positions
#         point.positions = joint_position
#         point.velocities = velocity_list  # Apply the velocity
#         point.time_from_start = rospy.Duration(duration)
#         goal.trajectory.points.append(point)

#     trajectory_client.send_goal(goal)
#     trajectory_client.wait_for_result()


# def main():
#     rospy.init_node('velocity_ctrl', anonymous=True)
#     rospy.Subscriber('/joint_states', JointState, joint_callback)
#     #rospy.sleep(1)  # Wait a bit to ensure the joint callback has been called
#     #joint_cartesian_trajectory()
#     rospy.spin()

# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

########################################   Velocity Ctrl through topic

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


global pub
pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)


def joint_callback(msg):
    global pub

    joint_position = msg.position
    print("joint_position", joint_position)

    # Define the desired velocity
    velocity = Float64MultiArray(data=[-0.1,0.00,0.0,0.0,0.0,0.0])
    #velocity = Float64MultiArray()
    # velocity.data = [-0.01,-0.0,0.0,0.0,0.0,0.0]
    print(velocity)
    # Publish the velocity command
    pub.publish(velocity)


def main():
    rospy.init_node('velocity_control', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_callback)

    print("main")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

################################################

# #!/usr/bin/env python

# def main():
#     rospy.init_node('velocity_publisher', anonymous=True)
#     pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
#     rospy.sleep(1)  # Wait for the publisher to connect

#     rate = rospy.Rate(10)  # 10 Hz
#     while not rospy.is_shutdown():
#         vel_msg = Float64MultiArray()
#         vel_msg.data = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]  # Set velocity for one joint
        
#         pub.publish(vel_msg)
#         rate.sleep()

# if __name__ == "__main__":
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass



