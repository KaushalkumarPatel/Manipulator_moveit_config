#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()


arm_group = moveit_commander.MoveGroupCommander("arm")
#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
#arm_group.set_named_target("home")
joint_goal = arm_group.get_current_joint_values()
joint_goal_1 = arm_group.get_current_joint_values()

joint_goal[0] = joint_goal_1[0]
joint_goal[1] = joint_goal_1[1]
joint_goal[2] = joint_goal_1[2]
joint_goal[3] = joint_goal_1[3]
joint_goal[4] = 0.575
joint_goal[5] = 1.6

print joint_goal
arm_group.go(joint_goal, wait=True)

#gripper_group = moveit_commander.MoveGroupCommander("gripper")
#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
#gripper_group.set_named_target("open")
#plan2 = gripper_group.go()






rospy.sleep(5)
moveit_commander.roscpp_shutdown()
