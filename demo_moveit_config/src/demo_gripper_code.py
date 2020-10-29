#!/usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
from moveit_msgs.msg import Grasp
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from math import pi
success=False
scale=1

rospy.init_node('scene_test', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


group = moveit_commander.MoveGroupCommander("gripper")
group.set_named_target("close")
plan1 = group.go()


hand_group = moveit_commander.MoveGroupCommander("gripper")
hand_group.set_named_target("open")
plan2 = hand_group.go()


"""group = moveit_commander.MoveGroupCommander("arm")


joint_goal = group.get_current_joint_values()
joint_goal_1 = group.get_current_joint_values()

joint_goal[0] = joint_goal_1[0]
joint_goal[1] = joint_goal_1[1]
joint_goal[2] = joint_goal_1[2]
joint_goal[3] = joint_goal_1[3]
joint_goal[4] = 0.6
joint_goal[5] = joint_goal_1[5]

print joint_goal
group.go(joint_goal, wait=True)

rospy.sleep(5)"""
moveit_commander.roscpp_shutdown()
