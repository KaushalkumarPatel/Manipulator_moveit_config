#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
scale = 1

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
group = moveit_commander.MoveGroupCommander("arm")

wpose_1 = group.get_current_pose().pose
wpose_2 = group.get_current_pose().pose
print wpose_1

waypoints = []
wpose_1.position.x -= scale * 0.38 # Second move forward/backwards in (x)
wpose_1.position.y = wpose_2.position.y
wpose_1.position.z = wpose_2.position.z
print wpose_1
waypoints.append(copy.deepcopy(wpose_1))

(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

group.execute(plan, wait=True)
