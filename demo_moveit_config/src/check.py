#!/usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
# from moveit_commander.PlanningSceneInterface.move_group.setGoalTolerance import
from moveit_msgs.msg import Grasp
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from math import pi
success=False
scale=1
import tf
from tf.transformations import quaternion_from_euler


rospy.init_node('scene_test', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("arm")
#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


# create tf listener
listener = tf.TransformListener()

# pause 5.0 seconds for tf buffer to fill
rospy.loginfo("Node Initialized, sleeping...")
rospy.sleep(5.0)

# transform point
try:
    ((x,y,z), rot) = listener.lookupTransform('arm_base_link', 'door_handle_tf', rospy.Time(0))
    rospy.loginfo("Transform: (" + str(x) + "," + str(y) + "," + str(z) + "," + ")")
    rospy.loginfo(str(rot))
except (tf.LookupException, tf.ConnectivityException):
    rospy.logerr("Transform failed.")




p=PoseStamped()
p.header.frame_id='arm_base_link'
p.header.stamp=rospy.Time(0)


eef_link = group.get_end_effector_link()
#current_pose = group.get_current_pose()
#current_rpy = group.get_current_rpy()
print "============ End effector: %s" % eef_link

#k = group.get_current_pose()

p.pose.position.x= x - 0.2#+ k.position.x #- 0.2      #0.5
p.pose.position.y= y #+ k.position.y #+ 0.05          #-0.12
p.pose.position.z= z #+ k.position.z #+ 0.05    # 0.32


p.pose.orientation.x= -rot[0] #0.1
p.pose.orientation.y= -rot[1]
p.pose.orientation.z= rot[2]
p.pose.orientation.w= rot[3]
group.set_pose_target(p)

print p
#print current_pose
#print current_rpy
plan=group.plan()

if plan.joint_trajectory.points:
    group.go(wait=True)
