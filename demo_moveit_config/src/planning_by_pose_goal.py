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
import tf
from tf.transformations import quaternion_from_euler


rospy.init_node('scene_test', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("arm")
#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
#group.set_planner_id("PRMkConfigDefault")
#group.set_planning_time(10)



# create tf listener
listener = tf.TransformListener()

# pause 5.0 seconds for tf buffer to fill
rospy.loginfo("Node Initialized, sleeping...")
rospy.sleep(5.0)

# transform point
try:
    ((x,y,z), rot) = listener.lookupTransform('arm_base_link', 'door_handle_tf', rospy.Time(0))
    rospy.loginfo("Transform: (" + str(x) + "," + str(y) + "," + str(z) + "," + ")")
    #rospy.loginfo("Transform: (" + str(rot[0] + "," + str(rot[1]) + "," + str(rot[2]) + "," + str(rot[3]) +  + ")")

except (tf.LookupException, tf.ConnectivityException):
    rospy.logerr("Transform failed.")



while not success:


    p=PoseStamped()
    p.header.frame_id='arm_base_link'
    p.header.stamp=rospy.Time(5.0)


    #eef_link = group.get_end_effector_link()
    #print "============ End effector: %s" % eef_link

    p.pose.position.x= x #+ k.position.x #- 0.2      #0.5
    p.pose.position.y= y #+ k.position.y #+ 0.05          #-0.12
    p.pose.position.z= z #+ k.position.z #+ 0.05    # 0.32


    #p.pose.orientation.x= rot[0]#-0.049
    #p.pose.orientation.y= rot[1]#0.012
    #p.pose.orientation.z= rot[2]#0.78
    #p.pose.orientation.w= rot[3]#0.622

    group.set_pose_target(p)
    print p
    plan=group.plan()
    group.go(wait=True)
    success=True




#joint_goal_3 = group.get_current_joint_values()
#print joint_goal_3
        # We can get the joint values from the group and adjust some of the values:
"""joint_goal = group.get_current_joint_values()
joint_goal_1 = group.get_current_joint_values()

joint_goal[0] = joint_goal_1[0]
joint_goal[1] = joint_goal_1[1]
joint_goal[2] = joint_goal_1[2]
joint_goal[3] = joint_goal_1[3]
joint_goal[4] = 0.54
joint_goal[5] = 1.6

print joint_goal
group.go(joint_goal, wait=True)

wpose_1 = group.get_current_pose().pose
wpose_2 = group.get_current_pose().pose
print wpose_1

waypoints = []
wpose_1.position.x += scale * 0.13 # Second move forward/backwards in (x)
wpose_1.position.y = wpose_2.position.y
wpose_1.position.z = wpose_2.position.z

waypoints.append(copy.deepcopy(wpose_1))
print waypoints
(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

group.execute(plan, wait=True)



hand_group = moveit_commander.MoveGroupCommander("gripper")
hand_group.set_named_target("close")
plan2 = hand_group.go()


"""
"""group = moveit_commander.MoveGroupCommander("arm")
joint_goal_3 = group.get_current_joint_values()
joint_goal_4 = group.get_current_joint_values()

joint_goal_3[0] = joint_goal_4[0]
joint_goal_3[1] = joint_goal_4[1]
joint_goal_3[2] = joint_goal_4[2]
joint_goal_3[3] = joint_goal_4[3]
joint_goal_3[4] = 0.6
joint_goal_3[5] = joint_goal_4[5]

#print joint_goal
group.go(joint_goal_3, wait=True)

moveit_commander.roscpp_shutdown()"""
