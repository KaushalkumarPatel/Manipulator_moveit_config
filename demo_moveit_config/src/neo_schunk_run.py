#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import math
import time
import pcl
import numpy as np
import ctypes
import struct
from array import *
from decimal import Decimal
import sensor_msgs.point_cloud2 as pc2
import geometry_msgs.msg
import moveit_msgs.msg

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Solve_traj:

    # initialise your node, publisher and subscriber as well as some member variables
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        # initialize node
        rospy.init_node('Neo_traj')

        self.rate = rospy.Rate(10)

        # define subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/extract_plane_indices/output', PointCloud2, self.points_callback)
        rospy.Subscriber('/sick_s300_front/scan', LaserScan , self.scan_callback)

        # define publishers
        self.velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 5)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group1 = moveit_commander.MoveGroupCommander("Arm")
        self.group2 = moveit_commander.MoveGroupCommander("gripper")


        self.msg = PointCloud2()
        # declare member variables
        self.vel = Twist()
        self.front_dist = 0.0
        self.laser_data = []
        self.abc = []
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0
        self.odom_rx = 0.0
        self.odom_ry = 0.0
        self.odom_rz = 0.0
        self.odom_rw = 0.0
        self.tag = 'True'
        self.list_p = []
        self.xa = 0.0
        self.ya = 0.0
        self.za = 0.0

        # in the callback functions, the received data is saved to the member variables
    def scan_callback(self, data):
        self.size = len(data.ranges)
        self.front_dist = data.ranges[self.size/2]
        self.laser_data = data.ranges
        self.min_value = min(self.laser_data)

    def points_callback(self, data):
        assert isinstance(data, PointCloud2)
        self.ros_cl = data

    def odom_callback(self, data):
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y
        self.odom_z = data.pose.pose.position.z
        self.odom_rx = data.pose.pose.orientation.x
        self.odom_ry = data.pose.pose.orientation.y
        self.odom_rz = data.pose.pose.orientation.z
        self.odom_rw = data.pose.pose.orientation.w

    def calc_dist(self,x1,y1,z1,x2,y2,z2):
        distance = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**0.5
        return distance

    def calc_cent(self, points):
        P = points
        n = len(P)
        #dist = []
        a = []
        #d = 0
        #xb = self.odom_x
        #yb = self.odom_y
        #zb = self.odom_z
        a = [sum(x) for x in zip(*P)]
        print a
        self.xa = 3*a[0]/n
        self.ya = 3*a[1]/n
        self.za = 3*a[2]/n
        print self.xa, self.ya, self.za
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = self.xa
        pose_target.position.y = self.ya
        pose_target.position.z = self.za
        pose_target.orientation.x = 0.726358736478
        pose_target.orientation.y = -0.056727466543
        pose_target.orientation.z = -0.055606618864
        pose_target.orientation.w = 0.682709956292
        self.group1.set_pose_target(pose_target)
        plan1 = self.group1.plan()
        go1 = self.group1.go(wait=True)
        print go1
        print 1
        print "Current Pose:"
        print self.group1.get_current_pose()
        pose_bot = self.group1.get_current_pose().pose
        print pose_bot
        #self.group1.stop()
        #self.group1.clear_pose_targets()
        q = pose_bot.position.y
        print "Q = "
        print q
        #r = round(x,2)
        if q >= 0.28:
            self.group1.stop()
            self.group2.set_named_target("close")
            go2 = self.group2.go(wait=True)
            print go2
            if go2 == 'False':
                #self.group1.stop()
                self.group2.stop()
                print "Move back"
                self.publish_vel(-0.5, 0.0)
            elif go2 == 'True':
                self.group2.set_named_target("close")
                go2 = self.group2.go(wait=False)

        #for i in P:
            #for j in i:
                #a.append(j)
                #d = self.calc_dist(xb,yb,zb,a[0],a[1],a[2])
                #dist.append(d)
        #print dist

    def ros_to_pcl(self, cl):
        points_list = []
        gen = pc2.read_points(cl, field_names = ("x", "y", "z"), skip_nans=True)
        print type(gen)
        for p in gen:
            points_list.append(p)
        #for data in pc2.read_points(ros_cl, field_names = ("x", "y", "z"), skip_nans=True):
            #points_list.append(data)
        #pcl_data = pcl.PointCloud_PointXYZRGB()
        #pcl_data.from_list(points_list)
        #print len(points_list)
        self.tag = 'False'
        return points_list

        # function to publish velocity
    def publish_vel(self, lx, az):
        self.vel.linear.x = lx
        self.vel.angular.z = az
        self.velPub.publish(self.vel)

    def go_to_pose(self):
        self.group2.set_named_target("open")
        go = self.group2.go(wait=True)

        #function to check if laser data is available
    def check_data(self):
        while self.laser_data != []:
            break
        else:
            time.sleep(2)

        # MazeSolver algorithm
    def startTraj(self):

        rospy.loginfo("Start")

        self.check_data()
        #self.go_to_pose()

        while not rospy.is_shutdown():

            if self.front_dist > 0.51:
                rospy.loginfo("Moving towards door")
                self.publish_vel(0.5, 0.0)
            else:
                rospy.loginfo("Stop before door")
                self.publish_vel(0.0, 0.0)
                if self.tag == 'True':
                    self.list_p = self.ros_to_pcl(self.ros_cl)
                else:
                    self.calc_cent(self.list_p)

            #sleep to prevent flooding your console
            self.rate.sleep()

        rospy.spin()

#main function outside your class that is called when the script is executed
if __name__ == "__main__":
        # instantiates your class and calls the __init__ function
        neo_runner = Solve_traj()

        # Starting  after instantiation
        neo_runner.startTraj()
