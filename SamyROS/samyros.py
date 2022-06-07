#!/usr/bin/env python3
import time
import sys
import copy
import os
os.environ["ROS_NAMESPACE"] = "/dsr01h2017"
import rospy
import roslaunch
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from pytransform3d import rotations
from pytransform3d import transformations
import numpy

from moveit_commander.conversions import pose_to_list
from pubsub import pub
from samyplugin.CRCL_DataTypes import *

import json
import ast
import copy


###class SamyRos###
###used to connect ROS with Moveit with the SamyCore###

class Samyros:
###Init###
    def __init__(self,robot_settings):

        rospy.init_node("main", anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("arm")

        #Subscribe
        pub.subscribe(self.move_to, "MoveTo")
        pub.subscribe(self.GetStatus, "GetStatus")
        
        #Debug
        self.cp = self.move_group.get_current_pose().pose
        print("Info: initial pose ")
        print("x: ",self.cp.position.x)
        print("y: ",self.cp.position.y)
        print("z: ",self.cp.position.z)
        print("qx: ",self.cp.orientation.x)
        print("qy: ",self.cp.orientation.y)
        print("qz: ",self.cp.orientation.z)
        print("qw: ",self.cp.orientation.w)


###CRCL functions###
    def move_to(self,data):
        if data.MoveStraight:
            self.movel(data)
        else:
            self.movej(data)

    def GetStatus(self,data):
        crcldata = CRCL_PoseDataType()
        self.cp = self.move_group.get_current_pose().pose
        try:
            q = numpy.array([self.cp.orientation.w,self.cp.orientation.x,self.cp.orientation.y,self.cp.orientation.z])
            rot = rotations.matrix_from_quaternion(q)
            crcldata.xAxis.i = rot[0,0]
            crcldata.xAxis.j = rot[0,1]
            crcldata.xAxis.k = rot[0,2]
            crcldata.zAxis.i = rot[2,0]
            crcldata.zAxis.i = rot[2,1]
            crcldata.zAxis.i = rot[2,2]
            crcldata.point.x = self.cp.position.x
            crcldata.point.y = self.cp.position.y
            crcldata.point.z = self.cp.position.z
            pub.sendMessage("write_information_source", name="current_pose",  data=crcldata)
        except:
            print("Error: Pose-Callback failed!")


###Helper functions###
    def executePlan(self,plan):
        try:
            self.move_group.execute(plan, wait=True)
        except:
            print("")
        finally:
            self.move_group.stop()
            self.move_group.clear_pose_targets()


    def movej(self,data):
        pose_goal = geometry_msgs.msg.Pose()
        xaxis = numpy.array([data.EndPosition.xAxis.i, data.EndPosition.xAxis.j, data.EndPosition.xAxis.k])
        zaxis = numpy.array([data.EndPosition.zAxis.i, data.EndPosition.zAxis.j, data.EndPosition.zAxis.k])
        yaxis = numpy.cross(zaxis, xaxis)
        
        try:
            rot = rotations.matrix_from_two_vectors(xaxis,yaxis)
            qw,qx,qy,qz = rotations.quaternion_from_matrix(rot)

            pose_goal.orientation.x = qx
            pose_goal.orientation.y = qy
            pose_goal.orientation.z = qz
            pose_goal.orientation.w = qw
            pose_goal.position.x = data.EndPosition.point.x
            pose_goal.position.y = data.EndPosition.point.y
            pose_goal.position.z = data.EndPosition.point.z
            print("Info: Goal pose: ")
            print("x: ",pose_goal.position.x)
            print("y: ",pose_goal.position.y)
            print("z: ",pose_goal.position.z)
            print("qx: ",qx)
            print("qy: ",qy)
            print("qz: ",qz)
            print("qw: ",qw)
        except:
            print("Error: Calcualtion of the roation matrixes failed due to a mathematical problem!")
        try:
            self.move_group.set_pose_target(pose_goal)
        except:
            print("Error: Setting the goal pose failed!")


        try:
            plan = self.move_group.go(wait=True)
            self.executePlan(plan)
        except:
            print("Error: Planning for Move_j failed!")


    def movel(self,data):
        waypoints = []        
        wpose = geometry_msgs.msg.Pose()
        self.cp = self.move_group.get_current_pose().pose

        deltax = data.EndPosition.point.x - self.cp.position.x
        deltay = data.EndPosition.point.y - self.cp.position.y
        deltaz = data.EndPosition.point.z - self.cp.position.z

        wpose.position.x = self.cp.position.x + deltax
        wpose.position.y = self.cp.position.y + deltay
        wpose.position.z = self.cp.position.z + deltaz
        wpose.orientation.x = self.cp.orientation.x
        wpose.orientation.y = self.cp.orientation.y
        wpose.orientation.z = self.cp.orientation.z
        wpose.orientation.w = self.cp.orientation.w
        waypoints.append(copy.deepcopy(wpose))

        try:
            (plan, _) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  #waypoints, steps, jump treshold
            self.executePlan(plan)
        except:
            print("Error: Planning for Move_l failed!")
            
            
            
            
 
