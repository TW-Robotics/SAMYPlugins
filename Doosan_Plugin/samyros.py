#!/usr/bin/env python3
import time
import math
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
        
        #Constraints
        self.constr = moveit_msgs.msg.Constraints()
        #self.joint_constr = moveit_msgs.msg.JointConstraint()        
        self.constr.name = "Joint_Constrains"
        
        
        
        #Constrain Joint 1
        jc1 = moveit_msgs.msg.JointConstraint()
        jc1.position = 0.0
        jc1.tolerance_above = math.pi
        jc1.tolerance_below = math.pi
        jc1.weight = 1.0
        jc1.joint_name = self.move_group.get_joints()[0]
        self.constr.joint_constraints.append(jc1)
        
        
        #Constrain Joint 2
        jc2 = moveit_msgs.msg.JointConstraint()
        jc2.position = 0.0
        jc2.tolerance_above = math.pi/2
        jc2.tolerance_below = math.pi/2
        jc2.weight = 1.0
        jc2.joint_name = self.move_group.get_joints()[1]
        self.constr.joint_constraints.append(jc2)
        
        #Constrain Joint 3
        jc3 = moveit_msgs.msg.JointConstraint()
        jc3.position = 0.0
        jc3.tolerance_above = math.pi/2
        jc3.tolerance_below = math.pi/2
        jc3.weight = 1.0
        jc3.joint_name = self.move_group.get_joints()[2]
        self.constr.joint_constraints.append(jc3)
        
        #Constrain Joint 4
        #jc4 = moveit_msgs.msg.JointConstraint()
        #jc4.position = 0.0
        #jc4.tolerance_above = math.pi
        #jc4.tolerance_below = math.pi
        #jc4.weight = 1.0
        #jc4.joint_name = self.move_group.get_joints()[3]
        #self.constr.joint_constraints.append(jc4)
        
        #Constrain Joint 5
        #jc5 = moveit_msgs.msg.JointConstraint()
        #jc5.position = 0.0
        #jc5.tolerance_above = math.pi
        #jc5.tolerance_below = math.pi
        #jc5.weight = 1.0
        #jc5.joint_name = self.move_group.get_joints()[4]
        #self.constr.joint_constraints.append(jc5)
        
        #Constrain Joint 6
        #jc6 = moveit_msgs.msg.JointConstraint()
        #jc6.position = 0.0
        #jc6.tolerance_above = math.pi
        #jc6.tolerance_below = math.pi
        #jc6.weight = 1.0
        #jc6.joint_name = self.move_group.get_joints()[5]
        #self.constr.joint_constraints.append(jc6)
        
        #Write Constrains to move_group
        self.move_group.set_path_constraints(self.constr)
        
                
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
        self.move_group.set_goal_tolerance(0.01)
        #self.move_group.allow_replanning(True)
        self.robot = moveit_commander.RobotCommander()
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")




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

    def movej(self,data):
        self.move_group.clear_pose_targets()
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
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        while(True):
            self.cp = self.move_group.get_current_pose().pose
            if((abs(pose_goal.position.x - self.cp.position.x)) < 0.01 and (abs(pose_goal.position.y - self.cp.position.y)) < 0.01 and (abs(pose_goal.position.z - self.cp.position.z)) < 0.01):
                break
            time.sleep(0.5)
        self.move_group.stop()
        self.move_group.clear_pose_targets()



    def movel(self,data):
        self.move_group.clear_pose_targets()
        waypoints = []
        wpose = geometry_msgs.msg.Pose()
        self.cp = self.move_group.get_current_pose().pose

        wpose.position.x = data.EndPosition.point.x
        wpose.position.y = data.EndPosition.point.y
        wpose.position.z = data.EndPosition.point.z
        wpose.orientation.x = self.cp.orientation.x
        wpose.orientation.y = self.cp.orientation.y
        wpose.orientation.z = self.cp.orientation.z
        wpose.orientation.w = self.cp.orientation.w
        waypoints.append(copy.deepcopy(wpose))
        self.move_group.set_start_state_to_current_state()

        plan, _ = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0, avoid_collisions = True)  #waypoints, steps, jump treshold
        print("Plan = {}".format(plan))
        self.move_group.execute(plan, wait=True)
        while(True):
            self.cp = self.move_group.get_current_pose().pose
            if((abs(wpose.position.x - self.cp.position.x)) < 0.01 and (abs(wpose.position.y - self.cp.position.y)) < 0.01 and (abs(wpose.position.z - self.cp.position.z)) < 0.01):
                break
            time.sleep(0.5)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        

