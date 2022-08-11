#!/usr/bin/env python3
import time
import sys
import copy
import os
#os.environ["ROS_NAMESPACE"] = "/dsr01h2017"
import rospy
import roslaunch
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from open_manipulator_msgs.msg import KinematicsPose, JointPosition
from open_manipulator_msgs.srv import *

from pytransform3d import rotations
from pytransform3d import transformations
import numpy
import logging

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

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        log_handler = logging.StreamHandler()
        log_handler.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(log_handler)
        self.current_pose = CRCL_PoseDataType()

        rospy.init_node("main", anonymous=False)
        rospy.Subscriber("/gripper/kinematics_pose", KinematicsPose, self.update_current_pose)

        #Subscribe
        pub.subscribe(self.move_to, "MoveTo")
        pub.subscribe(self.get_status, "GetStatus")
        pub.subscribe(self.set_end_effector, "SetEndeffector")


###CRCL functions###
    def move_to(self,data):
        if data.MoveStraight:
            self.logger.error("MoveL is not supported")
        else:
            self.movej(data)
            

    def get_status(self,data):
        self.logger.info("Got GetStatus command")
        pub.sendMessage("write_information_source", name="OpenManCurrentPose",  data=self.current_pose)


    def set_end_effector(self, data):
        self.logger.info("Got SetEndeffector command")
        goal_tool_control = rospy.ServiceProxy('goal_tool_control', SetJointPosition)
        joint_position = JointPosition()
        joint_position.joint_name = ['gripper']
        joint_position.position = [data.Setting.fraction]
        response = goal_tool_control("", joint_position, 1)
        

### Helper functions ###
    def update_current_pose(self, data):
        self.current_pose.point.x = data.pose.position.x
        self.current_pose.point.y = data.pose.position.y
        self.current_pose.point.z = data.pose.position.z

        q = numpy.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
        rot = rotations.matrix_from_quaternion(q)
        self.current_pose.xAxis.i = rot[0,0]
        self.current_pose.xAxis.j = rot[0,1]
        self.current_pose.xAxis.k = rot[0,2]
        self.current_pose.zAxis.i = rot[2,0]
        self.current_pose.zAxis.j = rot[2,1]
        self.current_pose.zAxis.k = rot[2,2]

    def movej(self,data):
        pose_goal = KinematicsPose()
        xaxis = numpy.array([data.EndPosition.xAxis.i, data.EndPosition.xAxis.j, data.EndPosition.xAxis.k])
        zaxis = numpy.array([data.EndPosition.zAxis.i, data.EndPosition.zAxis.j, data.EndPosition.zAxis.k])
        yaxis = numpy.cross(zaxis, xaxis)
        
        rot = rotations.matrix_from_two_vectors(xaxis,yaxis)
        qw,qx,qy,qz = rotations.quaternion_from_matrix(rot)

        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw
        pose_goal.pose.position.x = data.EndPosition.point.x
        pose_goal.pose.position.y = data.EndPosition.point.y
        pose_goal.pose.position.z = data.EndPosition.point.z

        set_kinematics_pose = rospy.ServiceProxy('goal_task_space_path', SetKinematicsPose)
        response = set_kinematics_pose("", "gripper", pose_goal, 4)

        print("Info: Goal pose: ")
        print("x: ",pose_goal.pose.position.x)
        print("y: ",pose_goal.pose.position.y)
        print("z: ",pose_goal.pose.position.z)
        print("qx: ",pose_goal.pose.orientation.x)
        print("qy: ",pose_goal.pose.orientation.y)
        print("qz: ",pose_goal.pose.orientation.z)
        print("qw: ",pose_goal.pose.orientation.w)

        # Call rosservice for open_manipulator

            
            
            
            
 
