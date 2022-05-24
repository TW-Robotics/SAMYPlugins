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
import samyplugin.CRCL_DataTypes

import json
import ast
import copy




class Samyros:
    def __init__(self,robot_settings):

        rospy.init_node("main", anonymous=False)

        #parameters of movements
        w = 1.0
        x = 0.7
        y = 0.5
        z = 1.0

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.move_group = moveit_commander.MoveGroupCommander("arm")

        #Subscribe
        pub.subscribe(self.move_to, "MoveTo")


    def executePlan(self,plan):
        try:
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
        except:
            print("Ausführung fehlgeschlagen!")


    def move_to(self,data):
        if data.MoveStraight:
            self.movel(data)
        else:
            self.movej(data)

        
    
    def movej(self,data):
        # Convert CRCL pose to pytransform3d transform
        xaxis = numpy.array([data.EndPosition.xAxis.i, data.EndPosition.xAxis.j, data.EndPosition.xAxis.k])
        zaxis = numpy.array([data.EndPosition.zAxis.i, data.EndPosition.zAxis.j, data.EndPosition.zAxis.k])
        yaxis = numpy.cross(zaxis, xaxis)
        rot = rotations.matrix_from_two_vectors(xaxis,yaxis)
        qx,qy,qz,qw = rotations.quaternion_from_matrix(rot)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.orientation.w = qw
        pose_goal.position.x = data.EndPosition.point.x
        pose_goal.position.y = data.EndPosition.point.y
        pose_goal.position.z = data.EndPosition.point.z
        self.move_group.set_pose_target(pose_goal)
        
        try:
            plan = move_group.go(wait=True)
            executePlan(plan)
        except:
            print("Move_J Planung fehlgeschlagen")


    def movel(self,data):
        waypoints = []
        scale = 1 
        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        try:
            (plan, _) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  #waypoints, steps, jump treshold
        except:
            print("Move_L Plannung fehlgeschlagen")
            
        executePlan(plan)
    






#if __name__ == "__main__":


    
    #movej(w,x,y,z)
    #print("Sleeping for 5 seconds...")
    #time.sleep(5)
    #print("Awake again :) ")
    #movel()




