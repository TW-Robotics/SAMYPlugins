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
from time import sleep

import json
import ast
import copy

from dsr_msgs.msg import *
from dsr_msgs.srv import *
#from dsr_msgs import Robotiq2FOpen

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "h2017"
# __dsr__id = ""
# __dsr__model = ""

# __dsr__id = ROBOT_ID
# __dsr__model = ROBOT_MODEL
# from DSR_ROBOT import *



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
        #self.move_group.set_path_constraints(self.constr)
        
                
        #Subscribe
        pub.subscribe(self.move_to, "MoveTo")
        pub.subscribe(self.GetStatus, "GetStatus")
        pub.subscribe(self.SetGripper, "SetEndeffector")
        pub.subscribe(self.checkPalette, "EnableSensor")

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


    def SetGripper(self,data):
    	if(data.setting.fraction>0):
    	    self.close_gripper()
    	else:
    	    self.open_gripper()

#ROS Services laut: http://wiki.ros.org/doosan-robotics?action=AttachFile&do=get&target=%EC%9D%BC%EB%B0%98%EB%B9%84_Doosan_Robotics_ROS_Manual_v1.13_EN.pdf
#Greifer öffnen an Pin X
    def open_gripper(self):
        pin = 4
        #srv_robotiq_2f_open = rospy.ServiceProxy('/' + ROBOT_ID + ROBOT_MODEL + '/gripper/robotiq_2f_open', Robotiq2FOpen) 
        setToolopen = rospy.ServiceProxy('/dsr01h2017/io/set_digital_output', SetCtrlBoxDigitalOutput)
        try:
            setToolopen(pin,1)
        except rospy.ServiceException as exc:
            print("Greifer wurde nicht geöffnet! Grund: " + str(exc))
 
 #Greifer schließen an Pin X
    def close_gripper(self):
        pin = 4 
        setToolclose = rospy.ServiceProxy('/dsr01h2017/io/set_digital_output', SetCtrlBoxDigitalOutput)
        try:
            setToolclose(pin,0)
        except rospy.ServiceException as exc:
            print("Greifer wurde nicht geschlossen! Grund: " + str(exc))
                  
 #Lichtschranke auslesen an Pin X                 
    def get_lichtschranke(self):
        pin = 15
        getLichtschranke = rospy.ServiceProxy('/dsr01h2017/io/get_digital_input', GetCtrlBoxDigitalOutput)
        while(True):
            try:
                response = getLichtschranke(pin)
                wert = response.value
                print("Value Lichtschranke: " + str(response.value))
                if wert == 1:
                    sleep(0.1)
                    return wert == 1
            except rospy.ServiceException as exc:
                print("Der Wert der Lichtschranke konnte nicht ausgelesen werden! Grund: " + str(exc))
        
        
    def checkPalette(self, data):
        pin = 9
        wert = -1 
        getPaletteLinks = rospy.ServiceProxy('/dsr01h2017/io/get_digital_input', GetCtrlBoxDigitalInput) 
        try:
            response = getPaletteLinks(pin)
            wert = response.value
            print("Value checkPallet: " + str(response.value))
        except rospy.ServiceException as exc:
		        print("Der Wert der Palette Links konnte nicht ausgelesen werden! Grund: " + str(exc))
        palette_data = wert == 1
        try:
        	pub.sendMessage("write_information_source", name="LichtschrankePalletDoosan",  data=palette_data)
        except:
        	print("Palette Links konnt nicht ausgelesen werden!")
        

    def move_to(self, data):
        if data.moveStraight:
            self.movel(data)
        else:
            self.movej(data)

    def GetStatus(self, data):
        crcldata = CRCL_PoseDataType()
        lichtschranke_data = False
        self.cp = self.move_group.get_current_pose().pose
        try:
            q = numpy.array([self.cp.orientation.w,self.cp.orientation.x,self.cp.orientation.y,self.cp.orientation.z])
            rot = rotations.matrix_from_quaternion(q)
            crcldata.name = "currentPoseDoosan"
            crcldata.id = 1
            crcldata.point.name = "point"
            crcldata.xAxis.name = "xaxis"
            crcldata.zAxis.name = "zaxis"
            crcldata.xAxis.i = rot[0,0]
            crcldata.xAxis.j = rot[0,1]
            crcldata.xAxis.k = rot[0,2]
            crcldata.zAxis.i = rot[2,0]
            crcldata.zAxis.i = rot[2,1]
            crcldata.zAxis.i = rot[2,2]
            crcldata.point.x = self.cp.position.x
            crcldata.point.y = self.cp.position.y
            crcldata.point.z = self.cp.position.z
            lichtschranke_data = self.get_lichtschranke() # waits for a box to be available 
            pub.sendMessage("write_information_source", name="currentPoseDoosan",  data=crcldata)
            #pub.sendMessage("write_information_source", name="LichtschrankeDoosan",  data=lichtschranke_data)
        except:
            print("Error: Pose-Callback failed!")


###Helper functions###

    def movej(self,data):
        self.move_group.clear_pose_targets()
        pose_goal = geometry_msgs.msg.Pose()
        xaxis = numpy.array([data.endPosition.xAxis.i, data.endPosition.xAxis.j, data.endPosition.xAxis.k])
        zaxis = numpy.array([data.endPosition.zAxis.i, data.endPosition.zAxis.j, data.endPosition.zAxis.k])
        yaxis = numpy.cross(zaxis, xaxis)

        try:
            rot = rotations.matrix_from_two_vectors(xaxis,yaxis)
            qw,qx,qy,qz = rotations.quaternion_from_matrix(rot)


            pose_goal.orientation.x = qx
            pose_goal.orientation.y = qy
            pose_goal.orientation.z = qz
            pose_goal.orientation.w = qw
            pose_goal.position.x = data.endPosition.point.x
            pose_goal.position.y = data.endPosition.point.y
            pose_goal.position.z = data.endPosition.point.z
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



    def movel(self, data):
        self.move_group.clear_pose_targets()
        waypoints = []
        wpose = geometry_msgs.msg.Pose()
        self.cp = self.move_group.get_current_pose().pose

        wpose.position.x = data.endPosition.point.x
        wpose.position.y = data.endPosition.point.y
        wpose.position.z = data.endPosition.point.z
       
        xaxis = numpy.array([data.endPosition.xAxis.i, data.endPosition.xAxis.j, data.endPosition.xAxis.k])
        zaxis = numpy.array([data.endPosition.zAxis.i, data.endPosition.zAxis.j, data.endPosition.zAxis.k])
        yaxis = numpy.cross(zaxis, xaxis)
        rot = rotations.matrix_from_two_vectors(xaxis,yaxis)
        qw,qx,qy,qz = rotations.quaternion_from_matrix(rot)

        print("EULER Intrinsic ZYZ")
        print(rotations.intrinsic_euler_zyz_from_active_matrix(rot))
        print("!!!!!!!!!!!!!!!!!!!")

        wpose.orientation.x = qx
        wpose.orientation.y = qy
        wpose.orientation.z = qz
        wpose.orientation.w = qw

        waypoints.append(copy.deepcopy(wpose))
        self.move_group.set_start_state_to_current_state()

        plan, _ = self.move_group.compute_cartesian_path(waypoints, 0.1, 0.0, avoid_collisions = True)  #waypoints, steps, jump treshold
        print("Plan = {}".format(plan))
        self.move_group.execute(plan, wait=True)
        while(True):
            self.cp = self.move_group.get_current_pose().pose
            if((abs(wpose.position.x - self.cp.position.x)) < 0.01 and (abs(wpose.position.y - self.cp.position.y)) < 0.01 and (abs(wpose.position.z - self.cp.position.z)) < 0.01):
                break
            time.sleep(0.5)
        self.move_group.stop()
        self.move_group.clear_pose_targets()