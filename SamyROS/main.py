#!/usr/bin/env python3
import sys
import copy
import os
os.environ["ROS_NAMESPACE"] = "/dsr01h2017"
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list


def moveRobot(w,x,y,z):

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = w
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    move_group.set_pose_target(pose_goal)
    
    try:
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        move_group.execute(plan, wait=True)
    except:
        pass




if __name__ == "__main__":
    #parameters of movement
    w = 1.0
    x = 0.7
    y = 0.5
    z = 1.0 

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("main", anonymous=False)
    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander("arm")
    moveRobot(w,x,y,z)




