
from RobotLib import Robot
import sys
import plugin
import math

if __name__ == "__main__":

    rob = Robot(sys.argv[1])  # 172.16.36.128 192.168.0.10
    a = 0.3
    v = 200
    #r = 0
    #tcp = (0,0,0,0,0,0)
    #rob.set_tcp(20,tcp)
    #pose = rob.get_actual_tcp()
    #pose = [71.5, 61.203, 0.3, 0, 0, 0]
    #pose1 = pose.copy()
    #pose2 = pose.copy()
    #pose1[0] -= 50
    #pose1[1] -= 50
    #pose1[2] += 10
    #pose2[0] -= 20
    #pose2[1] += 40
    #print(pose)
    #rob.movel(2, pose1.copy(), a, v)
    #rob.compare_pose(pose1[0], pose1[1], pose1[2])
    #rob.movel(2, pose2.copy(), a, v)
    #rob.compare_pose(pose2[0], pose2[1], pose2[2])
    #rob.movel(21, pose.copy(), a, v)
    #rob.compare_pose(pose[0], pose[1], pose[2])

    #rob.movec(3, pose1.copy(), pose2.copy(), a, v)
    #rob.compare_pose(pose2[0], pose2[1], pose2[2])

    plugin.moveto(rob, 10, True, -24, -420, 226, -1, 0, 0, 0, 0, -1)
    plugin.compare_pose(rob, -24, -420, 226)
    plugin.set_end_effector(rob, 20, 1)
    plugin.get_end_effector(rob, 30)
    plugin.set_end_effector(rob, 20, 0)
    plugin.get_end_effector(rob, 30)
    #rob.movel(20, pose, a, v)
    #plugin.compare_pose(rob, pose[0], pose[1], pose[2])
    #rob.movel(21, pose1, a, v)
    #plugin.compare_pose(rob, pose1[0], pose1[1], pose1[2])

    #pose[3] = 2
    #pose = rob.get_actual_tcp()
    #rob.wait()
    #rob.popup("Das ist ein test")
    #rob.movel(1, pose_0, a, v=40)
    #rob.wait()
    #rob.open_gripper(1)
    #rob.dwell(2, 2)
    #rob.open_gripper(3)
    #time.sleep(9)
    #rob.close_gripper(4)
    #rob.holds_object(5)
    #rob.movel(5, pose1, a, v=0.3)
    #rob.wait()
    #rob.shutdown()
    #rob.set_analog_output(0, 0)

    #th1.join()

    #rob.close()
