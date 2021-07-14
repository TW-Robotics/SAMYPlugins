from RobotLib import Robot
import sys
import plugin

if __name__ == "__main__":

    rob = Robot(sys.argv[1])  # 172.16.36.128 192.168.0.10

    #rob.start_mission("MoveToTestPose")
    plugin.message_command(rob, 1, "MoveToTestPose")
    #plugin.send_message("Mission1")
