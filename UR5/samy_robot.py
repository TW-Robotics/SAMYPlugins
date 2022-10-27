from pubsub import pub
import yaml
import time
from urx import Robot
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
import math3d as m3d
import numpy
import logging

class RobotSettings:

    def __init__(self, name):
        self.max_speed = 5.0  # m/s
        self.max_accel = 2.0  # m/s²
        self.max_rot_speed = 3.14
        self.max_rot_accel = 3.14

        self.name = name
        self.lengthUnit = "meter"
        self.angleUnit = "radian"
        self.forceUnit = "newton"
        self.rotSpeed = 2.0
        self.rotAccel = 0.3
        self.transSpeed = 1
        self.transAccel = 0.2
        self.radius = 0.0
        self.workobject = [[-0.060, 0.400, 0.100],[3.14, -3.14, 0]]   # in mm and rad [[-110, 350, 250],[3.14, -3.14, 0]], [[-560, -40, -195], [0, 0, 0]] [[-0.0, 0.350, 0.200],[3.14, -3.14, 0]]
        self.workobject_m3d = m3d.Transform(m3d.Orientation.new_rotation_vector((
            self.workobject[1][0], self.workobject[1][1], self.workobject[1][2])),
            m3d.Vector(self.workobject[0][0], self.workobject[0][1], self.workobject[0][2]))
        #self.tcp = (0, 0, 0, 0, 0, 0)  # (x, y, z, rx, ry, rz)

    def read_global_settings(self, global_settings): # not used anymore
        workobject = global_settings["Workobject"]
        # Write the relevant settings into the RobotSettings member variables
        self.workobject = [[workobject[0], workobject[1], workobject[2]],[workobject[3], workobject[4], workobject[5]]]
        print(self.workobject)
        self.workobject_m3d = m3d.Transform(m3d.Orientation.new_rotation_vector((
            self.workobject[1][0], self.workobject[1][1], self.workobject[1][2])),
            m3d.Vector(self.workobject[0][0], self.workobject[0][1], self.workobject[0][2]))

class SAMY_Robot():
    def __init__(self, global_settings):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        log_handler = logging.StreamHandler()
        log_handler.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(log_handler)
        

        self.robot = Robot(global_settings["Address"])
        self.gripper = Robotiq_Two_Finger_Gripper(self.robot)
        self.gripper.init_gripper()
        self.robot_settings = RobotSettings(global_settings["Name"])
        self.global_settings = global_settings
        self.robot_settings.read_global_settings(global_settings)
        # Subscribe to Topics
        self.logger.info("Subscribing to topics")
        pub.subscribe(self.popup, "Message")
        pub.subscribe(self.move_to, "MoveTo")
        pub.subscribe(self.move_through_to, "MoveThroughTo")
        pub.subscribe(self.dwell, "Dwell")
        pub.subscribe(self.write_pose, "GetStatus")
        pub.subscribe(self.set_end_effector, "SetEndeffector")
        pub.subscribe(self.set_trans_accel, "SetTransAccel")
        pub.subscribe(self.set_trans_speed, "SetTransSpeed")
        #pub.subscribe(self.set_length_units, "SetLengthUnits")
        pub.subscribe(self.stop_robot, "StopMotion")

    def popup(self, data):
        self.robot.send_message(str(data.message))

    def move_to(self, data):
        self.logger.info("Got MoveTo")
        trans = self.crcl_pose_to_m3d_pose(data.endPosition)

        if data.moveStraight:
            command = "movel"
        else:
            command = "movej"
        self.robot.set_pose(trans, acc=self.robot_settings.transAccel, vel=self.robot_settings.transSpeed, wait=True, command=command, threshold=None)

    def move_through_to(self, data):
        self.logger.info("Not supported")

    def write_pose(self, data):
        self.logger.info("Writing pose to information source.")
        vars = []
        vars.append(True)
        vars.append(True)
        pub.sendMessage("write_information_source", name="CameraReady", data=vars)

    def dwell(self, data):
        """ Sleep for an amount of time.
            Input parameters:
                dwell_time: time the robot waits
        """
        line = "sleep({})\n".format(data.dwellTime)
        self.robot.send_program(line)

    def send_status(self, data):
        self.logger.info("Not supported")

    def set_end_effector(self, data):
        if data.setting.fraction == 0.0:
            #self.robot.open_gripper()
            self.gripper.open_gripper()
            #self.robot.send_program(self.gripper.ret_program_to_run())
            self.logger.info("Open Gripper")
        else:
            #self.robot.close_gripper()
            self.gripper.close_gripper()
            #self.robot.send_program(self.gripper.ret_program_to_run())
            self.logger.info("Close Gripper")
            if self.robot.holds_object():
                self.logger.info("Gripper is holding an Object.")
                #pub.sendMessage("write_information_source", name="GripperHoldsObject", data=True)
            else:
                self.logger.info("Gripper is empty. ")
                #pub.sendMessage("write_information_source", name="GripperHoldsObject", data=False)
                #pub.sendMessage("command_halt")
                #time.sleep(1)
                #pub.sendMessage("command_reset")

    def set_trans_accel(self, data):
        if data.transAccel.switchField == 1:
            self.robot_settings.transAccel = data.TransAccel.unionValue.setting
        else:
            self.logger.info("Setting transAccel as fraction is not supported")

    def set_trans_speed(self, data):
        if data.transSpeed.switchField == 1:
            self.robot_settings.transSpeed = data.transSpeed.unionValue.setting
        else:
            self.logger.info("Setting transSpeed as fraction is not supported")

    def stop_robot(self, data):
        self.robot.stop()


    def crcl_pose_to_m3d_pose(self, crcl_pose):
        xaxis = numpy.array([crcl_pose.xAxis.i, crcl_pose.xAxis.j, crcl_pose.xAxis.k])
        zaxis = numpy.array([crcl_pose.zAxis.i, crcl_pose.zAxis.j, crcl_pose.zAxis.k])
        trans = m3d.Transform()
        trans.pos.x = (crcl_pose.point.x)
        trans.pos.y = (crcl_pose.point.y)
        trans.pos.z = (crcl_pose.point.z)
        trans.orient.from_xz(xaxis, zaxis)

        new_pose = self.robot_settings.workobject_m3d * trans
        self.logger.info("New Pose after transformation")
        self.logger.info(new_pose)

        return new_pose
