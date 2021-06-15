import math
import numpy
import sys
import CRCLLib


class URTranslator():
    ## This class provides methodes to parse and translate CRCL Commands.
    # PyXB was used to create the parser.

    def __init__(self, rob, CommandStatus):
        self.rob = rob  # < the Robot object
        self.CommandStatus = CommandStatus
        self.lengthUnit = "meter"
        self.angleUnit = "radian"
        self.forceUnit = "newton"
        self.rotSpeed = 2.0
        self.rotAccel = 0.3
        self.transSpeed = 0.05
        self.transAccel = 0.2
        self.radius = 0.0

    def parse(self, xml):
        command = None
        program = None

        try:
            command = CRCLLib.crclCommandInstance.CreateFromDocument(xml)
        except:
            try:
                program = CRCLLib.crclProgramInstance.CreateFromDocument(xml)
            except:
                print("Parsing failed")
        if program != None:
            return program, "program"
            # self.translate_program(program)
        if command != None:
            return command, "command"
            # self.translate_command(command.CRCLCommand)

    def translate_program(self, program):
        """ Translates a CRCL program into URScript commands and sends them to
            the robot.
        """
        for command in program.MiddleCommand:
            if isinstance(command, CRCLLib.crclProgramInstance.MoveToType):
                self.move_to(command)
            elif isinstance(command, CRCLLib.crclProgramInstance.MoveThroughToType):
                self.move_throught_to(command)
            elif isinstance(command, CRCLLib.crclProgramInstance.SetEndEffectorType):
                self.set_end_effector(command)
            elif isinstance(command, CRCLLib.crclProgramInstance.SetLengthUnitsType):
                self.set_length_units(command)
            elif isinstance(command, CRCLLib.crclProgramInstance.SetTransSpeedType):
                self.set_trans_speed(command)
            elif isinstance(command, CRCLLib.crclProgramInstance.SetTransAccelType):
                self.set_trans_accel(command)
            elif isinstance(command, CRCLLib.crclProgramInstance.SetRotSpeedType):
                self.set_rot_speed(command)
            elif isinstance(command, CRCLLib.crclProgramInstance.SetRotAccelType):
                self.set_rot_accel(command)
            elif isinstance(command, CRCLLib.crclProgramInstance.SetIntermediatePoseToleranceType):
                self.set_intermediate_pose_tolerance(command)
            elif isinstance(command, CRCLLib.crclProgramInstance.ActuateJointsType):
                self.actuate_joints(command)
            else:
                print("Unknown command!")

    def translate_command(self, command):
        """ Translates a single CRCL commands into URScript commands and sends them to
            the robot.
        """
        if isinstance(command, CRCLLib.crclCommandInstance.MoveToType):
            self.move_to(command)
        elif isinstance(command, CRCLLib.crclCommandInstance.MoveThroughToType):
            self.move_throught_to(command)
        elif isinstance(command, CRCLLib.crclCommandInstance.SetEndEffectorType):
            self.set_end_effector(command)
        elif isinstance(command, CRCLLib.crclCommandInstance.SetLengthUnitsType):
            self.set_length_units(command)
        elif isinstance(command, CRCLLib.crclCommandInstance.SetTransSpeedType):
            self.set_trans_speed(command)
        elif isinstance(command, CRCLLib.crclCommandInstance.SetTransAccelType):
            self.set_trans_accel(command)
        elif isinstance(command, CRCLLib.crclCommandInstance.SetRotSpeedType):
            self.set_rot_speed(command)
        elif isinstance(command, CRCLLib.crclCommandInstance.SetRotAccelType):
            self.set_rot_accel(command)
        elif isinstance(command, CRCLLib.crclCommandInstance.SetIntermediatePoseToleranceType):
            self.set_intermediate_pose_tolerance(command)
        elif isinstance(command, CRCLLib.crclCommandInstance.ActuateJointsType):
            self.actuate_joints(command)
        else:
            print("Unknown command!")

    def set_length_units(self, data):
        self.lengthUnit = data.UnitName
        self.CommandStatus.sendCommandStatus(data.CommandID, "CRCL_Done")

    def set_trans_speed(self, data):
        try:
            self.transSpeed = data.TransSpeed.Setting
        except:
            self.transSpeed = self.rob.max_speed * data.TransSpeed.Fraction
        self.CommandStatus.sendCommandStatus(data.CommandID, "CRCL_Done")

    def set_trans_accel(self, data):
        try:
            self.transAccel = data.TransAccel.Setting
        except:
            self.transAccel = (self.rob.max_accel * data.TransAccel.Fraction)
        self.CommandStatus.sendCommandStatus(data.CommandID, "CRCL_Done")

    def set_rot_speed(self, data):
        try:
            self.rotSpeed = data.RotSpeed.Setting
        except:
            self.rotSpeed = self.rob.max_rot_speed * data.rotSpeed.Fraction
        self.CommandStatus.sendCommandStatus(data.CommandID, "CRCL_Done")

    def set_rot_accel(self, data):
        try:
            self.rotAccel = data.RotAccel.Setting
        except:
            self.rotAccel = (self.rob.max_rot_accel * data.RotAccel.Fraction)
        self.CommandStatus.sendCommandStatus(data.CommandID, "CRCL_Done")

    def set_intermediate_pose_tolerance(self, data):

        self.radius = (
                                  data.Tolerance.XPointTolerance + data.Tolerance.YPointTolerance + data.Tolerance.ZPointTolerance) / 3
        self.CommandStatus.sendCommandStatus(data.CommandID, "CRCL_Done")

    def move_throught_to(self, data):
        x = 1
        id = data.CommandID
        if data.MoveStraight == True:
            for waypoint in data.Waypoint:
                pose = self.get_pose(waypoint)
                if x == data.NumPositions:
                    self.rob.movel(id, pose, self.transAccel, self.transSpeed, 0)
                else:
                    self.rob.movel(id, pose, self.transAccel, self.transSpeed, self.radius)
                x += 1
        else:
            for waypoint in data.Waypoint:
                pose = self.get_pose(waypoint)
                if x == data.NumPositions:
                    self.rob.movej_p(id, pose, self.rotAccel, self.rotSpeed, 0, 0)
                else:
                    self.rob.movej_p(id, pose, self.rotAccel, self.rotSpeed, 0, self.radius)
                x += 1

    def move_to(self, data):
        pose = self.get_pose(data.EndPosition)
        id = data.CommandID
        if data.MoveStraight == True:
            self.rob.movel(id, pose, self.transAccel, self.transSpeed, 0)
        else:
            self.rob.movej_p(id, pose, self.rotAccel, self.rotSpeed, 0, 0)

    def move_screw(self, data):
        # Not supported by UR5
        pass

    def set_end_effector(self, data):
        id = data.CommandID
        if data.Setting == 0:
            self.rob.close_gripper(id)
        elif data.Setting == 1:
            self.rob.open_gripper(id)
        else:
            print("Only 1 (open) or 0 (close) are accepted values")

    def actuate_joints(self, data):
        pass

        # =========================================================
        #                   Status Report
        # =========================================================

    def report_tcp(self):
        return rob.get_actual_tcp()

    def report_status(self):
        pass

        # =========================================================
        #                   Helper functions
        # =========================================================

    def get_pose(self, waypoint):
        """ Convets a CRCL waypoint into a pose.
            Return value: pose as array
        """
        pose = [0, 0, 0, 0, 0, 0]
        xaxis = numpy.array([waypoint.XAxis.I, waypoint.XAxis.J, waypoint.XAxis.K])
        zaxis = numpy.array([waypoint.ZAxis.I, waypoint.ZAxis.J, waypoint.ZAxis.K])
        yaxis = numpy.cross(zaxis, xaxis)

        matrix = numpy.matrix([xaxis, yaxis, zaxis])

        pose[3], pose[5], pose[4] = CRCLLib.Transform.euler_from_matrix(matrix, "sxzy")

        pose[0] = waypoint.Point.X
        pose[1] = waypoint.Point.Y
        pose[2] = waypoint.Point.Z

        return pose

    def parse_waypoint(self, position):
        point = CRCLLib.PointType(float(position["Point"]["X"]), float(position["Point"]["Y"]),
                                  float(position["Point"]["Z"]))
        xaxis = CRCLLib.VectorType(float(position["XAxis"]["I"]), float(position["XAxis"]["J"]),
                                   float(position["XAxis"]["K"]))
        yaxis = CRCLLib.VectorType(float(position["ZAxis"]["I"]), float(position["ZAxis"]["J"]),
                                   float(position["ZAxis"]["K"]))
        waypoint = CRCLLib.PoseType(point, xaxis, yaxis)
        return waypoint
