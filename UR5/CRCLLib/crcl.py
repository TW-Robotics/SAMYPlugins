#!/usr/bin/python
from abc import ABC
from enum import Enum
from TransformLib import transform


# =========================================================
#                   Enum
# =========================================================


class AngleUnitEnumType(Enum):
    degree = 1
    radian = 2


class ForceUnitEnumType(Enum):
    newton = 1
    pound = 2
    ounce = 3


class LengthUnitEnumType(Enum):
    meter = 1
    millimeter = 2
    inch = 3


class TorqueUnitEnumType(Enum):
    newtonMeter = 1
    footPound = 2


class GuardLimitEnumType(Enum):
    OVER_MAX = 1
    UNDER_MAX = 2
    INCREASE_OVER_LIMIT = 3
    DECREASE_BEYOND_LIMIT = 4


class CommandStateEnumType(Enum):
    DONE = "CRCL_Done"
    ERROR = "CRCL_Error"
    WORKING = "CRCL_Working"
    READY = "CRCL_Ready"


class StopConditionEnumType(Enum):
    Immediate = 1
    Fast = 2
    Normal = 3


# =========================================================
#                   Data Types
# =========================================================


class DataThingType(ABC):

    def __init__(self, name=None):
        self.Name = name

    def __str__(self):
        if self.Name:
            return "Name= " + self.Name + "\n"
        return ""

    def run(self):
        # Put your code here
        print("Not supported by this Robot or not implemented yet!")
        pass


class PointType(DataThingType):

    def __init__(self, x: float = None, y: float = None, z: float = None, name: str = None):
        super(PointType, self).__init__(name)
        self.X = x
        self.Y = y
        self.Z = z

    def __str__(self):
        output = super().__str__()
        return output + ("X=" + str(self.X) + " Y=" + str(self.Y) + " Z=" + str(self.Z))

    def set(self, x, y, z, name=None):
        self.Name = name
        self.X = x
        self.Y = y
        self.Z = z


class VectorType(DataThingType):

    def __init__(self, i: float = None, j: float = None, k: float = None, name: str = None):
        super(VectorType, self).__init__(name)
        self.I = i
        self.J = j
        self.K = k

    def set(self, i, j, k, name=None):
        self.I = i
        self.J = j
        self.K = k
        self.Name = name

    def __str__(self):
        output = super().__str__()
        return output + "I=" + str(self.I) + " J=" + str(self.J) + " K=" + str(self.K)


class TwistType(DataThingType):

    def __init__(self, linear_velocity=VectorType(), angular_velocity=VectorType(), name=None):
        super().__init__(name)
        self.LinearVelocity = linear_velocity
        self.AngularVelocity = angular_velocity

    def set_angular(self, i, j, k):
        self.AngularVelocity.set(i, j, k)

    def set_linear(self, i, j, k):
        self.LinearVelocity.set(i, j, k)

    def __str__(self):
        output = super().__str__()
        output = output + "AngularVelocity: I=" + str(self.AngularVelocity.I) + " J=" + str(
            self.AngularVelocity.J) + " K=" + str(self.AngularVelocity.K)
        output = output + "\nLinearVelocity: I=" + str(self.LinearVelocity.I) + " J=" + str(
            self.LinearVelocity.J) + " K=" + str(self.LinearVelocity.K)
        return output


class WrenchType(DataThingType):

    def __init__(self, force=VectorType(), moment=VectorType(), name: str = None):
        super(WrenchType, self).__init__(name)
        self.Force = force
        self.Moment = moment

    def set_force(self, i, j, k):
        self.Force.set(i, j, k)

    def set_moment(self, i, j, k):
        self.Moment.set(i, j, k)

    def __str__(self):
        output = super().__str__()
        return output + "WrenchType:\n" + "Force= " + str(self.Force) + "\nMoment= " + str(self.Moment)


class PoseType(DataThingType):

    def __init__(self, point=PointType(), x_axis=VectorType(), z_axis=VectorType(), name: str = None):
        super(PoseType, self).__init__(name)
        self.Point = point
        self.XAxis = x_axis
        self.ZAxis = z_axis

    def set(self, point: PointType, xaxis: VectorType, zaxis: VectorType):
        self.Point = point
        self.XAxis = xaxis
        self.ZAxis = zaxis

    def set_point(self, x, y, z, name=None):
        self.Point = PointType(x, y, z, name)

    def set_x_axis(self, i, j, k):
        self.XAxis = VectorType(i, j, k)

    def set_z_axis(self, i, j, k):
        self.ZAxis = VectorType(i, j, k)

    def __str__(self):
        output = super().__str__()
        output = output + "Point: " + str(self.Point) + "\n"
        output = output + "XAxis: " + str(self.XAxis) + "\n"
        output = output + "ZAxis: " + str(self.ZAxis)
        return output


class ParameterSettingsType(DataThingType):

    def __init__(self, parameter_name=None, parameter_value=None, name: str = None):
        super(ParameterSettingsType, self).__init__(name)
        self.ParameterName = parameter_name
        self.ParameterValue = parameter_value

    def set(self, name, value):
        self.ParameterName = name
        self.ParameterValue = value

    def __str__(self):
        output = super().__str__()
        return output + "ParameterName=" + str(self.ParameterName) + "\nParameterValue=" + str(self.ParameterValue)


class TransSpeedType(DataThingType):

    def __init__(self, name: str = None):
        super(TransSpeedType, self).__init__(name)

    def __str__(self):
        return super().__str__()


class TransSpeedAbsoluteType(TransSpeedType):

    def __init__(self, setting: float = None, name: str = None):
        super(TransSpeedAbsoluteType, self).__init__(name)
        self.Setting = setting

    def set(self, setting):
        self.Setting = setting

    def __str__(self):
        output = super().__str__()
        return output + "Absolute Speed=" + str(self.Setting)


class TransSpeedRelativeType(TransSpeedType):

    def __init__(self, fraction: float = None, name: str = None):
        super(TransSpeedRelativeType, self).__init__(name)
        self.Fraction = fraction

    def set(self, fraction):
        self.Fraction = fraction

    def __str__(self):
        output = super().__str__()
        return output + "Relative Speed=" + str(self.Fraction)


class TransAccelType(DataThingType):

    def __init__(self, name: str = None):
        super(TransAccelType, self).__init__(name)

    def __str__(self):
        return super().__str__()


class TransAccelAbsoluteType(TransAccelType):

    def __init__(self, setting: float = None, name: str = None):
        super(TransAccelAbsoluteType, self).__init__(name)
        self.Setting = setting

    def set(self, setting):
        self.Setting = setting

    def __str__(self):
        output = super().__str__()
        return output + "Absolute Accel=" + str(self.Setting)


class TransAccelRelativeType(TransAccelType):

    def __init__(self, fraction: float = None, name: str = None):
        super(TransAccelRelativeType, self).__init__(name)
        self.Fraction = fraction

    def set(self, fraction):
        self.Fraction = fraction

    def __str__(self):
        output = super().__str__()
        return output + "Relative Accel=" + str(self.Fraction)


class RotSpeedType(DataThingType):

    def __init__(self, name: str = None):
        super(RotSpeedType, self).__init__(name)

    def __str__(self):
        return super().__str__()


class RotSpeedAbsoluteType(RotSpeedType):

    def __init__(self, setting: float = None, name: str = None):
        super(RotSpeedAbsoluteType, self).__init__(name)
        self.Setting = setting

    def set(self, setting):
        self.Setting = setting

    def __str__(self):
        output = super().__str__()
        return output + "Absolute RotSpeed=" + str(self.Setting)


class RotSpeedRelativeType(RotSpeedType):

    def __init__(self, fraction: float = None, name: str = None):
        super(RotSpeedRelativeType, self).__init__(name)
        self.Fraction = fraction

    def set(self, fraction):
        self.Fraction = fraction

    def __str__(self):
        output = super().__str__()
        return output + "Relative RotSpeed=" + str(self.Fraction)


class RotAccelType(DataThingType):

    def __init__(self, name: str = None):
        super(RotAccelType, self).__init__(name)

    def __str__(self):
        return super().__str__()


class RotAccelAbsoluteType(RotAccelType):

    def __init__(self, setting: float = None, name: str = None):
        super(RotAccelAbsoluteType, self).__init__(name)
        self.Setting = setting

    def set(self, setting):
        self.Setting = setting

    def __str__(self):
        output = super().__str__()
        return output + "Absolute RotAccel=" + str(self.Setting)


class RotAccelRelativeType(RotAccelType):

    def __init__(self, fraction: float = None, name: str = None):
        super(RotAccelRelativeType, self).__init__(name)
        self.Fraction = fraction

    def set(self, fraction):
        self.Fraction = fraction

    def __str__(self):
        output = super().__str__()
        return output + "Relative RotAccel=" + str(self.Fraction)


class PoseToleranceType(DataThingType):

    def __init__(self, x_point_tolerance: float = None, y_point_tolerance: float = None,
                 z_point_tolerance: float = None, x_axis_tolerance: float = None,
                 z_axis_tolerance: float = None, name: str = None):
        super(PoseToleranceType, self).__init__(name)
        self.XPointTolerance = x_point_tolerance
        self.YPointTolerance = y_point_tolerance
        self.ZPointTolerance = z_point_tolerance
        self.XAxisTolerance = x_axis_tolerance
        self.ZAxisTolerance = z_axis_tolerance

    def set(self, x_point_tolerance, y_point_tolerance, z_point_tolerance, x_axis_tolerance,
            z_axis_tolerance):
        self.XPointTolerance = x_point_tolerance
        self.YPointTolerance = y_point_tolerance
        self.ZPointTolerance = z_point_tolerance
        self.XAxisTolerance = x_axis_tolerance
        self.ZAxisTolerance = z_axis_tolerance

    def __str__(self):
        output = super().__str__()
        output = output + "XPoint Tolerance=" + str(self.XPointTolerance) + "\nYPoint Tolerance=" + str(
            self.YPointTolerance)
        output = output + "\nZPoint Tolerance=" + str(self.ZPointTolerance)
        output = output + "\nXAxis Tolerance=" + str(self.XAxisTolerance) + "\nZAxis Tolerance=" + str(
            self.ZAxisTolerance)
        return output


class PoseAndSetType(PoseType):

    def __init__(self, coordinated: bool = None, trans_speed=TransSpeedType(), rot_speed=RotSpeedType(),
                 trans_accel=TransAccelType(), rot_accel=RotAccelType(), tolerance=PoseToleranceType()):
        super().__init__()
        self.Coordinated = coordinated
        self.TransSpeed = trans_speed
        self.RotSpeed = rot_speed
        self.TransAccel = trans_accel
        self.RotAccel = rot_accel
        self.Tolerance = tolerance

    def __str__(self):
        output = super().__str__()
        output = output + "\nCoordinated=" + str(self.Coordinated)
        output = output + "\nTransSpeed=" + str(self.TransSpeed)
        output = output + "\nRotSpeed=" + str(self.RotSpeed)
        output = output + "\nTransAccel=" + str(self.TransAccel)
        output = output + "\nRotAccel=" + str(self.RotAccel)
        output = output + "\nTolerance=" + str(self.Tolerance)
        return output


class GuardType(DataThingType):

    def __init__(self, sensor_id=None, sub_field=None, limit_type: GuardLimitEnumType = None, limit_value: float = None,
                 recheck_time_micro_seconds: float = None, check_count: float = None, last_check_time: float = None,
                 last_check_value: float = None, name: str = None):
        super(GuardType, self).__init__(name)
        self.SensorID = sensor_id
        self.SubField = sub_field
        self.LimitType = limit_type
        self.LimitValue = limit_value
        self.RecheckTimeMicroSeconds = recheck_time_micro_seconds
        self.CheckCount = check_count
        self.LastCheckTime = last_check_time
        self.LastCheckValue = last_check_value

    def set(self, sensor_id, limit_type, limit_value):
        self.SensorID = sensor_id
        self.LimitType = limit_type
        self.LimitValue = limit_value

    def __str__(self):
        output = super().__str__()
        output = output + "SensorID= " + str(self.SensorID) + "\nSubField= " + str(
            self.SubField) + "\nLimitType= " + str(self.LimitType) + "\nLimitValue= " + str(
            self.LimitValue) + "\nRecheckTimeMicroSeconds= " + str(
            self.RecheckTimeMicroSeconds) + "\nCheckCount= " + str(self.CheckCount) + "\nLastCheckTime= " + str(
            self.LastCheckTime) + "\nLastCheckValue=" + str(self.LastCheckValue)
        return output

    # =========================================================
    #                   Commands
    # =========================================================


class CRCLCommandType(DataThingType):

    def __init__(self, command_id: int = None, name: str = None):
        super(CRCLCommandType, self).__init__(name)
        self.CommandID = command_id

    def __str__(self):
        output = super().__str__()
        return (output + "CommandID= " + str(self.CommandID))


class CRCLCommandInstanceType(DataThingType):

    def __init__(self, crcl_command=CRCLCommandType(), name=None):
        super(CRCLCommandInstanceType, self).__init__(name)
        self.CRCLCommand = crcl_command

    def set(self, crcl_command):
        self.CRCLCommand = crcl_command

    def __str__(self):
        output = super().__str__()
        return output + str(self.CRCLCommand)


class MiddleCommandType(CRCLCommandType):

    def __init__(self, command_id=None, name=None):
        super(MiddleCommandType, self).__init__(command_id, name)

    # def set(self, command_id):
    #    self.CommandID = command_id

    def __str__(self):
        return super().__str__()


class ActuateJointType(DataThingType):

    def __init__(self, joint_number=None, joint_position=None, joint_details=None, name=None):
        super(ActuateJointType, self).__init__(name)
        self.JointNumber = joint_number
        self.JointPosition = joint_position
        self.JointDetails = joint_details

    def set(self, joint_number, joint_position, joint_details):
        self.JointNumber = joint_number
        self.JointPosition = joint_position
        self.JointDetails = joint_details

    def __str__(self):
        output = super().__str__()
        return output + "JointNumber= " + str(self.JointNumber) + "\nJointPosition= " + str(
            self.JointPosition) + "\nJointDetails= " + str(self.JointDetails)


class ActuateJointsType(MiddleCommandType):

    def __init__(self, actuate_joint=[], command_id=None, name=None):
        super(ActuateJointsType, self).__init__(command_id, name)
        self.ActuateJoint = actuate_joint

    def add(self, actuate_joint=ActuateJointType()):
        self.ActuateJoint.append(actuate_joint)

    def __str__(self):
        output = super().__str__()
        for joint in self.ActuateJoint:
            output = output + "\nJoint: \n" + str(joint)
        return output

    def run(self, robot):
        # Put your code here
        print("Not supported by this Robot or not implemented yet!")
        pass


class CloseToolChangeType(MiddleCommandType):

    def __init__(self, command_id=None, name=None):
        super(CloseToolChangeType, self).__init__(command_id, name)

    def __str__(self):
        return super().__str__()


class JointDetailsType(DataThingType):

    def __init__(self, name=None):
        super(JointDetailsType, self).__init__(name)

    def __str__(self):
        return super().__str__()


class JointForceTorqueType(JointDetailsType):

    def __init__(self, setting=None, change_rate=None, name=None):
        super(JointForceTorqueType, self).__init__(name)
        self.Setting = setting
        self.ChangeRate = change_rate

    def set_setting(self, setting):
        self.Setting = setting

    def set_change_rate(self, change_rate):
        self.ChangeRate = change_rate

    def __str__(self):
        output = super().__str__()
        return output + "Setting= " + str(self.Setting) + "\n ChangeRate= " + str(self.ChangeRate)


class JointSpeedAccelType(JointDetailsType):

    def __init__(self, joint_speed=None, joint_accel=None, name=None):
        super(JointSpeedAccelType, self).__init__(name)
        self.JointSpeed = joint_speed
        self.JointAccel = joint_accel

    def set(self, joint_speed, joint_accel):
        self.JointSpeed = joint_speed
        self.JointAccel = joint_accel

    def __str__(self):
        output = super().__str__()
        output = output + "JointSpeed= " + str(self.JointSpeed) + "\nJointAccel= " + str(self.JointAccel)
        return output


class ConfigureJointReportType(DataThingType):

    def __init__(self, joint_number=None, report_position=None, report_torque_or_force=None, report_velocity=None,
                 name=None):
        super(ConfigureJointReportType, self).__init__(name)
        self.JointNumber = joint_number
        self.ReportPosition = report_position
        self.ReportTorqueOrForce = report_torque_or_force
        self.ReportVelocity = report_velocity

    def set(self, joint_number, report_position, report_torque_or_force, report_velocity):
        self.JointNumber = joint_number
        self.ReportPosition = report_position
        self.ReportTorqueOrForce = report_torque_or_force
        self.ReportVelocity = report_velocity

    def __str__(self):
        output = super().__str__()
        return output + "JointNumber= " + str(self.JointNumber) + "\nReportPosition= " + str(
            self.ReportPosition) + "\nReportTorqueOrForce= " + str(
            self.ReportTorqueOrForce) + "\nReportVelocity= " + str(self.ReportVelocity)


class ConfigureJointReportsType(MiddleCommandType):

    def __init__(self, reset_all=None, configure_joint_report=[], command_id=None, name=None):
        super(ConfigureJointReportsType, self).__init__(command_id, name)
        self.ResetAll = reset_all
        self.ConfigureJointReport = configure_joint_report

    def add(self, configure_joint_report=ConfigureJointReportType()):
        self.ConfigureJointReport.append(configure_joint_report)

    def __str__(self):
        output = super().__str__()
        for report in self.ConfigureJointReport:
            output = output + "\n" + str(report)
        return output


class DwellType(MiddleCommandType):

    def __init__(self, dwell_time=None, command_id=None, name=None):
        super(DwellType, self).__init__(command_id, name)
        self.DwellTime = dwell_time

    def set(self, dwell_time):
        self.DwellTime = dwell_time

    def __str__(self):
        output = super().__str__()
        return output + "DwellTime= " + str(self.DwellTime)


class MessageType(MiddleCommandType):

    def __init__(self, message=None, command_id=None, name=None):
        super(MessageType, self).__init__(command_id, name)
        self.Message = message

    def set(self, message):
        self.Message = message

    def __str__(self):
        output = super().__str__()
        output = output + "Message= " + str(self.Message)
        return output


class EndCanonType(CRCLCommandType):

    def __init__(self, command_id=None, name=None):
        super(EndCanonType, self).__init__(command_id, name)

    def __str__(self):
        return super().__str__()


class InitCanonType(CRCLCommandType):

    def __init__(self, command_id=None, name=None):
        super(InitCanonType, self).__init__(command_id, name)

    def __str__(self):
        return super().__str__()


class GetStatusType(MiddleCommandType):

    def __init__(self, command_id=None, name=None):
        super(GetStatusType, self).__init__(command_id, name)

    def __str__(self):
        return super().__str__()


class SetTransSpeedType(MiddleCommandType):

    def __init__(self, trans_speed=None, command_id=None, name=None):
        super(SetTransSpeedType, self).__init__(command_id, name)
        self.TransSpeed = trans_speed

    def set(self, trans_speed):
        self.TransSpeed = trans_speed

    def __str__(self):
        return super().__str__() + "\nTransSpeed= " + str(self.TransSpeed)

    def run(self, robot):
        # Specific robot Code goes here
        pass


class SetTransAccelType(MiddleCommandType):

    def __init__(self, trans_accel=None, command_id=None, name=None):
        super(SetTransAccelType, self).__init__(command_id, name)
        self.TransAccel = trans_accel

    def set(self, trans_accel):
        self.TransAccel = trans_accel

    def __str__(self):
        return super().__str__() + "\nTransAccel= " + str(type(self.TransAccel))

    def run(self, robot):
        # Specific robot Code goes here
        pass


class SetRotAccelType(MiddleCommandType):

    def __init__(self, rot_accel: RotAccelType = None, command_id=None, name=None):
        super(SetRotAccelType, self).__init__(command_id, name)
        self.RotAccel = rot_accel

    def set(self, rot_accel):
        self.RotAccel = rot_accel

    def __str__(self):
        return super().__str__() + "\nRotAccel:\n" + str(self.RotAccel)


class SetRotSpeedType(MiddleCommandType):

    def __init__(self, rot_speed: RotSpeedType = None, command_id=None, name=None):
        super(SetRotSpeedType, self).__init__(command_id, name)
        self.RotSpeed = rot_speed

    def set(self, rot_speed):
        self.RotSpeed = rot_speed

    def __str__(self):
        return super().__str__() + "\nRotSpeed:\n" + str(self.RotSpeed)


class SetTorqueUnitsType(MiddleCommandType):

    def __init__(self, unit_name: TorqueUnitEnumType = None, command_id=None, name=None):
        super(SetTorqueUnitsType, self).__init__(command_id, name)
        self.UnitName = unit_name

    def set(self, unit_name):
        self.UnitName = unit_name

    def __str__(self):
        return super().__str__() + "\nUnitName= " + TorqueUnitEnumType(self.UnitName).name


class SetEndPoseToleranceType(MiddleCommandType):

    def __init__(self, tolerance=None, command_id=None, name=None):
        super(SetEndPoseToleranceType, self).__init__(command_id, name)
        self.Tolerance = tolerance

    def set(self, tolerance):
        self.Tolerance = tolerance

    def __str__(self):
        return super().__str__() + "\nTolerance= " + str(self.Tolerance)

    def run(self, robot):
        # Specific robot Code goes here
        pass


class OpenToolChangerType(MiddleCommandType):

    def __init__(self, command_id=None, name=None):
        super(OpenToolChangerType, self).__init__(command_id, name)

    def __str__(self):
        return super().__str__()

    def run(self, robot):
        # Specific robot Code goes here
        pass


class SetForceUnitsType(MiddleCommandType):

    def __init__(self, unit_name: ForceUnitEnumType = 1, command_id=None, name=None):
        super(SetForceUnitsType, self).__init__(command_id, name)
        self.UnitName = unit_name

    def set(self, unit_name):
        self.UnitName = unit_name

    def __str__(self):
        return super().__str__() + "\nUnitName= " + str(self.UnitName)


class SetIntermediatePoseToleranceType(MiddleCommandType):

    def __init__(self, tolerance: PoseToleranceType = None, command_id=None, name=None):
        super(SetIntermediatePoseToleranceType, self).__init__(command_id, name)
        self.Tolerance = tolerance

    def set(self, tolerance):
        self.Tolerance = tolerance

    def __str__(self):
        return super().__str__() + "\nTolerance= " + str(self.Tolerance)


class SetLengthUnitsType(MiddleCommandType):

    def __init__(self, unit_name: LengthUnitEnumType = None, command_id=None, name=None):
        super(SetLengthUnitsType, self).__init__(command_id, name)
        self.UnitName = unit_name

    def set(self, unit_name):
        self.UnitName = unit_name

    def __str__(self):
        return super().__str__() + "\nUnitName= " + LengthUnitEnumType(self.UnitName).name


class SetMotionCoordinationType(MiddleCommandType):

    def __init__(self, coordinated=None, command_id=None, name=None):
        super(SetMotionCoordinationType, self).__init__(command_id, name)
        self.Coordinated = coordinated

    def set(self, coordinated):
        self.Coordinated = coordinated

    def __str__(self):
        return super().__str__() + "\nCoordinated= " + str(self.Coordinated)


class SetRobotParametersType(MiddleCommandType):

    def __init__(self, parameter_setting=[], command_id=None, name=None):
        super(SetRobotParametersType, self).__init__(command_id, name)
        self.ParameterSetting = parameter_setting

    def add(self, parameter_setting):
        self.ParameterSetting.append(parameter_setting)

    def __str__(self):
        output = super().__str__() + "\nParameterSetting:"
        for setting in self.ParameterSetting:
            output = output + "\n" + str(setting)
        return output


class CloseToolChangerType(MiddleCommandType):

    def __init__(self, command_id=None, name=None):
        super(CloseToolChangerType, self).__init__(command_id, name)

    def __str__(self):
        return super().__str__()

    def run(self, robot):
        # Specific robot Code goes here
        pass


class RunProgramType(MiddleCommandType):

    def __init__(self, program_text=None, command_id=None, name=None):
        super(RunProgramType, self).__init__(command_id, name)
        self.ProgramText = program_text

    def set(self, program_text):
        self.ProgramText = program_text

    def __str__(self):
        output = super().__str__()
        output = output + "\nProgramText= " + self.ProgramText
        return output


class StopMotionType(MiddleCommandType):

    def __init__(self, stop_condition: StopConditionEnumType = None, command_id=None, name=None):
        super(StopMotionType, self).__init__(command_id, name)
        self.StopCondition = stop_condition

    def set(self, stop_condition):
        self.StopCondition = stop_condition

    def __str__(self):
        return super().__str__() + "\nStopCondition= " + StopConditionEnumType(self.StopCondition).name


class MoveScrewType(MiddleCommandType):

    def __init__(self, start_position=PoseType(), axis_point=PointType(), axial_distance_free: float = None,
                 axial_distance_screw: float = None, turn: float = None, command_id: int = None, name: str = None):
        super(MoveScrewType, self).__init__(command_id, name)
        self.StartPosition = start_position
        self.AxisPoint = axis_point
        self.AxialDistanceFree = axial_distance_free
        self.AxialDistanceScrew = axial_distance_screw
        self.Turn = turn

    def set(self, axial_distance_screw, turn, start_position=None, axis_point=None, axial_distance_free=None):
        self.AxialDistanceScrew = axial_distance_screw
        self.Turn = turn
        self.StartPosition = start_position
        self.AxisPoint = axis_point
        self.AxialDistanceFree = axial_distance_free

    def __str__(self):
        output = super().__str__()
        output = output + "StartPosition= " + str(self.StartPosition) + "\nAxisPoint= " + str(
            self.AxisPoint) + "\n AxialDistanceFree= " + str(self.AxialDistanceFree) + "\n AxialDistanceScrew= " + str(
            self.AxialDistanceScrew) + "\nTurn= " + str(self.Turn)
        return output


class MoveThroughToType(MiddleCommandType):

    def __init__(self, move_straight=None, waypoint=[], command_id=None, name=None):
        super(MoveThroughToType, self).__init__(command_id, name)
        self.MoveStraight = move_straight
        self.Waypoint = waypoint
        self.NumPositions = len(self.Waypoint)
        self.Robot = None

    def set(self, move_straight):
        self.MoveStraight = move_straight

    def add(self, wp):
        try:
            self.Waypoint.append(wp)
        except:
            return False
        self.NumPositions = len(self.Waypoint)
        return True

    def __str__(self):
        output = super().__str__()
        output = output + "\nMoveStraight= " + str(self.MoveStraight) + "\nNumPositions= " + str(
            self.NumPositions) + "\nWaipoints:"
        for wp in self.Waypoint:
            output = output + "\n" + str(wp)
        return output

    def run(self):
        # Put your code here!
        pass


class MoveToType(MiddleCommandType):

    def __init__(self, move_straight: bool = None, end_position=PoseType(), command_id=None, name=None):
        super(MoveToType, self).__init__(command_id, name)
        self.MoveStraight = move_straight
        self.EndPosition = end_position
        self.Robot = None

    def set(self, move_straight: bool, end_position: PoseType):
        self.EndPosition = end_position
        self.MoveStraight = move_straight

    def __str__(self):
        output = super().__str__()
        output = output + "\nMoveStraight= " + str(self.MoveStraight) + "\nEndPosition:\n" + str(self.EndPosition)
        return output

    def run(self, robot):
        pose = transform.get_pose(self.EndPosition)
        if self.MoveStraight:
            self.Robot.movel(self.CommandID, pose, self.Robot.robot_setings.transAccel, self.Robot.robot_setings.transSpeed, 0)
        else:
            self.Robot.movej_p(self.CommandID, pose, self.Robot.robot_settings.rotAccel, self.Robot.robot_settings.rotSpeed, 0, 0)


class SetAngleUnitsType(MiddleCommandType):

    def __init__(self, unit_name: AngleUnitEnumType = "radian", command_id=None, name=None):
        super(SetAngleUnitsType, self).__init__(command_id, name)
        self.UnitName = unit_name

    def set(self, unit_name):
        self.UnitName = unit_name

    def __str__(self):
        return super().__str__() + "\nUnitName= " + AngleUnitEnumType(self.UnitName).name

    def run(self):
        # Put your code here!
        pass


class ConfigureStatusReportType(MiddleCommandType):

    def __init__(self, report_joint_statuses=None, report_pose_status=None, report_gripper_status=None,
                 report_settings_status=None, report_sensors_status=None, report_guards_status=None, command_id=None,
                 name=None):
        super(ConfigureStatusReportType, self).__init__(command_id, name)
        self.ReportJointStatuses = report_joint_statuses
        self.ReportPoseStatus = report_pose_status
        self.ReportGripperStatus = report_gripper_status
        self.ReportSettingsStatus = report_settings_status
        self.ReportSensorsStatus = report_sensors_status
        self.ReportGuardsStatus = report_guards_status

    def set(self, report_joint_statuses, report_pose_status, report_gripper_status, report_settings_status,
            report_sensors_status, report_guards_status):
        self.ReportJointStatuses = report_joint_statuses
        self.ReportPoseStatus = report_pose_status
        self.ReportGripperStatus = report_gripper_status
        self.ReportSettingsStatus = report_settings_status
        self.ReportSensorsStatus = report_sensors_status
        self.ReportGuardsStatus = report_guards_status

    def __str__(self):
        return super().__str__() + "\nReportJointStatuses= " + str(
            self.ReportJointStatuses) + "\nReportPoseStatus= " + str(
            self.ReportPoseStatus) + "\nRiportGripperStatus= " + str(
            self.ReportGripperStatus) + "\nReportSettingsStatus= " + str(
            self.ReportSettingsStatus) + "\nReportSensorsStatus= " + str(
            self.ReportSensorsStatus) + "\nReportGuardsStatus= " + str(self.ReportGuardsStatus)

    def run(self):
        # Todo
        pass


class EnableSensorType(MiddleCommandType):

    def __init__(self, sensor_id=None, sensor_option=[], command_id=None, name=None):
        super(EnableSensorType, self).__init__(command_id, name)
        self.SensorID = sensor_id
        self.SensorOption = sensor_option

    def add(self, sensor_option):
        self.SensorOption.append(sensor_option)

    def set(self, sensor_id):
        self.SensorID = sensor_id

    def __str__(self):
        output = super().__str__() + "\nSensorID= " + str(self.SensorID)
        for option in self.SensorOption:
            output = output + "\nSensorOption:\n" + str(option)
        return output

    def run(self):
        # Put your code here!
        pass


class DisableSensorType(MiddleCommandType):

    def __init__(self, sensor_id=None, command_id=None, name=None):
        super(DisableSensorType, self).__init__(command_id, name)
        self.SensorID = sensor_id

    def set(self, sensor_id):
        self.SensorID = sensor_id

    def __str__(self):
        return super().__str__() + "\nSensorID= " + str(self.SensorID)

    def run(self):
        # Put your code here!
        pass


class SetEndEffectorParametersType(MiddleCommandType):

    def __init__(self, parameter_settings: list = [], command_id=None, name=None):
        super(SetEndEffectorParametersType, self).__init__(command_id, name)
        self.ParameterSettings = parameter_settings

    def add(self, parameter_settings):
        self.ParameterSettings.append(parameter_settings)

    def __str__(self):
        output = super().__str__()
        for setting in self.ParameterSettings:
            output = output + "\nParamterSetting:\n" + str(setting)
        return output

    def run(self):
        # Put your code here!
        pass


class SetEndEffectorType(MiddleCommandType):

    def __init__(self, setting=None, command_id=None, name=None):
        super(SetEndEffectorType, self).__init__(command_id, name)
        self.Setting = setting

    def set(self, setting):
        self.Setting = setting

    def __str__(self):
        return super().__str__() + "\nSetting= " + str(self.Setting)

    def run(self):
        # Put your code here!
        pass


class EnableGripperType(MiddleCommandType):

    def __init__(self, gripper_name=None, gripper_option=[], command_id=None, name=None):
        super(EnableGripperType, self).__init__(command_id, name)
        self.GripperName = gripper_name
        self.GripperOption = gripper_option

    def set(self, gripper_name):
        self.GripperName = gripper_name

    def add(self, gripper_option):
        self.GripperOption.append(gripper_option)

    def __str__(self):
        output = super().__str__() + "\nGripperName= " + str(self.GripperName)
        for option in self.GripperOption:
            output = output + "\nGripperOption:\n" + str(option)
        return output

    def run(self):
        # Put your code here!
        pass


class DisableGripperType(MiddleCommandType):

    def __init__(self, gripper_name=None, command_id=None, name=None):
        super(DisableGripperType, self).__init__(command_id, name)
        self.GripperName = gripper_name

    def set(self, gripper_name):
        self.GripperName = gripper_name

    def __str__(self):
        output = super().__str__() + "\nGripperName= " + str(self.GripperName)

    def run(self):
        # Put your code here!
        pass


class EnableRobotParameterStatusType(MiddleCommandType):

    def __init__(self, robot_parameter_name=None, command_id=None, name=None):
        super(EnableRobotParameterStatusType, self).__init__(command_id, name)
        self.RobotParameterName = robot_parameter_name

    def set(self, robot_parameter_name):
        self.RobotParameterName = robot_parameter_name

    def __str__(self):
        return super().__str__() + "\nRobotParameterName= " + str(self.RobotParameterName)

    def run(self):
        # Todo
        pass


class DisableRobotParameterStatusType(MiddleCommandType):

    def __init__(self, robot_parameter_name=None, command_id=None, name=None):
        super(DisableRobotParameterStatusType, self).__init__(command_id, name)
        self.RobotParameterName = robot_parameter_name

    def set(self, robot_parameter_name):
        self.RobotParameterName = robot_parameter_name

    def __str__(self):
        return super().__str__() + "\nRobotParameterName= " + str(self.RobotParameterName)

    def run(self):
        # Put your code here!
        pass

    # =========================================================
    #                   Status
    # =========================================================


class CommandStatusType(DataThingType):

    def __init__(self, command_id=None, status_id=None, command_state: CommandStateEnumType = "CRCL_DONE",
                 state_description=None, program_file=None, program_index=None, program_length=None,
                 override_percent=None, name=None):
        super(CommandStatusType, self).__init__(name)
        self.CommandID = command_id
        self.StatusID = status_id
        self.CommandState = command_state
        self.StateDescription = state_description
        self.ProgramFile = program_file
        self.ProgramIndex = program_index
        self.ProgramLength = program_length
        self.OverridePercent = override_percent

    def set(self, command_id, status_id, command_state):
        self.CommandID = command_id
        self.StatusID = status_id
        self.CommandState = command_state

    def __str__(self):
        return super().__str__() + "\nCommandID= " + str(self.CommandID) + "\nStatusID= " + str(
            self.StatusID) + "\nCommandState= " + CommandStateEnumType(
            self.CommandState).name + "\nStateDescription= " + str(self.StateDescription) + "\nProgramFile= " + str(
            self.ProgramFile) + "\nProgramIndex= " + str(self.ProgramIndex) + "\nProgramLength= " + str(
            self.ProgramLength) + "\nOverridePercent= " + str(self.OverridePercent)


class PoseStatusType(DataThingType):

    def __init__(self, pose=None, twist=None, wrench=None, configuration=None, name=None):
        super(PoseStatusType, self).__init__(name)
        self.Pose = pose
        self.Twist = twist
        self.Wrench = wrench
        self.Configuration = configuration

    def __str__(self):
        return super().__str__() + "\nPose= " + str(self.Pose) + "\nTwist= " + str(self.Twist) + "\nWrench" + str(
            self.Wrench) + "\nConfiguration= " + str(self.Configuration)

    def update(self):
        # Put your code here
        pass


class JointStatusType(DataThingType):

    def __init__(self, joint_number=None, joint_position=None, joint_torque_or_force=None, joint_velocity=None,
                 name=None):
        super(JointStatusType, self).__init__(name)
        self.JointNumber = joint_number
        self.JointPosition = joint_position
        self.JointTorqueOrForce = joint_torque_or_force
        self.JointVelocity = joint_velocity

    def __str__(self):
        return super().__str__() + "\nJointNumber= " + str(self.JointNumber) + "\JointPosition= " + str(
            self.JointPosition) + "\nJointTorqueOrForce= " + str(self.JointTorqueOrForce) + "\nJointVelocity= " + str(
            self.JointVelocity)

    def update(self, robot, i):
        joints = robot.get_actual_q()
        print(joints)


class JointStatusesType(DataThingType):

    def __init__(self, joint_status=[], name=None):
        super(JointStatusesType, self).__init__(name)
        self.JointStatus = joint_status

    def add(self, wps):
        try:
            self.JointStatus.append(wps)
        except:
            return False
        return True

    def __str__(self):
        output = super().__str__()
        for status in self.JointStatus:
            output = output + "\nJointStatus:\n" + str(status)
        return output

    def update(self, status_conf):
        # check what joints need to be reported and update the status of them.
        # Todo
        pass


class JointLimitType(DataThingType):

    def __init__(self, joint_number=None, joint_min_position=None, joint_max_position=None,
                 joint_max_torque_or_force=None, joint_max_velocity=None, name=None):
        super(JointLimitType, self).__init__(name)
        self.JointNumber = joint_number
        self.JointMinPosition = joint_min_position
        self.JointMaxPosition = joint_max_position
        self.JointMaxTorqueOrForce = joint_max_torque_or_force
        self.JointMaxVelocity = joint_max_velocity

    def __str__(self):
        return super().__str__() + "\nJointNumber= " + str(self.JointNumber) + "\nJointMinPosition= " + str(
            self.JointMinPosition) + "\nJointMaxPosition= " + str(
            self.JointMaxPosition) + "\nJointMaxTorquOrForce= " + str(
            self.JointMaxTorqueOrForce) + "\nJointMaxVelocity= " + str(self.JointMaxVelocity)


class GripperStatusType(DataThingType):

    def __init__(self, gripper_name=None, gripper_option=[], holding_opject=None, name=None):
        super(GripperStatusType, self).__init__(name)
        self.GripperName = gripper_name
        self.GripperOption = gripper_option
        self.HoldingObject = holding_opject

    def set(self, gripper_name):
        self.GripperName = gripper_name

    def add(self, gripper_option):
        self.GripperOption.append(gripper_option)

    def __str__(self):
        output = super().__str__() + "\nGripperName= " + str(self.GripperName)
        for option in self.GripperOption:
            output = output + "\nGripperOption:\n" + str(option)
        return output


class ParallelGripperStatusType(GripperStatusType):

    def __init__(self, separation=None, gripper_name=None, gripper_option=[], holding_object=None, name=None):
        super(ParallelGripperStatusType, self).__init__(gripper_name, gripper_option, holding_object, name)
        self.Separation = separation

    def __str__(self):
        return super().__str__() + "\nSeparation= " + str(self.Separation)


class ThreeFingerGripperStatusType(GripperStatusType):

    def __init__(self, finger1_position=None, finger2_position=None, finger3_position=None, finger1_force=None,
                 finger2_force=None, finger3_force=None, gripper_name=None, gripper_option=[], holding_object=None,
                 name=None):
        super(ThreeFingerGripperStatusType, self).__init__(gripper_name, gripper_option, holding_object, name)
        self.Finger1Position = finger1_position
        self.Finger2Position = finger2_position
        self.Finger3Position = finger3_position
        self.Finger1Force = finger1_force
        self.Finger2Force = finger2_force
        self.Finger3Force = finger3_force

    def __str__(self):
        return super().__str__() + "\nFinger1Position= " + str(self.Finger1Position) + "\nFinger2Position= " + str(
            self.Finger2Position) + "\nFinger3Position= " + str(self.Finger3Position) + "\nFinger1Force= " + str(
            self.Finger1Force) + "\nFinger2Force= " + str(self.Finger2Force) + "\nFinger3Force= " + str(
            self.Finger3Force)


class VacuumGripperStatusType(GripperStatusType):

    def __init__(self, is_powered=None, gripper_name=None, gripper_option=[], holding_object=None, name=None):
        super(VacuumGripperStatusType, self).__init__(gripper_name, gripper_option, holding_object, name)
        self.IsPowered = is_powered

    def __str__(self):
        return super().__str__() + "\nIsPowered= " + str(self.IsPowered)


class SensorStatusType(DataThingType):

    def __init__(self, sensor_id=None, read_count=None, last_read_time=None, sensor_parameter_setting=[], name=None):
        super(SensorStatusType, self).__init__(name)
        self.SensorID = sensor_id
        self.ReadCount = read_count
        self.LastReadTime = last_read_time
        self.SensorParameterSetting = sensor_parameter_setting

    def __str__(self):
        output = super().__str__() + "\nSensorID= " + str(self.SensorID) + "\nReadCount= " + str(
            self.ReadCount) + "\nLastReadTime= " + str(self.LastReadTime)
        for setting in self.SensorParameterSetting:
            output = output + "\nSensorParameterSetting:\n" + str(setting)
        return output


class OnOffSensorStatusType(SensorStatusType):

    def __init__(self, on=None, sensor_id=None, read_count=None, last_read_time=None, sensor_parameter_setting=[],
                 name=None):
        super(OnOffSensorStatusType, self).__init__(sensor_id, read_count, last_read_time, sensor_parameter_setting,
                                                    name)
        self.On = on

    def __str__(self):
        return super().__str__() + "\nOn= " + str(self.On)


class ForceTorqueSensorStatusType(SensorStatusType):

    def __init__(self, f_x=None, f_y=None, f_z=None, t_x=None, t_y=None, t_z=None, sensor_id=None, read_count=None,
                 last_read_time=None, sensor_parameter_setting=[],
                 name=None):
        super(ForceTorqueSensorStatusType, self).__init__(sensor_id, read_count, last_read_time,
                                                          sensor_parameter_setting,
                                                          name)
        self.Fx = f_x
        self.Fy = f_y
        self.Fz = f_z
        self.Tx = t_x
        self.Ty = t_y
        self.Tz = t_z

    def __str__(self):
        return super().__str__() + "\nFx= " + str(self.Fx) + "\nFy= " + str(self.Fy) + "\nFz= " + str(
            self.Fz) + "\nTx= " + str(self.Tx) + "\nTy= " + str(self.Ty) + "\nTz= " + str(self.Tz)


class ScalarSensorStatusType(SensorStatusType):

    def __init__(self, scalar_value=None, sensor_id=None, read_count=None, last_read_time=None,
                 sensor_parameter_setting=[],
                 name=None):
        super(ScalarSensorStatusType, self).__init__(sensor_id, read_count, last_read_time, sensor_parameter_setting,
                                                     name)
        self.ScalarValue = scalar_value

    def __str__(self):
        return super().__str__() + "\nScalarValue= " + str(self.ScalarValue)


class CountSensorStatusType(SensorStatusType):

    def __init__(self, count_value=None, sensor_id=None, read_count=None, last_read_time=None,
                 sensor_parameter_setting=[],
                 name=None):
        super(CountSensorStatusType, self).__init__(sensor_id, read_count, last_read_time, sensor_parameter_setting,
                                                    name)
        self.CountValue = count_value

    def __str__(self):
        return super().__str__() + "\nCountValue= " + str(self.CountValue)


class SensorStatusesType(DataThingType):

    def __init__(self, on_off_sensor_status=[], scalar_sensor_status=[], count_sensor_status=[],
                 force_torque_sensor_status=[], name=None):
        super(SensorStatusesType, self).__init__(name)
        self.OnOffSensorStatus = on_off_sensor_status
        self.ScalarSensorStatus = scalar_sensor_status
        self.CountSensorStatus = count_sensor_status
        self.ForceTorqueSensorStatus = force_torque_sensor_status

    def __str__(self):
        output = super().__str__()
        for status in self.OnOffSensorStatus:
            output = output + "\nOnOffSensorStatus:\n" + str(status)
        for status in self.ScalarSensorStatus:
            output = output + "\nScalarSensorStatus:\n" + str(status)
        for status in self.CountSensorStatus:
            output = output + "\nCountSensorStatus:\n" + str(status)
        for status in self.ForceTorqueSensorStatus:
            output = output + "\nForceTorqueSensorStatus:\n" + str(status)


class GuardsStatusesType(DataThingType):

    def __init__(self, guard=[], trigger_count=None, trigger_stop_time_micros=None, trigger_value=None,
                 trigger_pose=None, name=None):
        super(GuardsStatusesType, self).__init__(name)
        self.Guard = guard
        self.TriggerCount = trigger_count
        self.TriggerStopTimeMicros = trigger_stop_time_micros
        self.TriggerValue = trigger_value
        self.TriggerPose = trigger_pose

    def __str__(self):
        output = super().__str__() + "\nTriggerCount= " + str(self.TriggerCount) + "\nTriggerStopTimeMicros= " + str(
            self.TriggerStopTimeMicros) + "\nTriggerValue= " + str(self.TriggerValue) + "\nTriggerPose= " + str(
            self.TriggerPose)
        for g in self.Guard:
            output = output + "\nGuard:\n" + str(g)
        return output


class SettingsStatusType(DataThingType):

    def __init__(self, angel_unit_name=None, end_effector_parameter_setting=None, end_effector_setting=None,
                 force_unit_name=None, joint_limits=[], intermediate_pose_tolerance=None, length_unit_name=None,
                 max_cartesian_limit=None, min_cartesian_limit=None, motion_coordinated=None, pose_tolerance=None,
                 robot_parameter_setting=None, rot_accel_absolut=None, rot_accel_relative=None, rot_speed_absolut=None,
                 rot_speed_relative=None, torque_unit_name=None, trans_accel_absolute=None, trans_accel_relative=None,
                 trans_speed_absolute=None, trans_speed_relative=None, name=None):
        super(SettingsStatusType, self).__init__(name)
        self.AngleUnitName = angel_unit_name
        self.EndEffectorParameterSetting = end_effector_parameter_setting
        self.EndEffectorSetting = end_effector_setting
        self.ForceUnitName = force_unit_name
        self.IntermediatePoseTolerance = intermediate_pose_tolerance
        self.JointLimits = joint_limits
        self.LengthUnitName = length_unit_name
        self.MaxCartesianLimit = max_cartesian_limit
        self.MinCartesianLimit = min_cartesian_limit
        self.MotionCoordinated = motion_coordinated
        self.PoseTolerance = pose_tolerance
        self.RobotParameterSetting = robot_parameter_setting
        self.RotAccelAbsolute = rot_accel_absolut
        self.RotAccelRelative = rot_accel_relative
        self.RotSpeedAbsolute = rot_speed_absolut
        self.RotSpeedRelative = rot_speed_relative
        self.TorqueUnitName = torque_unit_name
        self.TransAccelAbsolute = trans_accel_absolute
        self.TransAccelRelative = trans_accel_relative
        self.TransSpeedAbsolute = trans_speed_absolute
        self.TransSpeedRelative = trans_speed_relative


class CRCLStatusType(DataThingType):

    def __init__(self, command_status=None, joint_statuses=None, pose_status=None, gripper_status=None,
                 settings_status=None, sensor_statuses=None, name=None):
        super(CRCLStatusType, self).__init__(name)
        self.CommandStatus = command_status
        self.JointStatuses = joint_statuses
        self.PoseStatus = pose_status
        self.GripperStatus = gripper_status
        self.SettingsStatus = settings_status
        self.SensorStatuses = sensor_statuses

    def __str__(self):
        return super().__str__() + "\nCommandStatus= " + str(self.CommandStatus) + "\nJointStatus= " + str(
            self.JointStatuses) + "\nPoseStatus= " + str(self.PoseStatus) + "\nGripperStatus= " + str(
            self.GripperStatus) + "\nSettingsStatus= " + str(self.SettingsStatus) + "\nSensorStatuses= " + str(
            self.SensorStatuses)
