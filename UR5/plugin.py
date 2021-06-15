from CRCLLib import crcl
from TransformLib import transform
import numpy
import time

class RobotUnits():
    def __init__(self):
        # self.orientation = "Quaternion" Unterschiedliche anzahl an parametern selbst konvertieren
        self.referencePoint = "RobotBase"
        self.lengthUnit = "mm"
        self.speedUnit = "mm/s"
        self.accelUnit = "mm/s^2"


def stop(robot):
    robot.stop_robot()


def compare_pose(robot, x, y, z):
    robot.compare_pose(x, y, z)


def set_mode(robot, command_id, mode):
    robot.set_mode(mode)


def run_buffered_commands(robot):
    robot.run_buffer()


def get_state(robot):
    return robot.get_state()


def set_length_unit(robot, command_id, unit):
    robot.lengthUnit = unit
    print("Python: Length unit set")
    # send command status Done


def set_angle_units(robot, command_id, unit):
    robot.angleUnit = unit
    print("Python: Angle units set")


def set_end_effector(robot, command_id, setting):
    if setting == 0:
        robot.close_gripper(command_id)
        print("EndEffector is false")
    else:
        robot.open_gripper(command_id)
        print("EndEffector is true")


def get_end_effector(robot, command_id):
    # retruns true if the gripper is open false if it ia closed
    return robot.get_gripper_state(command_id)

def set_trans_speed(robot, transSpeed):
    robot.robot_settings.transSpeed = transSpeed
    print("Python: Trans Speed set")


def set_trans_accel(robot, transAccel):
    robot.robot_settings.transAccel = transAccel
    print("Python: Trans Accel set")


def set_workobject(robot, workobject_x, workobject_y, workobject_z,
                   workobject_xAxis_i, workobject_xAxis_j, workobject_xAxis_k,
                   workobject_zAxis_i, workobject_zAxis_j, workobject_zAxis_k):
    xaxis = numpy.array([workobject_xAxis_i, workobject_xAxis_j, workobject_xAxis_k])
    zaxis = numpy.array([workobject_zAxis_i, workobject_zAxis_j, workobject_zAxis_k])
    yaxis = numpy.cross(zaxis, xaxis)
    matrix = numpy.matrix([xaxis, yaxis, zaxis])
    axis_angle = transform.matrix_to_axis_angle(matrix)
    robot.robot_settings.workobject = ((workobject_x, workobject_y, workobject_z), axis_angle)
    print("Python: Workobject set")


# new version with all datafields as seperate parameters
def moveto(robot, commandID, moveStraight, endPosition_point_x, endPosition_point_y, endPosition_point_z,
            endPosition_xAxis_i, endPosition_xAxis_j, endPosition_xAxis_k,
            endPosition_zAxis_i, endPosition_zAxis_j, endPosition_zAxis_k):
    crcl_point = crcl.PointType(endPosition_point_x, endPosition_point_y, endPosition_point_z)
    crcl_u = crcl.VectorType(endPosition_xAxis_i, endPosition_xAxis_j, endPosition_xAxis_k)
    crcl_v = crcl.VectorType(endPosition_zAxis_i, endPosition_zAxis_j, endPosition_zAxis_k)
    pose_type = crcl.PoseType(crcl_point, crcl_u, crcl_v)
    pose = transform.get_pose_axis_angle(pose_type)
    str_pose = str(pose)
    if moveStraight:
        robot.movel(commandID, pose, robot.robot_settings.transAccel, robot.robot_settings.transSpeed, 0)
    else:
        robot.movej_p(robot, commandID, pose, robot.robot_settings.rotAccel, robot.robot_settings.rotSpeed, 0, 0)
    #robot.wait()
    return str_pose


# old version with full struct as parameter
def movethroughto(robot, command_id, move_straight, waypoints, num_positions):
    x = 1
    if move_straight:
        for waypoint in waypoints:
            pose = transform.get_pose_axis_angle(waypoint)
            if x == num_positions:
                robot.rob.movel(command_id, pose, robot.transAccel, robot.transSpeed, 0)
            else:
                robot.rob.movel(command_id, pose, robot.robot_settings.transAccel, robot.robot_settings.transSpeed, robot.robot_settings.radius)
            x += 1
    else:
        for waypoint in waypoints:
            pose = transform.get_pose_axis_angle(waypoint)
            if x == num_positions:
                robot.rob.movej_p(command_id, pose, robot.transAccel, robot.transSpeed, 0, 0)
            else:
                robot.rob.movej_p(command_id, pose, robot.robot_settings.transAccel, robot.robot_settings.transSpeed, 0, robot.robot_settings.radius)
            x += 1
