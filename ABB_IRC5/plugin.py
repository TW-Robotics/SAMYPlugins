from CRCLLib import crcl
from TransformLib import transform
import numpy


class RobotUnits():
    def __init__(self):
        # self.orientation = "Quaternion" Unterschiedliche anzahl an parametern selbst konvertieren
        self.referencePoint = "RobotBase"
        self.lengthUnit = "mm"
        self.speedUnit = "mm/s"
        self.accelUnit = "mm/s^2"


def stop(robot):
    robot.stop_robot()


def set_mode(robot, command_id, mode):
    robot.live_mode = mode


def run_buffered_commands(robot):
    robot.buffer_execute()


def clear_buffer(robot):
    robot.clear_buffer()


def get_state(robot):
    return robot.get_state()

def compare_pose(robot, x, y, z):
    robot.compare_pose(x, y, z)

def set_length_units(robot, command_id, unit):
    robot.set_unit_linear(unit)
    # send command status "Done"


def set_angle_units(robot, command_id, unit):
    robot.set_unit_angular(unit)
    # send command status "Done"


def set_end_effector(robot, command_id, setting):
    pass


def set_trans_speed(robot, transSpeed):
    robot.robot_settings.transSpeed = transSpeed
    rotSpeed = 10 # deg/s
    speed = (transSpeed, rotSpeed, 0, 0)
    robot.set_speed(speed) # in mm/s


def set_trans_accel(robot, transAccel):
    pass


def set_workobject(robot, workobject_x, workobject_y, workobject_z,
                   workobject_xAxis_i, workobject_xAxis_j, workobject_xAxis_k,
                   workobject_zAxis_i, workobject_zAxis_j, workobject_zAxis_k):
    xaxis = numpy.array([workobject_xAxis_i, workobject_xAxis_j, workobject_xAxis_k])
    zaxis = numpy.array([workobject_zAxis_i, workobject_zAxis_j, workobject_zAxis_k])

    q = transform.matrix_to_quaternion(xaxis, zaxis)
    robot.robot_settings.workobject = ((workobject_x, workobject_y, workobject_z),
                                       (q[0], q[1], q[2], q[3]))


# new version with all datafields as seperate parameters
def moveto(robot, commandID, moveStraight, endPosition_point_x, endPosition_point_y, endPosition_point_z,
           endPosition_xAxis_i, endPosition_xAxis_j, endPosition_xAxis_k,
           endPosition_zAxis_i, endPosition_zAxis_j, endPosition_zAxis_k):
    crcl_point = crcl.PointType(endPosition_point_x, endPosition_point_y, endPosition_point_z)
    crcl_u = crcl.VectorType(endPosition_xAxis_i, endPosition_xAxis_j, endPosition_xAxis_k)
    crcl_v = crcl.VectorType(endPosition_zAxis_i, endPosition_zAxis_j, endPosition_zAxis_k)
    r, i, j, k = transform.matrix_to_quaternion(crcl_u, crcl_v)
    pose = (crcl_point.X, crcl_point.Y, crcl_point.Z, r, i, j, k)
    print(str(pose))
    if moveStraight:
        robot.set_cartesian(pose)
        print("Command with id " + str(commandID) + " is finished")
    else:
        print("Not Implementet jet")
    return str(pose)


# old version with full struck as parameter
def move_through_to(robot, command_id, move_straight, waypoints, num_positions):
    x = 1
    if move_straight:
        for waypoint in waypoints:
            pose = transform.get_pose_euler(waypoint)
            if x == num_positions:
                robot.rob.movel(command_id, pose, robot.transAccel, robot.transSpeed, 0)
            else:
                robot.rob.movel(command_id, pose, robot.robot_settings.transAccel, robot.robot_settings.transSpeed,
                                robot.robot_settings.radius)
            x += 1
    else:
        for waypoint in waypoints:
            pose = transform.get_pose_euler(waypoint)
            if x == num_positions:
                robot.rob.movej_p(command_id, pose, robot.transAccel, robot.transSpeed, 0, 0)
            else:
                robot.rob.movej_p(command_id, pose, robot.robot_settings.transAccel, robot.robot_settings.transSpeed, 0,
                                  robot.robot_settings.radius)
            x += 1

