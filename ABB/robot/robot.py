"""
Michael Dawson-Haggerty

abb.py: contains classes and support functions which interact with an ABB Robot running our software stack (RAPID code module SERVER)

For functions which require targets (XYZ positions with quaternion orientation),
targets can be passed as [[XYZ], [Quats]] OR [XYZ, Quats]

"""

import logging
import os
import sys
import time
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

from pubsub import pub
import samyplugin.CRCL_DataTypes
from robot import connection

import json
import ast
import copy


class RobotSettings:

    def __init__(self):
        self.max_speed = 5.0  # m/s
        self.max_accel = 2.0  # m/s
        self.max_rot_speed = 3.14
        self.max_rot_accel = 3.14

        self.lengthUnit = "millimeters"
        self.angleUnit = "degrees"
        self.forceUnit = "newton"
        self.rotSpeed = 2.0
        self.rotAccel = 0.3
        self.transSpeed = 0.1
        self.transAccel = 0.2
        self.radius = 0.0
        self.workobject = ((971.03, 0, 100), (0, 0, 0, 1)) # [[1.090, 0, -0.070], [1, 0, 0, 0]] # values for printing station
        self.tool = (0, 0, 0, 1, 0, 0, 0) #(116.563, 2.09, 113.7, 0.707106781, 0, 0.707106781, 0)


class Robot:
    def __init__(self, ipAddress):

        # create a logger for the robot object
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        fh = logging.StreamHandler()
        fh.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(fh)
        self.robot_settings = RobotSettings()
        self.connection = connection.Connection(ipAddress)

        self.state = "STOPPED"
        self.live_mode = True
        self.count = 0

        self.set_unit_linear(self.robot_settings.lengthUnit)
        self.set_unit_angular(self.robot_settings.angleUnit)
        time.sleep(0.2)
        self.set_workobject(self.robot_settings.workobject)
        time.sleep(0.2)
        self.set_tool(self.robot_settings.tool)
        time.sleep(0.2)
        self.set_zone(zone_key='z1', point_motion=False)
        time.sleep(0.2)
        print("Python: Robot finished init")

        # Subscribe to Topics
        pub.subscribe(self.move_to, "MoveTo")
        pub.subscribe(self.move_through_to, "MoveThroughTo")
        pub.subscribe(self.dwell, "Dwell")
        pub.subscribe(self.send_status, "GetStatus")
        pub.subscribe(self.set_end_effector, "SetEndeffector")
        pub.subscribe(self.set_trans_accel, "SetTransAccel")
        pub.subscribe(self.set_trans_speed, "SetTransSpeed")
        pub.subscribe(self.set_length_units, "SetLengthUnits")
        pub.subscribe(self.stop_robot, "StopMotion")


    def move_to(self, data):
        pass


    def get_state(self):
        return self.state
        pass

    def stop_robot(self):
        print("Stopping the ABB robot during move is not supported.")
        pass


    def set_unit_linear(self, linear):
        units_l = {"millimeters": 1.0,
                   "meters": 1000.0,
                   "inches": 25.4}
        self.scale_linear = units_l[linear]

    def set_unit_angular(self, angular):
        units_a = {'degrees': 1.0,
                   'radians': 57.2957795}
        self.scale_angle = units_a[angular]

    def set_cartesian(self, pose):
        """
        Executes a move immediately from the current pose,
        to 'pose', with units of millimeters.
        """
        global move_finished
        x, y, z = 0.0, 0.0, 0.0
        msg = "01 " + self.format_pose(pose)
        print(msg)
        if not self.live_mode:
            return self.buffer_add(pose)
        print("Python: sending move command")
        self.send_opcua(msg)

    def set_joints(self, joints):
        """
        Executes a move immediately, from current joint angles,
        to 'joints', in degrees.
        """
        if len(joints) != 6:
            return False
        msg = "02 "
        for joint in joints:
            msg += format(joint * self.scale_angle, "+08.2f") + " "
            msg += "#"
        return self.send_opcua(msg)

    def get_cartesian(self):
        """
        Returns the current pose of the robot, in millimeters
        """
        pose_string = self.opcua.currentTCP_node.get_value()
        tcp = ast.literal_eval(pose_string)
        return tcp[0][0], tcp[0][1], tcp[0][2]

    def get_joints(self):
        """
        Returns the current angles of the robots joints, in degrees.
        """
        msg = "04 #"
        data = self.send(msg).split()
        return [float(s) / self.scale_angle for s in data[2:8]]

    def get_external_axis(self):
        """
        If you have an external axis connected to your robot controller
        (such as a FlexLifter 600, google it), this returns the joint angles
        """
        msg = "05 #"
        data = self.send(msg).split()
        return [float(s) for s in data[2:8]]

    def get_robotinfo(self):
        """
        Returns a robot- unique string, with things such as the
        robot's model number.
        Example output from and IRB 2400:
        ['24-53243', 'ROBOTWARE_5.12.1021.01', '2400/16 Type B']
        """
        msg = "98 #"
        data = str(self.send(msg))[5:].split('*')
        log.debug('get_robotinfo result: %s', str(data))
        return data

    def set_tool(self, tool):
        """
        Sets the tool centerpoint (TCP) of the robot.
        When you command a cartesian move,
        it aligns the TCP frame with the requested frame.

        Offsets are from tool0, which is defined at the intersection of the
        tool flange center axis and the flange face.
        """
        msg = "06 " + self.format_pose(tool)
        print("Python: sending set tool")
        self.send_opcua(msg)

    def load_json_tool(self, file_name):
        # if file_obj.__class__.__name__ == 'str':
        #    file_obj = open(filename, 'rb')
        tool = check_coordinates(json.load(file_name))
        self.set_tool(tool)

    def get_tool(self):
        log.debug('get_tool returning: %s', str(self.robot_settings.tool))
        return self.robot_settings.tool

    def set_workobject(self, work_obj):
        """
        The workobject is a local coordinate frame you can define on the robot,
        then subsequent cartesian moves will be in this coordinate frame.
        """
        msg = "07 " + self.format_pose(work_obj)
        print("Python: sending set workobject")
        self.send_opcua(msg)

    def set_speed(self, speed):
        """
        speed: [robot TCP linear speed (mm/s), TCP orientation speed (deg/s),
                external axis linear, external axis orientation]
        """

        if len(speed) != 4: return False
        msg = "08 "
        msg += format(speed[0], "+08.1f") + " "
        msg += format(speed[1], "+08.2f") + " "
        msg += format(speed[2], "+08.1f") + " "
        msg += format(speed[3], "+08.2f") + " #"
        print("Python: sending set speed")
        self.send_opcua(msg)

    def set_zone(self, zone_key='z1', point_motion=True, manual_zone=[]):
        """
        Sets the motion zone of the robot. This can also be thought of as
        the flyby zone, AKA if the robot is going from point A -> B -> C,
        how close do we have to pass by B to get to C

        zone_key: uses values from RAPID handbook (stored here in zone_dict)
        with keys 'z*', you should probably use these

        point_motion: go to point exactly, and stop briefly before moving on

        manual_zone = [pzone_tcp, pzone_ori, zone_ori]
        pzone_tcp: mm, radius from goal where robot tool centerpoint
                   is not rigidly constrained
        pzone_ori: mm, radius from goal where robot tool orientation
                   is not rigidly constrained
        zone_ori: degrees, zone size for the tool reorientation
        """
        zone_dict = {'z0': [.3, .3, .03],
                     'z1': [1, 1, .1],
                     'z5': [5, 8, .8],
                     'z10': [10, 15, 1.5],
                     'z15': [15, 23, 2.3],
                     'z20': [20, 30, 3],
                     'z30': [30, 45, 4.5],
                     'z50': [50, 75, 7.5],
                     'z100': [100, 150, 15],
                     'z200': [200, 300, 30]}
        if point_motion:
            zone = [0, 0, 0]
        elif len(manual_zone) == 3:
            zone = manual_zone
        elif zone_key in zone_dict.keys():
            zone = zone_dict[zone_key]
        else:
            return False

        msg = "09 "
        msg += str(int(point_motion)) + " "
        msg += format(zone[0], "+08.4f") + " "
        msg += format(zone[1], "+08.4f") + " "
        msg += format(zone[2], "+08.4f") + " #"
        print("Python: sending set zone")
        self.send_opcua(msg)

    def buffer_add(self, pose):
        """
        Appends single pose to the remote buffer
        Move will execute at current speed (which you can change between buffer_add calls)
        """
        msg = "30 " + self.format_pose(pose)
        print("Python: sending buffer add")
        self.send_opcua(msg)
        time.sleep(0.01)

    def buffer_set(self, pose_list):
        """
        Adds every pose in pose_list to the remote buffer
        """
        self.clear_buffer()
        for pose in pose_list:
            self.buffer_add(pose)
        if self.buffer_len == len(pose_list):
            log.debug('Successfully added %i poses to remote buffer',
                      len(pose_list))
            return True
        else:
            log.warn('Failed to add poses to remote buffer!')
            self.clear_buffer()
            return False

    def clear_buffer(self):
        msg = "31 #"
        print("Python: sending clear buffer")
        self.send_opcua(msg)
        time.sleep(1)
        return

    def buffer_len(self):
        """
        Returns the length (number of poses stored) of the remote buffer
        """
        msg = "32 #"
        data = self.send(msg).split()
        return int(float(data[2]))

    def buffer_execute(self):
        """
        Immediately execute linear moves to every pose in the remote buffer.
        """
        global move_finished
        msg = "33 #"
        print("Python: sending buffer execute")
        self.send_opcua(msg)
        time.sleep(1)
        command_done = self.opcua.command_done_node.get_value()
        print(command_done)
        print(type(command_done))
        while not command_done:
            command_done = self.opcua.command_done_node.get_value()
            print(command_done)
            print("Python: waiting for robot to finish buffer")
            time.sleep(1)
        print("Python: Buffered commands executed")

    def set_external_axis(self, axis_values):
        if len(axis_values) != 6: return False
        msg = "34 "
        for axis in axis_values:
            msg += format(axis, "+08.2f") + " "
        msg += "#"
        return self.send_opcua(msg)

    def move_circular(self, pose_onarc, pose_end):
        """
        Executes a movement in a circular path from current position,
        through pose_onarc, to pose_end
        """
        msg_0 = "35 " + self.format_pose(pose_onarc)
        msg_1 = "36 " + self.format_pose(pose_end)

        self.send_opcua(msg_0)
        #data = self.send(msg_0).split()
        #if data[1] != '1':
        #    log.warning('move_circular incorrect response, bailing!')
        #    return False
        return self.send_opcua(msg_1)

    def set_dio(self, value, id=0):
        """
        A function to set a physical DIO line on the robot.
        For this to work you're going to need to edit the RAPID function
        and fill in the DIO you want this to switch.
        """
        msg = "97 " + str(int(bool(value))) + " #"
        # return
        return self.send_opcua(msg)

    def format_pose(self, pose):
        pose = check_coordinates(pose)
        msg = ""
        for cartesian in pose[0]:
            msg += format(cartesian * self.scale_linear, "+08.1f") + " "
        for quaternion in pose[1]:
            msg += format(quaternion, "+08.5f") + " "
        msg += "#"
        return msg

    def close(self):
        #self.sock.shutdown(socket.SHUT_RDWR)
        #self.sock.close()
        log.info('Disconnected from ABB robot.')

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()


def check_coordinates(coordinates):
    if (len(coordinates) == 2) and (len(coordinates[0]) == 3) and (len(coordinates[1]) == 4):
        return coordinates
    elif len(coordinates) == 7:
        return [coordinates[0:3], coordinates[3:7]]
    log.warning('Recieved malformed coordinate: %s', str(coordinates))
    raise NameError('Malformed coordinate!')
