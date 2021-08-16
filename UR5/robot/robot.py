"""@package docstring
Python library to control an UR robot through its TCP/IP interfaces
The library uses the RealTimeMonitor, the Dashboard interface and
the RealTimeDataExchange interface.

"""

import logging
import os
import sys
import time
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

from pubsub import pub
import CRCL_DataTypes

from robot import realmon
from robot import exchangeData
from robot import dashboard_server
from threading import Thread
import math3d as m3d
import math
import numpy
from transform import transform
from scipy.spatial.transform import Rotation as R



class RobotSettings:

    def __init__(self):
        self.max_speed = 5.0  # m/s
        self.max_accel = 2.0  # m/s²
        self.max_rot_speed = 3.14
        self.max_rot_accel = 3.14

        self.lengthUnit = "millimeter"
        self.angleUnit = "radian"
        self.forceUnit = "newton"
        self.rotSpeed = 2.0
        self.rotAccel = 0.3
        self.transSpeed = 200
        self.transAccel = 0.2
        self.radius = 0.0
        self.workobject = [[400, 0, 150],[0, 0, 0]]  # in mm and rad [[-110, 350, 250],[3.14, -3.14, 0]], [[-560, -40, -195], [0, 0, 0]]
        self.workobject_m3d = m3d.Transform(m3d.Orientation.new_rotation_vector((
            self.workobject[1][0], self.workobject[1][1], self.workobject[1][2])),
            m3d.Vector(self.workobject[0][0], self.workobject[0][1], self.workobject[0][2]))
        self.tcp = (0, 0, 0, 0, 0, 0)  # (x, y, z, rx, ry, rz)


class Robot(object):
    # This class represents the robot.

    def __init__(self, host):
        # Three objects are created. One for the Secondary Monitor interface,
        # one for the RTDE interface and one for the Dashboard interface
        # create a logger for the robot object
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        log_handler = logging.StreamHandler()
        log_handler.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(log_handler)

        self.host = host

        # create the socket connections to the robot
        self.real_mon = realmon.RealTimeMonitor(self.host)
        self.rtde = exchangeData.ExchangeData(self.host)
        logging.getLogger("rtde").setLevel(logging.DEBUG)
        self.dash = dashboard_server.Dashboard(self.host)

        self.robot_settings = RobotSettings()
        self.scale_linear = 0.001
        self.pose_goal = (0, 0, 0, 0, 0, 0,)
        self.ok = True
        self.live_mode = True
        self.buffer = "def myProg():\n"
        self.buffer_len = 0
        self.count = 0
        self.start_buffer = False
        self.set_tcp(self.robot_settings.tcp)  # Init should be done from SAMY Core

        # Subscribe to Topics
        pub.subscribe(self.popup, "Message")
        pub.subscribe(self.move_to, "MoveTo")
        pub.subscribe(self.move_through_to, "MoveThroughTo")
        pub.subscribe(self.dwell, "Dwell")
        pub.subscribe(self.send_status, "GetStatus")
        pub.subscribe(self.set_end_effector, "SetEndeffector")
        pub.subscribe(self.set_trans_accel, "SetTransAccel")
        pub.subscribe(self.set_trans_speed, "SetTransSpeed")
        pub.subscribe(self.set_length_units, "SetLengthUnits")
        pub.subscribe(self.stop_robot, "StopMotion")

        self.logger.info("Connected")

    # =========================================================
    #                   CRCL Methods
    # =========================================================

    def popup(self, data):
        self.dash.popup(str(data.Message))

    def move_to(self, data):
        pose = self.get_pose_from_crcl_position(data.EndPosition)
        if data.MoveStraight:
            self.movel(pose, self.robot_settings.transAccel, self.robot_settings.transSpeed, 0)
        else:
            self.movej_p(pose, self.robot_settings.rotAccel, self.robot_settings.rotSpeed, 0, 0)

    def move_through_to(self, data):
        for waypoint in data.Waypoint:
            pose = self.get_pose_from_crcl_position(waypoint)
            if data.MoveStraight:
                self.movel(pose, self.robot_settings.transAccel, self.robot_settings.transSpeed, 0)
                self.compare_pose(pose[0], pose[1], pose[2])
            else:
                self.movej_p(pose, self.robot_settings.rotAccel, self.robot_settings.rotSpeed, 0, 0)
                self.compare_pose(pose[0], pose[1], pose[2])

    def dwell(self, data):
        """ Sleep for an amount of time.
            Input parameters:
                dwell_time: time the robot waits
        """
        line = "sleep({})\n".format(data.DwellTime)
        self.send_command(line)

    def send_status(self, data):
        self.logger.info(data)
        # complex status information is not jet supported

    def set_end_effector(self, data):
        if data.Setting.fraction > 0.0:
            self.open_gripper()
        else:
            self.close_gripper()

    def set_trans_accel(self, data):
        if data.TransAccel.switchField == 1:
            self.robot_settings.transAccel = data.TransAccel.unionValue.setting
        else:
            self.logger.info("Setting transAccel as fraction is not supported")

    def set_trans_speed(self, data):
        if data.TransSpeed.switchField == 1:
            self.robot_settings.transSpeed = data.TransSpeed.unionValue.setting
        else:
            self.logger.info("Setting transSpeed as fraction is not supported")

    def set_length_units(self, data):
        if data.UnitName == 0:
            linear = "meters"
        elif data.UnitName == 1:
            linear = "millimeters"
        else:
            self.logger.error("Inch is not supported")

        units_l = {"millimeters": 0.001,
                   "meters": 1.0}
        self.scale_linear = units_l[linear]

    def stop_robot(self, data):
        self.dash.stop_program()



    # =========================================================
    #                   Helper Methods
    # =========================================================

    def get_pose_from_crcl_position(self, position):
        pose = []
        pose.append(position.point.x)
        pose.append(position.point.y)
        pose.append(position.point.z)
        rotvec = transform.axis_vectors_to_rotvec(position.xAxis, position.zAxis)
        pose.append(rotvec[0])
        pose.append(rotvec[1])
        pose.append(rotvec[2])
        return pose

    def send_command(self, line):
        if self.live_mode == True:
            #self.logger.info("Python: live mode is ON.")
            #self.logger.info(line)
            self.real_mon.send(line)
            #print("Line send over RealMon")
        else:
            self.logger.info("Python: live mode is OFF.")
            self.buffer = self.buffer + line  # add line to buffer
            self.buffer_len = self.buffer_len + 1   # increase buffer length counter
            self.logger.info("Python: buffer leng: {}".format(self.buffer_len))
            # Automatically start the buffer for testing
            # if self.buffer_len >= 16:
            #     print("Starting buffer")
            #     self.start_buffer = Thread(target=self.run_buffer(), args=())
            time.sleep(0.1)

    def compare_pose(self, x, y, z):
        if self.live_mode:
            dif_pose = [0, 0, 0]
            pose_reached = False
            self.logger.info("Python: comparing pose....")
            while not pose_reached:
                rob_pose = self.get_actual_tcp()
                dif_pose[0] = (abs(rob_pose[0]) - abs(self.pose_goal[0])*1000)
                dif_pose[1] = (abs(rob_pose[1]) - abs(self.pose_goal[1])*1000)
                dif_pose[2] = (abs(rob_pose[2]) - abs(self.pose_goal[2])*1000)
                #print("Rob Pose: {} || Dif Pose: {} || Goal: {}".format(rob_pose, dif_pose, self.pose_goal))
                if abs(dif_pose[0]) < 0.5 and abs(dif_pose[1]) < 0.5 and abs(dif_pose[2]) < 0.5:
                    pose_reached = True
                    self.logger.info("Python: Pose reached")
                time.sleep(0.009)
            return pose_reached

    def run_buffer(self):
        # send all buffered commands as one program to the robot
        self.logger.info("Python: run buffer")
        self.buffer = self.buffer + "end\n"
        self.logger.info(self.buffer)
        self.real_mon.send(self.buffer)
        self.buffer = "def myProg():\n"
        self.buffer_len = 0
        self.logger.info("Python: Buffer completed")

    def close(self):
        # This function closes all connections
        self.rtde.close()  # Stop the connection to the rtde interface
        self.dash.close()  # Stop the connection to the dashboard interface
        self.real_mon.close()  # Stop the connection to the real mon interface

    def shutdown(self):
        # close the connection to the robot and turn it off
        self.stop = True  # Set the stop variable to stop the run_que thread
        self.rtde.close()  # Stop the connection to the rtde interface
        self.real_mon.close()  # Stop the connection to the real mon interface
        self.dash.shutdown()  # Send the shutdown command to the robot
        # self.dash.close()

    def get_state(self):
        """ This function reads the runtime state of the robot.
            - STOPPING
            - STOPPED
            - RUNNING
            - PAUSING
            - PAUSED
            - RESUMING
            - RETRACTING
        """
        runtime_state = self.rtde.get_runtime_state()
        return runtime_state

    def set_mode(self, mode):
        # Set the operation mode of the Plugin:
        # live_mode = True: commands get executed as soon as the are recived
        # live_mode = False: commands get translated, buffered and then send
        # to the robot all at once
        if mode == 1:
            self.live_mode = True
        elif mode == 0:
            self.live_mode = False

    def adjust_pose(self, pose):
        # Adjust the robot pose to the current workobject
        # Create m3d objects from given pose
        # Transform m3d objects with m3d.Transform(workobject_m3d)
        # Create new pose with new data

        v = m3d.Vector(pose[0], pose[1], pose[2])
        #  ori = m3d.Orientation.new_rotation_vector((pose[3], pose[4], pose[5]))
        v = self.robot_settings.workobject_m3d * v

        pose[0] = (v.x) * self.scale_linear
        pose[1] = (v.y) * self.scale_linear
        pose[2] = (v.z) * self.scale_linear
        pose[3] = (pose[3] + self.robot_settings.workobject[1][0])
        pose[4] = (pose[4] + self.robot_settings.workobject[1][1])
        pose[5] = (pose[5] + self.robot_settings.workobject[1][2])
        self.pose_goal = pose.copy()
        return pose



    # =========================================================
    #                   Move commands
    # =========================================================

    def movel(self, pose: list, a, v, r=0):
        # The movel command moves the robot linear to the given pose.
        #    Input parameters:
        #        pose: The pose the robot moves to.
        #        a: constant acceleration of the TCP
        #        v: constant speed of the TCP
        #        r: radius within the robot stays when another move command follows.

        # Adjust pose to workobject and units
        self.logger.info("movel")
        pose_new = self.adjust_pose(pose)
        self.logger.info(pose_new)

        line = "movel(p[{},{},{},{},{},{}], a={}, v={}, r={})\n".format(*pose_new, a,
                                                                        v*self.scale_linear, r)
        self.logger.info(line)
        self.send_command(line)

    def movec(self, pose_via, pose_to, a, v, r=0, mode=0):
        # Move Circular: Move to position (circular in tool-space)
        #   TCP moves on the circular arc segment from current pose, through
        #   pose_via to pose_to. Accelerates to and moves with constant tool
        #   speed v. Use the mode parameter to define the orientation
        #   interpolation.
        #   Input parameter:
        #       pose_via
        #       pose_to
        #       a: constant acceleration of the TCP
        #       v: constant speed of the TCP
        # adjust pose to workobject
        pose_via_new = self.adjust_pose(pose_via)
        # adjust pose to workobject
        pose_to_new = self.adjust_pose(pose_to)

        data = [a, v, r, mode]
        line = "movec(p[{},{},{},{},{},{}], p[{},{},{},{},{},{}], a={}, v={}, r={}, mode={})\n".format(*pose_via_new,
                                                                                                       *pose_to_new, *data)
        self.send_command(line)

    def movej(self, q, a, v, t=0, r=0):
        # Move to position (linear in joint-space)
        #    Input Parameters:
        #        q: joint positions (q can also be specified as a pose, then
        #        inverse kinematics is used to calculate the corresponding
        #        joint positions)
        #        a: joint acceleration of leading axis [rad/s^2]
        #        v: joint speed of leading axis [rad/s]
        #        t: time [S]
        #        r: blend radius [m]

        line = "movej([{},{},{},{},{},{}], a={}, v={}, t={}, r={})\n".format(*q, a, v, t, r)
        self.send_command(line)

    def movej_p(self, pose, a, v, t=0, r=0):
        """ Move to position (linear in joint-space)
            Input Parameters:
                q: joint positions (q can also be specified as a pose, then
                inverse kinematics is used to calculate the corresponding
                joint positions)
                a: joint acceleration of leading axis [rad/s^2]
                v: joint speed of leading axis [rad/s]
                t: time [S]
                r: blend radius [m]
        """
        # adjust pose to workobject
        pose_new = self.adjust_pose(pose)
        line = "movej(p[{},{},{},{},{},{}], a={}, v={}, t={}, r={})\n".format(*pose_new, a, v, t, r)
        self.send_command(line)

    def movep(self, pose, a, v, r=0):
        """ Move Process
            Parameters
                pose: target pose (pose can also be specified as joint
                    positions, then forward kinematics is used to
                    calculate the corresponding pose)
                a: tool acceleration [m/s^2]
                v: tool speed [m/s]
                r: blend radius [m]
        """
        # adjust pose to workobject
        pose_new = self.adjust_pose(pose)
        line = "movep(p[{},{},{},{},{},{}], a={}, v={}, r={})\n".format(*pose_new, a, v, r)
        self.send_command(line)

    def servoc(self, pose, a, v, r=0):
        """ Servo Circular
            Parameters
                pose: target pose (pose can also be specified as joint
                    positions, then forward kinematics is used to
                    calculate the corresponding pose)
                a: tool acceleration [m/s^2]
                v: tool speed [m/s]
                r: blend radius (of target pose) [m]
        """
        # adjust pose to workobject
        pose_new = self.adjust_pose(pose)
        line = "servoc(p[{},{},{},{},{},{}], a={}, v={}, r={})\n".format(*pose_new, a, v, r)
        self.send_command(line)



    # =========================================================
    #                   Data write commands
    # =========================================================
    def set_workobject(self, x, y, z, roll, pitch, yaw):
        self.robot_settings.workobject = [[x, y, z],[roll, pitch, yaw]]

    def set_tcp(self, tcp):
        """ set robot flange to tool tip transformation
            Input parameters:
                tcp: The pose of the tcp measured from the robot flange.
        """
        line = "set_tcp(p[{},{},{},{},{},{}])\n".format(*tcp)
        self.send_command(line)

    def set_analog_output(self, output, val):
        """ Write analog output.
            Input parameter:
                int output: Number of analog output (0 or 1)
                double val: Range from 0 to 1 corresponds 0 to 24V
        """
        data = self.rtde.send_object
        data.__dict__["standard_analog_output_%i" % output] = val
        self.rtde.con.send(data)

    def set_payload(self, m, cog):
        """ Set Payload mass and center of gravity. The va
            Input Parameters:
                m: mass in kilograms
                cog: Center of Gravity, a vector [CoGx, CoGy, CoGz]
                    specifying the displacement (in meters) from the
                    tool mount.
        """
        line = "set_payload({}, [{},{},{}])\n".format(m, *cog)
        self.send_command(line)



    # =========================================================
    #                   Data read commands
    # =========================================================

    def get_actual_tcp(self):
        """ This function reads the actual TCP pose form the robot,
            relative to the workobject.
            It uses the RTDE interface.
        """
        pose = self.rtde.get_actual_tcp()
        # adjust pose to workobject
        pose[0] = pose[0] / self.scale_linear# - self.robot_settings.workobject[0][0]
        pose[1] = pose[1] / self.scale_linear# - self.robot_settings.workobject[0][1]
        pose[2] = pose[2] / self.scale_linear# - self.robot_settings.workobject[0][2]
        pose[3] = (pose[3] - self.robot_settings.workobject[1][0])
        pose[4] = (pose[4] - self.robot_settings.workobject[1][1])
        pose[5] = (pose[5] - self.robot_settings.workobject[1][2])
        return pose

    def get_actual_tcp_from_base(self):
        """ This function reads the actual TCP pose relative to  the robot
            base coordinate system.
            It uses the RTDE interface.
        """
        pose = self.rtde.get_actual_tcp()
        # adjust pose to workobject
        pose[0] = pose[0] / self.scale_linear# - self.robot_settings.workobject[0][0]
        pose[1] = pose[1] / self.scale_linear# - self.robot_settings.workobject[0][1]
        pose[2] = pose[2] / self.scale_linear# - self.robot_settings.workobject[0][2]
        return pose

    def get_target_q(self):
        """ Gets the Joints angles for the targeted position.
        """
        return self.rtde.get_target_q()

    def get_actual_q(self):
        """ Gets the Joints angles for the actual position.
        """
        return self.rtde.get_actual_q()


    # =========================================================
    #                   Robotiq commands
    # =========================================================

    def activate_gripper(self, num=1):
        """ Activates the RobotIQ gripper. This function
            must be called before any other gripper command.
            Input parameter:
                num: Number of the gripper (1-4)
        """
        if 1 <= num <= 4:
            line = "rq_activate_and_wait({})\n".format(num)
            self.send_command(line)
        else:
            self.logger.info("Gripper number must be between 1-4")

    def get_gripper_state(self):
        bits = self.rtde.get_digital_output_bits()
        if bits & 4 == 4:
            self.logger.info("Gripper is open")
            return True
        else:
            self.logger.info("Gripper is closed")
            return False

    def holds_object(self):
        time.sleep(7) # wait for the gripper to close
        bits = self.rtde.get_digital_output_bits()
        if bits & 16 == 16:
            self.logger.info("Gripper is holding an object")
            return True
        else:
            self.logger.info("Gripper is empty")
            return False

        # print("{0:b}".format(bits))

    def close_gripper(self):
        """ Moves the gripper to its fully closed position.
            Input parameter:
                num: Number of the gripper (1-4)
        """
        # if 1 <= num <= 4:
        line = "set_digital_out(3, True)\n"
        line3 = "set_digital_out(2, False)\n"
        line2 = "sleep(6)\n"
        # line = "rq_close_and_wait({})\n".format(num)
        self.send_command(line3)
        time.sleep(1)
        self.send_command(line)
        if self.live_mode == False:
            self.send_command(line2)
        else:
            time.sleep(4)
        #self.send_command(line2)
        # else:
        #    print("Gripper number must be between 1-4")

    def open_gripper(self):
        """ Moves the gripper to its fully opened position.
            Input parameter:
                num: Number of the gripper (1-4)
        """

        # line = "rq_open_and_wait({})\n".format(num)
        line = "set_digital_out(2, True)\n"
        line3 = "set_digital_out(3, False)\n"
        line2 = "sleep(6)\n"
        self.send_command(line3)
        time.sleep(1)
        self.send_command(line)
        if self.live_mode == False:
            self.send_command(line2)
        else:
            time.sleep(4)
        #self.send_command(line2)

    def move_gripper(self, position, num=1):
        """ Moves the gripper to the given position.
            Input parameter:
                position: Position the of the gripper in [mm]
                num: Number of the gripper (1-4)
        """
        # if 1 <= num <= 4:
        line = "rq_move_and_wait_mm({},{})\n".format(position, num)
        self.send_command(line)
        # else:
        #    print("Gripper number must be between 1-4")

    def set_gripper_force(self, force, num=1):
        """ Sets the gripping force.
            Input parameters:
                force: Value between 0 and 100. 0 = minimal force, 100 = maximal force.
                num: Number of the gripper (1-4)
        """
        if 1 <= num <= 4:
            line = "rq_set_force_norm({},{})\n".format(force, num)
            self.send_command(line)
        else:
            self.logger.info("Gripper number must be between 1-4")

    def set_gripper_speed(self, speed, num=1):
        """ Sets the gripping force.
            Input parameters:
                force: Value between 0 and 100. 0 = minimal force, 100 = maximal force.
                num: Number of the gripper (1-4)
        """
        if 1 <= num <= 4:
            line = "rq_set_speed_norm({},{})\n".format(speed, num)
            self.send_command(line)
        else:
            self.logger.info("Gripper number must be between 1-4")
