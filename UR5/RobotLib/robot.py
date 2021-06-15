"""@package docstring
Python library to control an UR robot through its TCP/IP interfaces
The library uses the RealTimeMonitor, the Dashboard interface and
the RealTimeDataExchange interface.

"""

from RobotLib import realmon
from RobotLib import exchangeData
from RobotLib import dashboard_server
from threading import Thread
import math3d as m3d
import math
import time
import logging


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
        fh = logging.StreamHandler()
        fh.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(fh)

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
        self.set_tcp(0, self.robot_settings.tcp)  # Init should be done from SAMY Core
        self.count = 0
        self.start_buffer = False

        self.logger.info("Connected")

    def send_command(self, line, command_id):
        if self.live_mode == True:
            print("Python: live mode is ON.")
            print(line)
            self.real_mon.send(line)
            #print("Line send over RealMon")
        else:
            print("Python: live mode is OFF.")
            self.buffer = self.buffer + line  # add line to buffer
            self.buffer_len = self.buffer_len + 1   # increase buffer length counter
            print("Python: buffer leng:")
            print(self.buffer_len)
            # Automatically start the buffer for testing
            if self.buffer_len >= 16:
                print("Starting buffer")
                self.start_buffer = Thread(target=self.run_buffer(), args=())
            time.sleep(0.1)

    def compare_pose(self, x, y, z):
        if self.live_mode:
            dif_pose = [0, 0, 0]
            pose_reached = False
            print("Python: comparing pose....")
            while not pose_reached:
                rob_pose = self.get_actual_tcp()
                #print(x, y, z)
                #print("Rob Pose")
                #print(rob_pose)
                dif_pose[0] = (abs(rob_pose[0]) - abs(self.pose_goal[0])*1000)
                dif_pose[1] = (abs(rob_pose[1]) - abs(self.pose_goal[1])*1000)
                dif_pose[2] = (abs(rob_pose[2]) - abs(self.pose_goal[2])*1000)
                #print("Dif Pose:")
                #print(dif_pose)
                #print("Goal:")
                #print(self.pose_goal)
                if abs(dif_pose[0]) < 0.5 and abs(dif_pose[1]) < 0.5 and abs(dif_pose[2]) < 0.5:
                    pose_reached = True
                    print("Python: Pose reached")
                time.sleep(0.009)
            return pose_reached

    def run_buffer(self):
        # send all buffered commands as one program to the robot
        print("Python: run buffer")
        self.buffer = self.buffer + "end\n"
        print(self.buffer)
        self.real_mon.send(self.buffer)
        self.buffer = "def myProg():\n"
        self.buffer_len = 0
        print("Python: Buffer completed")

    def send_status(self, command_id):
        # complex status information is not jet supported
        pass

    def close(self):
        # This function closes all connections
        self.rtde.close()  # Stop the connection to the rtde interface
        self.dash.close()  # Stop the connection to the dashboard interface
        self.real_mon.close()  # Stop the connection to the real mon interface

    def stop_robot(self):
        self.dash.stop_program()

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

    def set_unit_linear(self, linear):
        units_l = {"millimeters": 0.001,
                   "meters": 1.0}
        self.scale_linear = units_l[linear]

    # =========================================================
    #                   Move commands
    # =========================================================

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

    def movel(self, command_id, pose: list, a, v, r=0):
        # The movel command moves the robot linear to the given pose.
        #    Input parameters:
        #        pose: The pose the robot moves to.
        #        a: constant acceleration of the TCP
        #        v: constant speed of the TCP
        #        r: radius within the robot stays when another move command follows.

        # Adjust pose to workobject and units
        pose_new = self.adjust_pose(pose)

        line = "movel(p[{},{},{},{},{},{}], a={}, v={}, r={})\n".format(*pose_new, a,
                                                                        v*self.scale_linear, r)
        self.send_command(line, command_id)

    def movec(self, command_id, pose_via, pose_to, a, v, r=0, mode=0):
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
        self.send_command(line, command_id)

    def movej(self, command_id, q, a, v, t=0, r=0):
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
        self.send_command(line, command_id)

    def movej_p(self, command_id, pose, a, v, t=0, r=0):
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
        self.send_command( line, command_id)

    def movep(self, command_id, pose, a, v, r=0):
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
        self.send_command(line, command_id)

    def servoc(self, command_id, pose, a, v, r=0):
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
        self.send_command(line, command_id)

    def dwell(self, command_id, dwell_time):
        """ Sleep for an amount of time.
            Input parameters:
                dwell_time: time the robot waits
        """
        line = "sleep({})\n".format(dwell_time)
        self.send_command(line, command_id)

    # =========================================================
    #                   Data write commands
    # =========================================================
    def set_workobject(self, x, y, z, roll, pitch, yaw):
        self.robot_settings.workobject = [[x, y, z],[roll, pitch, yaw]]

    def set_tcp(self, command_id, tcp):
        """ set robot flange to tool tip transformation
            Input parameters:
                tcp: The pose of the tcp measured from the robot flange.
        """
        line = "set_tcp(p[{},{},{},{},{},{}])\n".format(*tcp)
        self.send_command(line, command_id)

    def set_analog_output(self, output, val):
        """ Write analog output.
            Input parameter:
                int output: Number of analog output (0 or 1)
                double val: Range from 0 to 1 corresponds 0 to 24V
        """
        data = self.rtde.send_object
        data.__dict__["standard_analog_output_%i" % output] = val
        self.rtde.con.send(data)

    def set_payload(self, command_id, m, cog):
        """ Set Payload mass and center of gravity. The va
            Input Parameters:
                m: mass in kilograms
                cog: Center of Gravity, a vector [CoGx, CoGy, CoGz]
                    specifying the displacement (in meters) from the
                    tool mount.
        """
        line = "set_payload({}, [{},{},{}])\n".format(m, *cog)
        self.send_command(line, command_id)

    def popup(self, msg):
        self.dash.popup(msg)

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

    def activate_gripper(self, command_id, num=1):
        """ Activates the RobotIQ gripper. This function
            must be called before any other gripper command.
            Input parameter:
                num: Number of the gripper (1-4)
        """
        if 1 <= num <= 4:
            line = "rq_activate_and_wait({})\n".format(num)
            self.send_command(line, command_id)
        else:
            print("Gripper number must be between 1-4")

    def get_gripper_state(self, command_id):
        bits = self.rtde.get_digital_output_bits()
        if bits & 4 == 4:
            print("Gripper is open")
            return True
        else:
            print("Gripper is closed")
            return False

    def holds_object(self, command_id):
        time.sleep(7) # wait for the gripper to close
        bits = self.rtde.get_digital_output_bits()
        if bits & 16 == 16:
            print("Gripper is holding an object")
            return True
        else:
            print("Gripper is empty")
            return False

        # print("{0:b}".format(bits))

    def close_gripper(self, command_id):
        """ Moves the gripper to its fully closed position.
            Input parameter:
                num: Number of the gripper (1-4)
        """
        # if 1 <= num <= 4:
        line = "set_digital_out(3, True)\n"
        line3 = "set_digital_out(2, False)\n"
        line2 = "sleep(6)\n"
        # line = "rq_close_and_wait({})\n".format(num)
        self.send_command(line3, command_id)
        time.sleep(1)
        self.send_command(line, command_id)
        if self.live_mode == False:
            self.send_command(line2, command_id)
        else:
            time.sleep(4)
        #self.send_command(line2, command_id)
        # else:
        #    print("Gripper number must be between 1-4")

    def open_gripper(self, command_id):
        """ Moves the gripper to its fully opened position.
            Input parameter:
                num: Number of the gripper (1-4)
        """

        # line = "rq_open_and_wait({})\n".format(num)
        line = "set_digital_out(2, True)\n"
        line3 = "set_digital_out(3, False)\n"
        line2 = "sleep(6)\n"
        self.send_command(line3, command_id)
        time.sleep(1)
        self.send_command(line, command_id)
        if self.live_mode == False:
            self.send_command(line2, command_id)
        else:
            time.sleep(4)
        #self.send_command(line2, command_id)

    def move_gripper(self, command_id, position, num=1):
        """ Moves the gripper to the given position.
            Input parameter:
                position: Position the of the gripper in [mm]
                num: Number of the gripper (1-4)
        """
        # if 1 <= num <= 4:
        line = "rq_move_and_wait_mm({},{})\n".format(position, num)
        self.send_command(line, command_id)
        # else:
        #    print("Gripper number must be between 1-4")

    def set_gripper_force(self, command_id, force, num=1):
        """ Sets the gripping force.
            Input parameters:
                force: Value between 0 and 100. 0 = minimal force, 100 = maximal force.
                num: Number of the gripper (1-4)
        """
        if 1 <= num <= 4:
            line = "rq_set_force_norm({},{})\n".format(force, num)
            self.send_command(line, command_id)
        else:
            print("Gripper number must be between 1-4")

    def set_gripper_speed(self, command_id, speed, num=1):
        """ Sets the gripping force.
            Input parameters:
                force: Value between 0 and 100. 0 = minimal force, 100 = maximal force.
                num: Number of the gripper (1-4)
        """
        if 1 <= num <= 4:
            line = "rq_set_speed_norm({},{})\n".format(speed, num)
            self.send_command(line, command_id)
        else:
            print("Gripper number must be between 1-4")
