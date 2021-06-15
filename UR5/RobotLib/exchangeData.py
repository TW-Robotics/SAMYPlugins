"""@package docstring
Python library to control an UR robot through its TCP/IP interface
RTDE
"""

import sys
import logging
import time
from rtde import rtde
from rtde import rtde_config


class ExchangeData():
    """ This class provides functions to receive data from the robot over
        the RTDE inteface.
    """

    def __init__(self, host):
        self.host = host
        self.port = 30004
        self.frequency = 125
        self.logger = logging.getLogger("rtde_logger")

        # load the config file to specify which data is to be received.
        self.conf = rtde_config.ConfigFile("RobotLib/RTDE_conf.xml")
        self.output_names, self.output_types = self.conf.get_recipe("out")
        self.input_names, self.input_types = self.conf.get_recipe("in")
        self.stop_bit_names, self.stop_bit_types = self.conf.get_recipe("stop_bit")

        # connect to the RTDE interface
        self.con = rtde.RTDE(self.host, self.port)
        self.con.connect()

        # get controller version
        self.con.get_controller_version()

        # send the configuration to the robot to setup recipes
        try:
            self.con.send_output_setup(self.output_names, self.output_types, frequency=self.frequency)
        except:
            self.logger.error('Unable to configure output')
            sys.exit()
        try:
            self.send_object = self.con.send_input_setup(self.input_names, self.input_types)
        except:
            self.logger.error("Unable to configure input")
            sys.exit()
        try:
            self.stop_object = self.con.send_input_setup(self.stop_bit_names, self.stop_bit_types)
        except:
            self.logger.error("Unable to configure stop bit")
            sys.exit()

        # start data synchronization
        if not self.con.send_start():
            self.logger.error('Unable to start synchronization')
            sys.exit()

    def _read(self):
        """ receives a rtde dataObject from the robot and returns the dataObject.
        """
        try:
            state = self.con.receive()
            return state
        except:
            print("No connection to Robot(RTDE)")

    def get_actual_tcp(self):
        """ Reads the actual TCP pose from the robot and returns it.
        """
        state = self._read()
        pose = state.__dict__["actual_TCP_pose"]
        return pose

    def get_target_q(self):
        """ Reads the Joints angles for the targeted position
            retun:
                actual_q: The actual joint angles in [rad]S
        """
        state = self._read()
        target_q = state.__dict__["target_q"]
        return target_q

    def get_actual_q(self):
        """ Reads the actual Joints angels.
            Return Value: Joint angles as list starting with the base joint.
        """
        state = self._read()
        actual_q = state.__dict__["actual_q"]
        return actual_q

    def get_runtime_state(self):
        """ Reads the runtime_state from the robot and returns it.
            Possible return values are: (as strings)
            - STOPPING
            - STOPPED
            - RUNNING
            - PAUSING
            - PAUSED
            - RESUMING
            - RETRACTING
        """
        states = {
            0: "STOPPING",
            1: "STOPPED",
            2: "RUNNING",
            3: "PAUSING",
            4: "PAUSED",
            5: "RESUMING",
            6: "RETRACTING"
        }

        state = self._read()  # get up to date data Robot
        run_state = state.__dict__["runtime_state"]  # read the runtime_state in the data
        if run_state in states:  # check if the runtime_state is valid
            return states.get(run_state)  # return the runtime_state as string
        else:
            self.logger.error("Unknown runtime state!")
            self.close()

    def get_digital_output_bits(self):
        state = self._read()
        bits = state.__dict__["actual_digital_output_bits"]
        return bits

    def write_bit_int(self, bit, num):
        """ Writes to a bit of the input_int_register in the robot controller.
            Parameters:
            bit: Value of the bit that is sended to the Robot.
            num: Number of the input_int_register.
        """
        register = "input_int_register_" + num  # geneerate the string that identivies the int register
        self.stop_object.__dict__[
            register] = bit  # set the bit in the data object that is going to be send to the robot
        self.con.send(self.stop_object)  # send the data object to the robot

    def close(self):
        """ Pauses and then closes the RTDE connection.
        """
        self.con.send_pause()
        self.con.disconnect()
