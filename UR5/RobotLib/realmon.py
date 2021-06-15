"""@package docstring
Python library to control an UR robot through its TCP/IP interface.

"""

import socket
import struct
import logging
import sys
import time


class RealTimeMonitor():
    """ This class connects to the
        Primary Monitor interface of a UR Robot.
    """


    # Struct for revision of the UR controller giving 692 bytes
    rtstruct692 = struct.Struct('>d6d6d6d6d6d6d6d6d18d6d6d6dQ')

    # for revision of the UR controller giving 540 byte. Here TCP
    # pose is not included!
    rtstruct540 = struct.Struct('>d6d6d6d6d6d6d6d6d18d')

    rtstruct5_1 = struct.Struct('>d1d6d6d6d6d6d6d6d6d6d6d6d6d6d6d1d6d1d1d1d6d1d6d3d6d1d1d1d1d1d1d1d6d1d1d3d3d')

    def __init__(self, host):
        """ Here the connection gets configured and established.
            First a socket is created and then the connection to the robot is started.
            Parameters:
                host: The IP address of the Robot
        """
        self.logger = logging.getLogger("sec_mon")
        self.host = host
        self.port = 30003
        self.real_socket = None

        try:
            self.real_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.real_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.real_socket.connect((self.host, self.port))
        except:
            self.logger.error("No connection to Robot!!!")
            sys.exit()

    def send(self, msg):
        """ It takes a string and sends it to the Primary Monitor interface
            of the UR Robot.
            Parameters:
            msg: The string that is send to the Robot.
        """
        msg = msg.encode()
        try:
            self.real_socket.send(msg)
        except:
            print("No connection to Robot(RealMon)")


    def __recv_bytes(self, nBytes):
        ''' Facility method for receiving exactly "nBytes" bytes from
        the robot connector socket.'''
        # Record the time of arrival of the first of the stream block
        recvTime = 0
        pkg = b''
        while len(pkg) < nBytes:
            pkg += self.real_socket.recv(nBytes - len(pkg))
            if recvTime == 0:
                recvTime = time.time()
        self.__recvTime = recvTime
        return pkg

    def get_actual_tcp(self):
        head = self.__recv_bytes(4)
        pkgsize = struct.unpack('>i', head)[0]
        #print('Received header telling that package is %s bytes long', pkgsize)
        payload = self.__recv_bytes(pkgsize - 4)
        if pkgsize >= 692:
            unp = self.rtstruct692.unpack(payload[:self.rtstruct692.size])
        elif pkgsize >= 540:
            unp = self.rtstruct540.unpack(payload[:self.rtstruct540.size])
        else:
            self.logger.warning(
                'Error, Received packet of length smaller than 540: %s ',
                pkgsize)
            return

        #self._qActual = np.array(unp[74:79])
        return list(unp[73:79])

    def close(self):
        """ It closes the connection to the Primary Monitor interface.
        """
        self.real_socket.close()
