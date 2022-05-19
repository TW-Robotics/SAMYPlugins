import time

from pubsub import pub
from mir_library.MiR import MiR_Robot
from samyplugin import CRCL_DataTypes
import json

class Robot():
    def __init__(self, settings):
        self.robot = MiR_Robot(settings["Address"], settings["AuthKey"])
        self.robot.checkConnection()
        #pub.subscribe(self.test, "Message")
        pub.subscribe(self.start_mission, "Message")

    def test(self, data):
        print('im program')
        time.sleep(5)
        
    def start_mission(self, data):
        print('str(data.Message): ', str(data.Message))
        self.robot.postToMissionQueue(str(data.Message))

        while 1:
            time.sleep(1)
            status = robot.getRegisters(1)
            #print(status)
            if (status == 1):
                break

        #while 1:
        #    if(robot.waitForMiR() == 1):
        #        break
        #    time.sleep(0.3)
        #    if(robot.waitForMiR() == -1):
        #        print('MiR - if self.checkStatus() != "Ready"')
        #        break
