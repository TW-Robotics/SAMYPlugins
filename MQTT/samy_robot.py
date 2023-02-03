from pubsub import pub
import yaml
import time
import math3d as m3d
import numpy
import logging
from mqtt import Mqtt
#import paho.mqtt.client as paho_mqtt
import threading
from opcua import ua

class SAMY_Robot():
    def __init__(self, global_settings):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        log_handler = logging.StreamHandler()
        log_handler.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(log_handler)

        self.global_settings = global_settings
        self.mqtt = Mqtt(global_settings["Address"], global_settings["MQTTPort"], global_settings)

        # Subscribe to Topics
        self.logger.info("Subscribing to topics")
        pub.subscribe(self.popup, "Message")
        pub.subscribe(self.get_status, "GetStatus")
        pub.subscribe(self.set_actuators, "ActuateJoints")

    def popup(self, data):
        self.logger.info("Got Message")

    def get_status(self, data):
        self.logger.info("Writing status in information source nodes")
        for sensor in self.global_settings["Sensors"]:
            response = self.mqtt.pull_data(self.global_settings["IOLinkAddress"], sensor["Port"])
            if sensor["Type"] == "DI":
                if response["value"] == "00":
                    #dv = ua.DataValue(ua.Variant(0, ua.VariantType.Int32))
                    pub.sendMessage("write_information_source", name=sensor["Name"], data=False) # False
                else:
                    #dv = ua.DataValue(ua.Variant(1, ua.VariantType.Int32))
                    pub.sendMessage("write_information_source", name=sensor["Name"], data=True) # True
            elif sensor["Type"] == "IO-Link":
                value = int(response["value"][:4], 16) / 10
                pub.sendMessage("write_information_source", name=sensor["Name"], data=value)
            else:
                self.logger.error("Sensor type not valid!!")

        time.sleep(1)



        #pub.sendMessage("write_information_source", name="LightBarrier", data=bool(self.mqtt.lightbarrier_value))
        #pub.sendMessage("write_information_source", name="DistanceSensor", data=self.mqtt.distance_value)
        #pub.sendMessage("write_information_source", name="InductiveSensor", data=self.mqtt.inductive_value)

    def set_actuators(self, data):
        for joint in data.actuateJoint:
            if joint.jointPosition > 0:
                self.logger.info("Setting Port {} to True".format(joint.jointNumber))
                self.mqtt.set_DO(self.global_settings["IOLinkAddress"], joint.jointNumber, 1)
            else:
                self.logger.info("Setting Port {} to False".format(joint.jointNumber))
                self.mqtt.set_DO(self.global_settings["IOLinkAddress"], joint.jointNumber, 0)
            time.sleep(1)
        