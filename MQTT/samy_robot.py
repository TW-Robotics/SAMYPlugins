from pubsub import pub
import yaml
import time
import math3d as m3d
import numpy
import logging
from mqtt import Mqtt
import paho.mqtt.client as mqtt
import threading

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

    def popup(self, data):
        self.logger.info("Got Message")

    def get_status(self, data):
        self.logger.info("Writing status in information source nodes")
        pub.sendMessage("write_information_source", name="LightBarrier", data=bool(self.mqtt.lightbarrier_value))
        pub.sendMessage("write_information_source", name="DistanceSensor", data=self.mqtt.distance_value)


    