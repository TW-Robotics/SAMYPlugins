import paho.mqtt.client as mqtt
from pubsub import pub
import logging
from pprint import pprint
import json
import subprocess 
from time import sleep
import requests

class Mqtt():
    def __init__(self, address, port, global_settings):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        log_handler = logging.StreamHandler()
        log_handler.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(log_handler)

        self.address = address
        self.global_settings = global_settings
        # Create MQTT client
        self.client = mqtt.Client()
        self.start_mqtt_server()
        # Configure the publishers on the IOLink Master
        self.configure_io_link_master(global_settings["IOLinkAddress"])
        # Connect callack functions with mqtt client
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_log = self.on_log
        # Connect to the MQTT Server
        self.client.connect(address, port, 60)
        # Variables to store the sensor values
        self.lightbarrier_value = None
        self.distance_value = None
        # Start non blocking mqtt loop
        self.client.loop_start()
        self.logger.info("MQTT loop startet")

    # This method reads the information about the subscriptions from the global settings (yaml files)
    # and uses the http interface of the IO-Link Master to configure the subscriptions on the IO-Link Master
    def configure_io_link_master(self, address):
        self.logger.info("Configuring subscriptions on the IO_Link Master.")
        url = "http://" + address
        for sub in self.global_settings["MQTT"]:
            obj = '{"adr": "00-02-01-68-84-e3/timer[1]/counter/datachanged/subscribe", "data" : {"callback" : "mqtt://' + self.address + ':2222/' + sub["Topic"] + '", "datatosend" : ["' + sub["Data"] + '"]}, "duration" : "lifetime", "cid" : 1, "code" : 10}'
            self.logger.info(obj)
            data = json.loads(obj)
            x = requests.post(url, json = data)
            self.logger.info(x.text)

    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(self, client, userdata, flags, rc):
        self.logger.debug("Connected with result code "+str(rc))
        self.logger.debug("Response flags: {}".format(flags))

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        # Subscribe to mqtt topics
        self.client.subscribe("sensors/#", 0)
        self.client.message_callback_add(self.global_settings["MQTT"][0]["Topic"], self.on_message_lightbarrier)
        self.client.message_callback_add(self.global_settings["MQTT"][1]["Topic"], self.on_message_distance)

    def on_subscribe(self, client, userdata, flags, rc):
        self.logger.info("MQTT-Subscribtion sucessfull")

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client, userdata, msg):
        self.logger.info("Got message that is not handled by a specific callback for that topic")
        self.logger.info("Received message '" + str(msg.payload) + "' on topic '"
        + msg.topic + "' with QoS " + str(msg.qos))

    def on_log(self, client, obj, level, string):
        self.logger.debug(string)

    def publish(self, topic:str, message:str):
        self.client.publish(topic, payload=message, qos=0, retain=False)

    # Starting the mqtt server using mosquitto
    def start_mqtt_server(self):
        mosquitto_server = subprocess.Popen(["mosquitto", "-p", "2222"])
        # Wait for the mqtt server to start
        sleep(2)

################### message handlers ########################
    
    # The callback when a message on the topic sensors/lightbarrier is received
    def on_message_lightbarrier(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.lightbarrier_value = data["data"]["payload"]["/iolinkmaster/port[1]/pin2in"]["data"]
        #pub.sendMessage("write_information_source", name="LightBarrier", data=bool(sensor_value))

    # The callback when a message on the topic sensors/distance is received
    def on_message_distance(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        sensor_value = data["data"]["payload"]["/iolinkmaster/port[3]/iolinkdevice/pdin"]["data"]
        self.distance_value = int(sensor_value[:4], 16) / 10
        #print("Hex: " + str(sensor_value[:4]) + "  Decimal: " + str(sensor_value_int))
        #pub.sendMessage("write_information_source", name="DistanceSensor", data=sensor_value_int)