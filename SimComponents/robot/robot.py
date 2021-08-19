
import logging
import os
import sys
import time
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

from pubsub import pub
import samyplugin.CRCL_DataTypes

from opcua import Server


class SubHandler(object):
    """
    Subscription Handler. To receive events from server for a subscription
    """

    def datachange_notification(self, node, val, data):
        print("Python: New data change event", node, val)


    def event_notification(self, event):
        print("Python: New event", event)


class Robot(object):
    # This class represents the robot.

    def __init__(self, host):
        # create a logger for the robot object
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        fh = logging.StreamHandler()
        fh.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(fh)

        self.host = host
        self.opcua_server = None
        self.idx = None
        self.cylinder_node = None
        self.conveyor_parts_node = None
        self.conveyor_holder_node = None
        self.sensor_node = None
        self.handler = None
        # Subscribe to the CRCL command topics
        pub.subscribe(self.start_device, "EnableGripperCommand")
        pub.subscribe(self.stop_device, "DisableGripperCommand")



    # Add robot specific methodes here:

    def create_opcua_server(self):
        self.opcua_server = Server()
        self.opcua_server.set_endpoint("opc.tcp://0.0.0.0:4840/PLC/")
        self.opcua_server.set_server_name("PLC sim Server")

        uri = "http://examples.freeopcua.github.io"
        self.idx = self.opcua_server.register_namespace(uri)

    def create_nodes(self):
        objects = self.opcua_server.get_objects_node()
        plc_object = objects.add_object(self.idx, "PLC_Nodes")
        self.cylinder_node = plc_object.add_variable(self.idx, "Cylinder", False)
        self.conveyor_parts_node = plc_object.add_variable(self.idx, "ConveyorParts", False)
        self.conveyor_holder_node = plc_object.add_variable(self.idx, "ConveyorHolder", False)
        self.sensor_node = plc_object.add_variable(self.idx, "Sensor", False)
        self.cylinder_node.set_writable()
        self.conveyor_parts_node.set_writable()
        self.conveyor_holder_node.set_writable()
        self.sensor_node.set_writable()

    def start_server(self):
        self.opcua_server.start()

    def subscribe_to_sensor(self):
        self.handler = SubHandler()
        sub = self.opcua_server.create_subscription(100, self.handler)
        handle = sub.subscribe_data_change(self.sensor_node)

    def start_device(self, data):
        if data.GripperName == "cylinder":
            self.cylinder_node.set_value(True)
            pub.sendMessage("command_finished")
        elif data.GripperName == "conveyor_parts":
            self.conveyor_parts_node.set_value(True)
            pub.sendMessage("command_finished")
        elif data.GripperName == "conveyor_holder":
            self.conveyor_holder_node.set_value(True)
            pub.sendMessage("command_finished")
        else:
            pub.sendMessage("command_error")

    def stop_device(self, data):
        if data.GripperName == "cylinder":
            self.cylinder_node.set_value(False)
            pub.sendMessage("command_finished")
        elif data.GripperName == "conveyor_parts":
            self.conveyor_parts_node.set_value(False)
            pub.sendMessage("command_finished")
        elif data.GripperName == "conveyor_holder":
            self.conveyor_holder_node.set_value(False)
            pub.sendMessage("command_finished")
        else:
            pub.sendMessage("command_error")
