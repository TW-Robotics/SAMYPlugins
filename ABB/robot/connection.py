
from opcua import Client
from samyplugin import CRCL_DataTypes

import logging

move_finished = False

class SubHandler(object):

    """
    Subscription Handler. To receive events from server for a subscription
    data_change and event methods are called directly from receiving thread.
    Do not do expensive, slow or network operation there. Create another
    thread if you need to do such a thing
    """

    def datachange_notification(self, node, val, data):
        global move_finished
        print("Python: New data change event", node, val)
        if val:
            move_finished = True

    def event_notification(self, event):
        print("Python: New event", event)


class OPCUA:
    def __init__(self, ip):
        link = ("opc.tcp://{}:61510/ABB.IRC5.OPCUA.Server").format(ip)
        self.client = Client(link) #"opc.tcp://192.168.178.21:61510/ABB.IRC5.OPCUA.Server"
        print(link)
        self.client.set_security_string("Basic256Sha256,SignAndEncrypt,key/certificate.der,key/key.pem")
        self.client.application_uri = "urn:example.org:FreeOpcUa:python-opcua"
        self.client.secure_channel_timeout = 100000
        self.client.session_timeout = 100000
        #self.client.open_secure_channel()
        self.client.connect()
        self.client.load_type_definitions()  # load definition of server specific structures/extension objects
        print("Python: OPC UA connected")
        # Create nodes
        self.root = self.client.get_root_node()
        self.objects = self.client.get_objects_node()
        uri = "http://opcfoundation.org/UA/DI/"
        idx = 2 # self.client.get_namespace_index(uri)
        alias_name = "IRB2400_1"
        #alias_name = "Robot_SIM"

        self.string_node = self.root.get_child(["0:Objects", "{}:IRC5".format(idx),
                                                "{}:{}".format(idx, alias_name), "{}:RAPID".format(idx),
                                                "{}:T_ROB1".format(idx), "{}:SERVER".format(idx),
                                                "{}:receivedString".format(idx)])
        self.wait_for_command_node = self.root.get_child(["0:Objects", "{}:IRC5".format(idx),
                                                "{}:{}".format(idx, alias_name), "{}:RAPID".format(idx),
                                                "{}:T_ROB1".format(idx), "{}:SERVER".format(idx),
                                                "{}:wait_for_command".format(idx)])
        self.command_done_node = self.root.get_child(["0:Objects", "{}:IRC5".format(idx),
                                                "{}:{}".format(idx, alias_name), "{}:RAPID".format(idx),
                                                "{}:T_ROB1".format(idx), "{}:SERVER".format(idx),
                                                "{}:command_done".format(idx)])
        self.currentTCP_node = self.root.get_child(["0:Objects", "{}:IRC5".format(idx),
                                                      "{}:{}".format(idx, alias_name), "{}:RAPID".format(idx),
                                                      "{}:reportTCP".format(idx), "{}:report_tcp".format(idx),
                                                      "{}:currentTCP".format(idx)])
        print("Python: OPC UA Nodes created")
        #except:
        #    print("Python: OPC UA connection failed")

    def subscription(self, client):
        handler = SubHandler()
        sub = client.create_subscription(500, handler)
        handle = sub.subscribe_data_change(self.command_done_node)

class Connection():
    def __init__(self, ipAddress):
        # create a logger for the robot object
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        fh = logging.StreamHandler()
        fh.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(fh)

        self.opcua = OPCUA(ipAddress)

    def send_opcua(self, msg):
        # Send the message as string over opcua
        send = False
        self.count = self.count + 1
        print(self.count)
        while not send:
            while not self.opcua.wait_for_command_node.get_value():
                time.sleep(0.001)
            try:
                self.opcua.string_node.set_value(msg)
                self.opcua.wait_for_command_node.set_value(False)
                send = True
            except:
                print("Sending Failed -> trying again")
        print("Python: Command send over OPC UA")

    def compare_pose(self, x, y, z):
        if self.live_mode == False:
            return True

        dif_pose = [10, 0, 0]
        pose_reached = False
        print("Python: comparing pose....")
        while not pose_reached:
            x_rob, y_rob, z_rob = self.get_cartesian()
            dif_pose[0] = (abs(x_rob) - abs(x))
            dif_pose[1] = (abs(y_rob) - abs(y))
            dif_pose[2] = (abs(z_rob) - abs(z))
            print("Dif Pose:")
            print(dif_pose)
            if abs(dif_pose[0]) < 0.5 and abs(dif_pose[1]) < 0.5 and abs(dif_pose[2]) < 0.5:
                print(dif_pose)
                pose_reached = True
                print("Python: Pose reached")
            time.sleep(0.1)
        return pose_reached
