
from opcua import Server
from opcua import Client
from IPython import embed
from opcua import ua
import time
import CRCL_DataTypes

def getNamespaces(client):
    namespaces = {}
    root_node = client.get_root_node()
    browse_path = ["0:Objects", "0:Server", "0:NamespaceArray"]
    namespaceArrayNode = root_node.get_child(browse_path)
    namespacesValue = namespaceArrayNode.get_value()
    for i, namespace in enumerate(namespacesValue):
        namespaces[namespace] = i
    return namespaces

def create_opcua_server(opcua_server, idx):
    opcua_server = Server()
    opcua_server.set_endpoint("opc.tcp://0.0.0.0:4842/PLC/")
    opcua_server.set_server_name("PLC sim Server")

    uri = "http://examples.freeopcua.github.io"
    uri2 = "http://opcfoundation.org/UA/Robotics/"
    idx = opcua_server.register_namespace(uri)
    idx2 = opcua_server.register_namespace(uri2)

    return opcua_server, idx2

def connect_to_core(address, port):
    # Connect to SAMYCore opcua server
    while True:
        try:
            # Create opcua client connection with SAMYCore
            address = ("opc.tcp://{}:{}").format(address, port)
            core_client = Client(address)
            core_client.connect()
            print("Conected to Core")
            break
        except:
            time.sleep(3)
    # Get information about all nodes of SAMYCore opcua server
    core_client.load_type_definitions()
    return core_client

if __name__ == "__main__":

    idx = None
    opcua_server = None
    core_client = None

    ur5_camera_offset = [0.050, 0.060, 0.754]
    abb_camera_offset = [0.060-0.015, 0.015+0.045, 0.760-0.010] # [0.060, 0.015, 0.760]

    opcua_server, idx = create_opcua_server(opcua_server, idx)
    core_client = connect_to_core("core", "4840")
    #core_client = connect_to_core("localhost", "4840")

    namespaces = getNamespaces(core_client)
    ns_skills = namespaces["http://SAMY.org/SAMYSkills"]
    ns_ur5 = namespaces["RobotUR5"]
    ns_abb = namespaces["RobotABB"]
    print("namespaces")
    print(namespaces)
    inf_source_browname = str(namespaces['http://SAMY.org/InformationSources']) + ':InformationSources'
    print("\n")
    print("inf_source_browsepath")
    print(inf_source_browname)
    browse_path_inf_source = ["0:Objects", inf_source_browname]

    robot_name = "RobotUR5"
    brose_path_database_poses = ["0:Objects", "3:DeviceSet", "{}:{}".format(ns_ur5, robot_name), "4:Controllers", "{}:{}".format(ns_ur5, robot_name), "5:Skills"]
    path_pose_ur5 = brose_path_database_poses.copy()
    path_pose_ur5.append("{}:PickAndPlaceSkill".format(ns_ur5))
    path_pose_ur5.append("3:ParameterSet")
    path_pose_ur5_above = path_pose_ur5.copy()
    path_pose_ur5_above2 = path_pose_ur5.copy()
    path_pose_ur5.append("{}:4_MoveToParameters".format(ns_skills))
    path_pose_ur5_above.append("{}:2_MoveToParameters".format(ns_skills))
    path_pose_ur5_above2.append("{}:7_MoveToParameters".format(ns_skills))
    print(path_pose_ur5)
    print(path_pose_ur5_above)
    print(path_pose_ur5_above2)

    robot_name = "RobotABB"
    brose_path_database_poses = ["0:Objects", "3:DeviceSet", "{}:{}".format(ns_abb, robot_name), "4:Controllers", "{}:{}".format(ns_abb, robot_name), "5:Skills"]
    path_pose_abb = brose_path_database_poses.copy()
    path_pose_abb.append("{}:PickAndPlaceSkill".format(ns_abb))
    path_pose_abb.append("3:ParameterSet")
    path_pose_abb_above = path_pose_abb.copy()
    path_pose_abb_above2 = path_pose_abb.copy()
    path_pose_abb.append("{}:4_MoveToParameters".format(ns_skills))
    path_pose_abb_above.append("{}:2_MoveToParameters".format(ns_skills))
    path_pose_abb_above2.append("{}:7_MoveToParameters".format(ns_skills))
    print("path_pose_abb")
    print(path_pose_abb)
    print("path_pose_above")
    print(path_pose_abb_above)

    ns_info_sources = namespaces["http://SAMY.org/InformationSources"]
    path_camera_type = browse_path_inf_source.copy()
    path_camera_type.append("{}:CameraDetectionString".format(ns_info_sources))
    path_camera_type.append("{}:CameraDetectionString_0".format(ns_info_sources))
    print(path_camera_type)

    path_light_barrier = browse_path_inf_source.copy()
    path_light_barrier.append("{}:LightBarrier-Active".format(ns_info_sources))
    path_light_barrier.append("{}:LightBarrier-Active_0".format(ns_info_sources))
    print(path_light_barrier)

    path_conveyor_holder = browse_path_inf_source.copy()
    path_conveyor_holder.append("{}:ConveyorHolder-Active".format(ns_info_sources))
    path_conveyor_holder.append("{}:ConveyorHolder-Active_0".format(ns_info_sources))
    print(path_conveyor_holder)

    path_conveyor_parts = browse_path_inf_source.copy()
    path_conveyor_parts.append("{}:ConveyorPart-Active".format(ns_info_sources))
    path_conveyor_parts.append("{}:ConveyorPart-Active_0".format(ns_info_sources))
    print(path_conveyor_parts)

    path_cylinder = browse_path_inf_source.copy()
    path_cylinder.append("{}:Cylinder-Active".format(ns_info_sources))
    path_cylinder.append("{}:Cylinder-Active_0".format(ns_info_sources))
    print(path_cylinder)

    # Get nodes with defined paths
    root_node = core_client.get_root_node()
    print("Get camera_type_node")
    camera_type_node = root_node.get_child(path_camera_type)
    print("Get pose_abb_node")
    pose_abb_node = root_node.get_child(path_pose_abb)
    print("Get pose_ur5_node")
    pose_ur5_node = root_node.get_child(path_pose_ur5)
    print("Get pose_abb_above_node")
    pose_abb_above_node = root_node.get_child(path_pose_abb_above)
    print("Get pose_ur5_above_node")
    pose_ur5_above_node = root_node.get_child(path_pose_ur5_above)
    print("Get pose_abb_above2_node")
    pose_abb_above2_node = root_node.get_child(path_pose_abb_above2)
    print("Get pose_ur5_above2_node")
    pose_ur5_above2_node = root_node.get_child(path_pose_ur5_above2)
    print("Get light_barrier_node")
    light_barrier_node = root_node.get_child(path_light_barrier)
    print("Get conveyor_holder_core_node")
    conveyor_holder_core_node = root_node.get_child(path_conveyor_holder)
    print("Get conveyor_parts_core_node")
    conveyor_parts_core_node = root_node.get_child(path_conveyor_parts)
    print("Get cylinder_core_node")
    cylinder_core_node = root_node.get_child(path_cylinder)

    # Add nodes to the PLC server
    objects = opcua_server.get_objects_node()
    plc_object = objects.add_object(idx, "PLC_Nodes")
    cylinder_node = plc_object.add_variable(idx, "Cylinder", False)
    conveyor_parts_node = plc_object.add_variable(idx, "ConveyorParts", False)
    conveyor_holder_node = plc_object.add_variable(idx, "ConveyorHolder", False)
    sensor_node = plc_object.add_variable(idx, "Sensor", False)
    camera_x_node = plc_object.add_variable(idx, "CameraX", 0.0)#ua.Variant(0.0, ua.VariantType.Double))
    camera_y_node = plc_object.add_variable(idx, "CameraY", 0.0)#ua.Variant(0.0, ua.VariantType.Double))
    camera_z_node = plc_object.add_variable(idx, "CameraZ", 0.0)#ua.Variant(0.0, ua.VariantType.Double))
    camera_object_node = plc_object.add_variable(idx, "CameraObject", "")
    cylinder_node.set_writable()
    conveyor_parts_node.set_writable()
    conveyor_holder_node.set_writable()
    sensor_node.set_writable()
    camera_x_node.set_writable()
    camera_y_node.set_writable()
    camera_z_node.set_writable()
    camera_object_node.set_writable()

    ur5_pose = CRCL_DataTypes.MoveToParametersSetDataType()
    ur5_pose.EndPosition.xAxis.i = -1
    ur5_pose.EndPosition.xAxis.j = 0
    ur5_pose.EndPosition.xAxis.k = 0
    ur5_pose.EndPosition.zAxis.i = 0
    ur5_pose.EndPosition.zAxis.j = 0
    ur5_pose.EndPosition.zAxis.k = -1
    abb_pose = CRCL_DataTypes.MoveToParametersSetDataType()
    abb_pose.EndPosition.xAxis.i = -1
    abb_pose.EndPosition.xAxis.j = 0
    abb_pose.EndPosition.xAxis.k = 0
    abb_pose.EndPosition.zAxis.i = 0
    abb_pose.EndPosition.zAxis.j = 0
    abb_pose.EndPosition.zAxis.k = -1
    opcua_server.start()

    # TODO change to event based update mechanism
    while True:
        camera_type_node.set_value(camera_object_node.get_value())
        ur5_pose.EndPosition.point.x = camera_x_node.get_value() + ur5_camera_offset[0]
        ur5_pose.EndPosition.point.y = -camera_y_node.get_value() + ur5_camera_offset[1]
        ur5_pose.EndPosition.point.z = -camera_z_node.get_value() + ur5_camera_offset[2]
        pose_ur5_node.set_value(ur5_pose)
        abb_pose.EndPosition.point.x = camera_x_node.get_value() + abb_camera_offset[0]
        abb_pose.EndPosition.point.y = -camera_y_node.get_value() + abb_camera_offset[1]
        abb_pose.EndPosition.point.z = -camera_z_node.get_value() + abb_camera_offset[2]
        pose_abb_node.set_value(abb_pose)
        ur5_pose.EndPosition.point.x = camera_x_node.get_value() + ur5_camera_offset[0]
        ur5_pose.EndPosition.point.y = -camera_y_node.get_value() + ur5_camera_offset[1]
        ur5_pose.EndPosition.point.z = -camera_z_node.get_value() + ur5_camera_offset[2] + 0.100
        pose_ur5_above_node.set_value(ur5_pose)
        pose_ur5_above2_node.set_value(ur5_pose)
        abb_pose.EndPosition.point.x = camera_x_node.get_value() + abb_camera_offset[0]
        abb_pose.EndPosition.point.y = -camera_y_node.get_value() + abb_camera_offset[1]
        abb_pose.EndPosition.point.z = -camera_z_node.get_value() + abb_camera_offset[2] + 0.100
        pose_abb_above_node.set_value(abb_pose)
        pose_abb_above2_node.set_value(abb_pose)
        light_barrier_node.set_value(sensor_node.get_value())
        conveyor_parts_core_node.set_value(conveyor_parts_node.get_value())
        conveyor_holder_core_node.set_value(conveyor_holder_node.get_value())
        cylinder_core_node.set_value(cylinder_node.get_value())
        # print('light_barrier_node.get_value  ', sensor_node.get_value())
        # print('conveyor_parts.get_value  ', conveyor_parts_node.get_value())
        # print('conveyor_holder.get_value  ', conveyor_holder_node.get_value())
        # print('cylinder_core_node.get_value  ', cylinder_node.get_value())
        time.sleep(0.1)

    #embed()
