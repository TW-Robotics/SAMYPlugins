import sys
import time
from IPython import embed

from robot import Robot
from samyplugin import Plugin
import tests

# Interface python3 main.py -IP SAMYCore- -Plugin Port- -IP Robot- -PluginName-

if __name__ == "__main__":

    if len(sys.argv) < 4:
        print("To few arguments:")
        print("\"address of SAMYCore\" \"port of SAMYCore\" \"address of robot\"")
        sys.exit(1)

    plugin_object = Plugin()
    robot_object = Robot(sys.argv[3])
    robot_object.create_opcua_server()
    robot_object.create_nodes()
    robot_object.start_server()
    robot_object.subscribe_to_sensor()


    plugin_object.connect_to_core(sys.argv[1], sys.argv[2])
    #plugin_object.subscribe_to_core()

    #testing = tests.Tests()

    plugin_object.disconnect_core()
