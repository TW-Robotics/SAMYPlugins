import sys
import time
import subprocess
import roslaunch
from samyplugin import Plugin
from samyros import Samyros
from pubsub import pub

if __name__ == "__main__":

    plugin_object = Plugin()

    if len(sys.argv) < 3:
        print("To few arguments:")
        print("\"address of SAMYCore\" \"RobotName\" Optional: \"Path to settings\"")
        sys.exit(1)
    elif len(sys.argv) > 3:
        plugin_object.set_settings_path(sys.argv[3])
        
    robot_settings = plugin_object.get_robot_settings(sys.argv[2])
    robot_object = Samyros(robot_settings)

    plugin_object.connect_to_core(sys.argv[1], 4840)
    plugin_object.get_information_source_nodes()
    plugin_object.subscribe_to_core(sys.argv[2])

    plugin_object.disconnect_core()
    print("Good Bye...")
    
