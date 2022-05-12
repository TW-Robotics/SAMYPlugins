# Start mqtt server with mosquitto
mosquitto -p 2222

# Publish test message
mosquitto_pub  -p 2222 -t "Test/topic" -m Test

# subscribe to sensor message from mqtt io link 
mosquitto_sub -p 2222 -t sensors/lightbarrier -q 1

# How ActuateJoints works
The parameter JointNumber describes wich port is controlled.
The parameter JointPosition sets the Value. At the moment only ports that are configured as DO can be controlled.
    - JointPosition > 0 -> Output on
    - JointPosition <= 0 -> Output off

# Adding a new IO-Link Sensor
1. Add Subscription in the RobotsConfig file
1. Add on_messageXX callback in mqtt.py 
1. Add callback to mqtt client
1. Add writing information source in getStatus CRCL command