# Start mqtt server with mosquitto
mosquitto -p 2222

# Publish test message
mosquitto_pub  -p 2222 -t "Test/topic" -m Test

# subscribe to sensor message from mqtt io link 
mosquitto_sub -p 2222 -t sensors/lightbarrier -q 1
