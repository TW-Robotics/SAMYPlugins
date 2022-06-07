# Ros SAMY Plugin with doosan Robots and MoveIt
start with docker-compose up
In the docker-compose.yaml set in line 15 after "start.sh" two arguments.
The first argument is the ip address of the robot(real robot).
The second argument is a parameter if the robot used is real or virtual. To use with a real robot type real. To use with the simulation type virtual.
To change the config files for the plugin an the core visit http://localhost:8000 .
