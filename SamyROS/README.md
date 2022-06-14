# SamyROS-Plugin for Doosan Robots

In the following ReadME a description is given, to start the plugin for the **h2017** or **m0617** doosan robot.

> Notice: In order to get the plugin working you need **docker** as well as **docker-compose** installed on your machine.
> The plugin also needs a connection to the **samycore**. Best option for testing is to startup the samycore in a separate docker container which can communicate with the plugin.

1. Clone the SamyPlugin repository from Github with: 
```console
$ git clone https://github.com/TW-Robotics/SAMYPlugins.git
```

2. Go into the root SamyRos folder which is containing the Dockerfile
3. Build the docker container for the SamyRos Plugin with:
```console
$ sudo docker build -t samyros .
```
> Notice: Don't forget the point at the end in the command before!

> Notice: The SamyRos plugin is embedded in the **docker-compose.yaml** file. The file assumes that there is a docker image for the samycore which is called **samycore**.

4. Edit the docker-compose.yaml file so that the plugin can communicate with the robot:
    * In line 14 of the docker-compose.yaml after "start.sh" there are three arguments seperated with whitespaces
    * The first argument is the ip-adress of the robot
    > Notice: If the robot is simulated the ip adress is always 127.0.0.1
    * The second argument is the port of the robot *(default: 12345)*
    * The third argument is the mode of the robot *(virtual for simulation, real for real robot)*

5. Start the docker-compose file with:
```console
$ docker-compose up
```

This starts the plugin and connects to the samycore.
