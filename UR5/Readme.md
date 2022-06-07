# UR5 Plugin
This is a plugin for the UR5 Robot for the SAMY framework.
The plugin uses the Python SAMYplugin template.

# Creating the docker image
The SAMY framework is designd to run as docker images. 
To create the docker image for the UR5 Plugin run:
```
docker build -t ur5-plugin .
```
This will create the docker image with the name ur5-plugin. With this name it can be used in a docker-compose file.
