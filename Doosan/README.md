# Instructions to test the plugin
- clone the repo and cd into the Doosan folder
- create the docker image with : `docker build -t doosanplugin .` # I had to modify the name to not contain capital letters
- start the docker image with: `docker run -it --rm --name doosan --add-host host.docker.internal:host-gateway --mount type=bind,source="$(pwd)"/,target=/home/samy/src/ doosanplugin` # With the --add-host host.docker.internal:host-gateway you can connect to the SAMYCore running in your device with the address host.docker.internal
- this will start the container and mount the Doosan folder into the container
- Within the container, change to build folder with `cd build`
- now build the plugin with cmake .. and make
- run the plugin with ./Doosan [ip of samy core] [Name of robot in samy core] [address of robot] (If SAMYCore is running in your local host one can use for example ./Doosan host.docker.internal RobotUR5 123456)
- the plugin is not connectiong to a robot yet so the address can be some number.
