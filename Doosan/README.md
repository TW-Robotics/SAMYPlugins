# Instructions to test the plugin
- clone the repo and cd into the Doosan folder
- create the docker image with : `docker build -t doosanPlugin .`
- start the docker image with: `docker run -it --rm --name doosan --mount type=bind,source="$(pwd)"/,target=/home/samy/src/ doosanPlugin`
- this will start the container and mount the Doosan folder into the container
- now build the plugin with cmake .. and make
- run the plugin with ./Doosan [ip of samy core] [Name of robot in samy core] [address of robot]
- the plugin is not connectiong to a robot jet so the address can be some number.
