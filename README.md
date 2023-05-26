# dynamicexploration
Each folder contains a docker file used to run a specific planner.
Each Dockerfile specifies the correct OS- and ROS-version to be used and downloads the right packages.

To setup the environt run:
`cd /path/to/repo`
followed by
`nano dev_env.sh`
and change the `IMAGE` variable to the name of one of the folders.

Then run:
`./dev_env.sh build`
followed by
`./dev_env.sh start`

If your user is `root`, you have succeded.

## cudagl16_aep
This folder contains the Docker file used to run the AEP planner.

## cudagl18_dep
This folder contains the Docker file used to run the DEP planner.

## cudagl18_eth
This folder contains the Docker file used to run the ETH Informative planner.

## ros_indigo_nvidia_nbvp
This folder contains the Docker file used to run the RH-NBV planner.

## Unreal Engine 5
If you like to run Unreal Engine 5, use the unreal5.sh to setup the environment.
First run 
`./unreal5.sh load /path/to/unreal5.tar`
to load in the Unreal Engine 5 image from a .tar file. Then
`./unreal5.sh start`
to start the container. To start the editor run
`./UnrealEditor`

##Troubleshooting
If you get the error saying **No available video device** when running ./UnrealEditor, type
`exit`
to exit the docker container and then run
`xhost + local:docker`
and then run the script again and try to start the UnrealEditor
