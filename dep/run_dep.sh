# 
#!/bin/bash

# PROBLEM WITH FUSE


LAUNCH=$1
WARMUP=$2
DOCKER="docker exec -it dev-ludwi-dep bash"

#### RUN ####
echo "RUNNING DEP"

# Open terminal 1
gnome-terminal --tab --title="Simulation" -- bash -c "echo Starting container; $DOCKER; echo Launch simulation; roslaunch drone_gazebo $LAUNCH"

# Wait for 5 seconds
sleep 5

# Open terminal 2
gnome-terminal --tab --title="Voxblox" -- bash -c "cho Starting container; $DOCKER; echo Launch voxblox; roslaunch voxblox_ros esdf.launch"

# Wait for 5 seconds
sleep 5

# Open terminal 3
gnome-terminal --tab --title="Warmup" -- bash -c "cho Starting container; $DOCKER; echo Warmup; rosrun drone_gazebo $2"

# Wait for 6 seconds
sleep 6

# Open terminal 4,
gnome-terminal --tab --title="DEP" -- bash -c "cho Starting container; $DOCKER; echo Running DEP; roslaunch DEP exploration.launch"

# Wait for 5 seconds
sleep 5

# Open terminal 5, run the publisher container
gnome-terminal --tab --title="Motion" -- bash -c "cho Starting container; $DOCKER; echo Running Motion; rosrun DEP move_and_rotate.py"


echo "Done"



