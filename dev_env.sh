#!/bin/bash

COMMAND=$1
IMAGE=$2
EXTRA="${@:3}"

IMAGE_PATH=images/$IMAGE
IMAGE_NAME="$IMAGE:$USER"
CONTAINER_NAME="dev-$USER-$IMAGE"

if [[ "$COMMAND" = "help" || "$COMMAND" = "-h" || "$COMMAND" = "--help" || "$COMMAND" = "" ]]; then
    echo "Usage: $0 COMMAND [OPTION...] IMAGE [OPTION...] "
    echo " Commands:"     
    echo "   build          - Build the container."
    echo "   start          - Start the container (builds it if it has changed)."
    echo "   bash           - Access the running container from the terminal."
    echo " Images:"     
    echo "   dep          - Dynamic exploration planner"
    echo "   aep          - Autonomous exploration planner"
    echo "   nbvp         - Next best view planner"
    echo "   daep         - Dynamic Autonomous exploration planner"		
    echo "   or any other image in images/"
    exit 0
fi


if [[ "$COMMAND" = "build" ]]; then
    docker build -q $IMAGE_PATH -t $IMAGE_NAME
    exit 0
fi

if [[ "$COMMAND" = "start" ]]; then
    docker run -it \
      --rm \
      --name $CONTAINER_NAME \
      --volume=/tmp/.X11-unix/:/tmp/.X11-unix/\
      --volume=$(pwd)/$IMAGE:/home/$IMAGE \
      --workdir="/home/$IMAGE" \
      --env HOME="/home/$IMAGE" \
      --env DISPLAY=$DISPLAY \
      --gpus all \
      --privileged \
      --env=NVIDIA_VISIBLE_DEVICES=all \
      --env=NVIDIA_DRIVER_CAPABILITIES=all \
      --env=QT_X11_NO_MITSHM=1 \
      --net host \
      $IMAGE_NAME /bin/bash \
      $EXTRA 
    exit 0
fi


if [[ "$COMMAND" = "bash" ]]; then
    docker exec -it $CONTAINER_NAME /bin/bash
    exit 0
fi


############## USED IN SIMULATION LOOP ################
if [[ "$COMMAND" = "exec" ]]; then
    docker exec -it $CONTAINER_NAME /bin/bash $EXTRA
    exit 0
fi  


if [[ "$COMMAND" = "topic" ]]; then
    docker exec $CONTAINER_NAME /bin/bash $EXTRA
    exit 0
fi  
#######################################################
