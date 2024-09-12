source /opt/ros/melodic/setup.bash
source catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/gazebo_models/
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/drone_gazebo/models

export SVGA_VGPU10=0
