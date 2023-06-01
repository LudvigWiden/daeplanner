cd
source .bashrc
world=$1
mode=$2
spawn_pos=$3
human_avoidance=$4
drone_avoidance=$5
echo $drone_avoidance


roslaunch drone_gazebo simulation.launch mode:=$mode world:=$world spawn_pos:=$spawn_pos avoidance:=$human_avoidance avoidance_mode:=$drone_avoidance

