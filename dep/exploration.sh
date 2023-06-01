cd
source .bashrc
config=$1
no_replan=$2
roslaunch DEP exploration.launch config_file:=$config no_replan:=$no_replan
