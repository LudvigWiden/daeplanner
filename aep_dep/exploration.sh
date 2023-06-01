cd
source .bashrc
config=$1
roslaunch rpl_exploration exploration.launch config_file:=$config
