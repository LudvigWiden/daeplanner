cd
source .bashrc
config=$1
roslaunch interface_nbvp_rotors exploration.launch config_file:=$config
