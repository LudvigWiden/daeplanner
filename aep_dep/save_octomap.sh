cd
source .bashrc
octomap_name=$1
mkdir -p octomaps
rosrun octomap_server octomap_saver -f /home/aep_dep/octomaps/$octomap_name octomap_full:=/aeplanner/octomap_full
