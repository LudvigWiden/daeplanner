cd
source .bashrc
octomap_name=$1
mkdir -p octomaps
rosservice call /firefly/nbvPlanner/save_map "/home/nbvp_dep/octomaps/$octomap_name"
