#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

echo "Generating docs"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
for server in ha.pool.sks-keyservers.net \
              hkp://p80.pool.sks-keyservers.net:80 \
              keyserver.ubuntu.com \
              hkp://keyserver.ubuntu.com:80 \
              pgp.mit.edu; do
    sudo apt-key adv --keyserver "$server" --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && break || echo "Trying new server..."
done
sudo apt-get -y update
sudo apt-get -y install ros-noetic-rosdoc-lite

line=$(cat CMakeLists.txt | grep "project(.*)" -o); tmp=${line:8}; proj_name=${tmp:0:${#tmp}-1}; # parse the project name
source /opt/ros/noetic/setup.bash
rosdoc_lite . # generate the documentation
grep -rl "mrs_msgs/html" doc | xargs sed -i 's+../../../mrs_msgs/html/+../+g' # remove the html part of paths in the generated files
grep -rl "std_msgs/html" doc | xargs sed -i 's+../../../std_msgs/html/+http://docs.ros.org/noetic/api/std_msgs/html/+g' # remove the html part of paths in the generated files
grep -rl "geometry_msgs/html" doc | xargs sed -i 's+../../../geometry_msgs/html/+http://docs.ros.org/noetic/api/geometry_msgs/html/+g' # remove the html part of paths in the generated files
cd doc/html; ln -s index-msg.html index.html # link index.html to index-msg.html so that a browser opens that by default
