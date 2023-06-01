FROM nvidia/cudagl:11.3.0-base-ubuntu18.04
# Kom ihåg att ändra från c++11 till c++14 i cmake i DEP mapp
ENV TZ=Europe/Stockholm
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y \
        build-essential \
        cmake \
        libsparsehash-dev \
        python-pip \
        libboost-dev \
        libboost-all-dev \
        net-tools \
        nano \
        wget \
        sudo \
        unzip \
        curl \
        rsync \
        git \
        python-pip \
        lsb-release \
      && rm -rf /var/lib/apt/lists/*


# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Setup sources
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
      ros-melodic-desktop-full \
      python-rosinstall \
      python-rosinstall-generator \
      python-rosdep \
      python-wstool \
      libtool \
      libyaml-cpp-dev  \
      libgoogle-glog-dev \
      liblapacke-dev \
      libgeographic-dev \
      ros-melodic-rqt-graph \
      ros-melodic-rqt-gui \
      ros-melodic-rqt-plot \
      ros-melodic-kobuki-core \
      ros-melodic-kobuki-dock-drive \
      ros-melodic-kobuki-driver \
      ros-melodic-kobuki-ftdi \
      ros-melodic-kobuki-msgs \
      ros-melodic-mav-msgs \
      ros-melodic-mav-msgs \
      ros-melodic-mav-planning-msgs \
      ros-melodic-nmea-msgs \
      ros-melodic-roscpp-tutorials \
      ros-melodic-octomap-* \
      ros-melodic-cmake-modules \
      ros-melodic-control-toolbox \
      ros-melodic-joy \
      ros-melodic-octomap-ros \
      ros-melodic-mavlink \
      ros-melodic-geographic-msgs \
      python-catkin-tools \
      mesa-utils \
      autoconf \
      automake \
      gdb \
      gedit \
      xdg-user-dirs \
      protobuf-compiler \ 
      gnome-terminal \
      psmisc \
    && rm -rf /var/lib/apt/lists/* 

RUN rosdep init && \
    rosdep update

RUN pip install wstool future unrealcv

# AEP
RUN apt-get update && apt-get install -y \
      libspatialindex-dev \
      ros-melodic-octomap-* \
      && rm -rf /var/lib/apt/lists/*


# Test benchmark
RUN apt-get update \
  && apt-get install -y -qq glmark2



