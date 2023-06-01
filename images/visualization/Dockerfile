FROM ubuntu:22.04

ENV TZ=Europe/Stockholm
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y \
        build-essential \
        cmake \
        libsparsehash-dev \
        python3-pip \
        libboost-dev \
        libboost-all-dev \
        net-tools \
        python3-tk \
	octovis \
        && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
        hyperopt\
        matplotlib\
        numpy \
        pandas \
        PyYAML \
        scipy \ 
        tabulate \
        pygame

