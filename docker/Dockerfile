
FROM nvidia/cuda:11.0.3-cudnn8-devel-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ros1_distro=noetic
###################################### user #####################################

ENV SHELL=/bin/bash \
    USER=user \
    UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER} 

RUN echo "root:root" | chpasswd
RUN echo "${USER}:nvgrasp" | chpasswd

## basic tools

RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    sudo \
    software-properties-common \
    git \
    cmake \
    wget \
    make \
    vim \
    build-essential \
    libblkid-dev \
    e2fslibs-dev \
    libboost-all-dev \
    libaudit-dev \
    tzdata \
    python3-pip

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration
RUN pip3 install --upgrade pip
## ROS1 installation

# Install ROS 1
RUN apt update && sudo apt install -y curl gnupg2 lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update
RUN apt install -y ros-$ros1_distro-desktop-full
 
# Install ROS 1 build tools
RUN apt install -y python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
 
RUN echo "ROS $ros1_distro installed successfully"

## Dependency installation
COPY ./package_apt.txt /
COPY ./requirement_pip.txt /
RUN apt update && xargs -a /package_apt.txt apt install -y
RUN pip3 install -r /requirement_pip.txt
RUN rm /package_apt.txt /requirement_pip.txt
## setting
RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL) ALL" > /etc/sudoers
RUN echo "root ALL=(ALL) ALL" > /etc/sudoers

# GPU support
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# setup entrypoint
COPY ./ros_bashrc /

ENTRYPOINT ["/ros_bashrc"]

USER ${USER}
WORKDIR ${HOME}
RUN ls
