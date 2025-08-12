ARG ARCH=
#2
ARG CORES=6
FROM ${ARCH}osrf/ros:noetic-desktop-full

LABEL maintainer="Duarte Cruz <duarte.cruz@isr.uc.pt>"

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

SHELL ["/bin/bash","-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update \
    && apt-get install -y \
    build-essential \
    apt-utils \
    curl \
    git \
    wget \
    vim \
    nano \
    libtbb-dev \
    libgoogle-glog-dev \
    packagekit-gtk3-module \
    libyaml-cpp-dev \
    libpcap-dev \
    libpthread-stubs0-dev \
    libeigen3-dev \
    liblcm-dev \
    psmisc \
    zsh \
    fonts-powerline \
    autojump

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install some python packages
RUN apt-get -y install \
    python3 \
    python3-pip \
    python3-serial \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosdep \
    python3-catkin-tools \
    python3-rosunit

RUN pip3 install "numpy>=1.20" pyquaternion

#Install ROS Packages
RUN apt-get install -y ros-${ROS_DISTRO}-pcl-ros \ 
    ros-${ROS_DISTRO}-geometry \
    ros-${ROS_DISTRO}-grid-map-core \
    ros-${ROS_DISTRO}-grid-map-ros \
    ros-${ROS_DISTRO}-grid-map-filters \
    ros-${ROS_DISTRO}-grid-map-rviz-plugin \
    ros-${ROS_DISTRO}-grid-map-visualization \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-imu-transformer \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-mapviz \
    ros-${ROS_DISTRO}-mapviz-plugins \
    ros-${ROS_DISTRO}-tile-map \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-roslib \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-message-generation \
    ros-${ROS_DISTRO}-message-runtime \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-nodelet \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-ddynamic-reconfigure \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-librealsense2 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-octomap-ros \
    ros-${ROS_DISTRO}-octomap-msgs \
    ros-${ROS_DISTRO}-serial \
    ros-${ROS_DISTRO}-rviz-imu-plugin \
    ros-${ROS_DISTRO}-imu-complementary-filter \
    yad

#Configure catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Clean-up
WORKDIR /
RUN apt-get clean

CMD ["bash"]
