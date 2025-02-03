ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO

LABEL Name="devcon_example_ros2_jazzy.v0.docker" \
      Maintainer="mgal@alps.cz" \
      Author="Marek Gal" \
      Date="2024.11.20" \
      Version="0.1.2"

ENV LANGUAGE=C.UTF-8
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Or your actual UID, GID on Linux if not the default 1000
ARG USERNAME=ubuntu
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID

# Choose RMW Implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp

ARG DEBIAN_FRONTEND=noninteractive

# Set environment variables
ARG VENV_DIR=/home/$USERNAME/.venv
ARG SITE_PACKAGES_DIR=$VENV_DIR/lib/python3.12/site-packages

# install essential packages.
RUN apt update && apt install -y --no-install-recommends \
    python3 \
    wget \
    usbutils \
    git \
    git-lfs \
    bash-completion \
    openssh-client \
    python3-pip \
    python3-tk \
    python3-venv
    # && python3 -m pip3 install --upgrade pip \
    # && python3 -m pip3 install -U numpy==1.23.5 \
    # && python3 -m pip3 install -U scikit-image
    # && python3 -m pip3 install -U pyserial

# Gstreamer layer
RUN apt update && apt-get install -y \
    libx264-dev \
    libjpeg-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav
    
# RPi implemetation: set lib path for gstreamer (fixing issue with "avdec_h264" not found)
# ENV LD_PRELOAD /usr/lib/aarch64-linux-gnu/libgomp.so.1

# install additional packages / mostly ROS related.
RUN apt update && apt install -y --no-install-recommends \
    python3-scipy \
    python3-opencv \
    python3-sklearn \
    python3-sklearn-lib \
    v4l-utils \
    libtheora-dev \
    libogg-dev \
    libboost-python-dev \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-camera-calibration \
    ros-$ROS_DISTRO-common-interfaces \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-image-common \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-sensor-msgs-py \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-geometry2 \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rosbag2 \
    ros-$ROS_DISTRO-topic-tools

# app specific packages
# RUN apt update && apt install -y --no-install-recommends \
#     can-utils

# Configure apt and install packages
# Create a non-root user to use if preferred - see https://aka.ms/vscode-remote/containers/non-root-user.
RUN usermod -aG video -aG dialout ubuntu\
    # [Optional] Add sudo support for non-root user
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # && apt-get -y install libgl1-mesa-glx libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Clean up APT stuff
RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
# Create a working directory
WORKDIR /home/ubuntu

# Copy only the requirements.txt file into the container
# COPY requirements.txt .


# Create the virtual environment
RUN python3 -m venv $VENV_DIR

# Activate the virtual environment and install dependencies
RUN /bin/bash -c "source $VENV_DIR/bin/activate && \
    pip install numpy==1.26.4 scikit-image pyserial ipykernel acconeer-exptool"

WORKDIR /workspaces
ENV DEBIAN_FRONTEND=dialog

# update rosdep
RUN rosdep update

# Set up auto-source of workspace for ros user
# ARG WORKSPACE
# RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/devcon/.bashrc
RUN echo "if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then source /opt/ros/$ROS_DISTRO/setup.bash; fi\n" >> /home/$USERNAME/.bashrc  && \
    # PYTHONPATH
    echo "if [[  \":\$PYTHONPATH:\" != *\":$SITE_PACKAGES_DIR:\"* ]]; then" >> /home/$USERNAME/.bashrc && \
    echo "    export PYTHONPATH=\"$SITE_PACKAGES_DIR:\$PYTHONPATH\"" >> /home/$USERNAME/.bashrc && \
    echo "fi\n" >> /home/$USERNAME/.bashrc
