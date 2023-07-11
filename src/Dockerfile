FROM ros:kinetic-robot-xenial

# update packages and user programs
RUN apt-get update && apt-get install -y \
    wget \
    vim \
    git \
    qtbase5-dev \
    libqt5widgets5 \
    python-pyqt5 \
    python-opencv \
    python-matplotlib

# install Gazebo 7 and ros-kinetic packages
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get install -y \
    libgtk2.0-dev \
    gazebo7 libignition-math2-dev \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-image-geometry \
    ros-kinetic-tf2-geometry-msgs \
    ros-kinetic-controller-manager \
    ros-kinetic-effort-controllers \
    ros-kinetic-joint-state-controller \
    ros-kinetic-controller-interface \
    && rm -rf /var/lib/apt/lists/*

ENV QT_X11_NO_MITSHM 1
ENV ROS_DISTRO=kinetic

# Creates and configure the workspace
RUN mkdir -p /root/catkin_ws/src 
RUN git clone -b $ROS_DISTRO https://github.com/ros-perception/image_pipeline.git /root/catkin_ws/src/image_pipeline
COPY ./gazebo_simulation /root/catkin_ws/src/my_package

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /root/catkin_ws && catkin_make"

# Adds the gazebo models
ENV GAZEBO_MODEL_PATH=/root/catkin_ws/src/my_package/models:${GAZEBO_MODEL_PATH}

# bashrc configuration
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
