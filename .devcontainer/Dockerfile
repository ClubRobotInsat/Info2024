FROM ros:humble-perception
ARG USERNAME=pi
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN echo $USERNAME:$USERNAME | chpasswd
RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y python3-pip openssh-server net-tools nano vim apt-utils ufw iputils-ping software-properties-common ros-humble-rqt* ros-humble-rviz2
RUN apt-get update && apt-get install -y ros-humble-turtlesim ros-humble-rosbridge-suite
RUN apt-get update && apt-get install -y ros-humble-teleop-twist-keyboard ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-dev ros-humble-xacro
RUN pip install setuptools==58.2.0 && pip install python-can # python-can is a package for CAN communication
# RUN sudo od /dev/ttyACM0 # This command is to check if the device is connected to the container
# RUN sudo adduser $USER dialout # Add the user to the dialout group to access the serial port

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# Source the ROS setup.bash file.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
# Enable SSH
RUN echo "sudo service ssh restart > /dev/null" >> /home/$USERNAME/.bashrc

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["service ssh restart"]