#!/bin/bash

################################################################################

# Link the default shell 'sh' to Bash.
alias sh='/bin/bash'

################################################################################

# Configure the terminal.

# Disable flow control. If enabled, inputting 'ctrl+s' locks the terminal until inputting 'ctrl+q'.
stty -ixon

################################################################################

# Configure 'umask' for giving read/write/execute permission to group members.
umask 0002

################################################################################

# Source the ROS environment.
echo "Sourcing the ROS environment from '/opt/ros/melodic/setup.bash'."
source /opt/ros/melodic/setup.bash

# Source the Catkin workspace.
echo "Sourcing the Catkin workspace from '/root/roomba_hack/catkin_ws/devel/setup.bash'."
source /root/external_catkin_ws/devel/setup.bash
source /root/roomba_hack/catkin_ws/devel/setup.bash

################################################################################

# Add the Catkin workspace to the 'ROS_PACKAGE_PATH'.
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/root/roomba_hack/catkin_ws/src/:/root/external_catkin_ws/src/
export ROS_WORKSPACE=/root/roomba_hack/catkin_ws

################################################################################

# Set roomba_hack/ROS network interface.
export ROS_IP=`hostname -I | cut -d' ' -f1`
echo "ROS_IP is set to '${ROS_IP}'."

export ROS_HOME=~/.ros

alias sim_mode='export ROS_MASTER_URI=http://localhost:11311; export PS1="\[[44;1;37m\]<local>\[[0m\]\w$ "'
alias roomba_mode='export ROS_MASTER_URI=http://TMP_IP:11311; export PS1="\[[41;1;37m\]<roomba>\[[0m\]\w$ "'

################################################################################

# Move to the working directory.
cd /root/roomba_hack/
