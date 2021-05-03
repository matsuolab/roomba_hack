#!/bin/bash

################################################################################

# Install system dependencies required by specific ROS packages.
# http://wiki.ros.org/rosdep
rosdep update

# Download package lists from Ubuntu repositories.
apt-get update

# Source the updated ROS environment.
source /opt/ros/melodic/setup.bash
source /root/external_catkin_ws/devel/setup.bash

################################################################################

# Initialize and build the Catkin workspace.
cd /root/roomba_hack/catkin_ws
rosdep install -y -i --from-paths -r src
catkin build -DCMAKE_BUILD_TYPE=Release

# Source the Catkin workspace.
source /root/roomba_hack/catkin_ws/devel/setup.bash