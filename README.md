# CHARM

This branch contains the linux/ROS port of the orinal charm code. Tested on Ubuntu 14.04 with ROS indigo.

Installing ROS:
http://wiki.ros.org/indigo/Installation

Setting up CHARM:

First set up a ROS workspace, git clone, then compile:

    mkdir ~/charm_ws
    cd ~/charm_ws
    mkdir src
    cd src
    git clone https://github.com/JoshSong/charm.git -b linux-ros
    cd ~/charm_ws
    catkin_make

Running:

    source devel/setup.bash
    rosrun charm charm_node
