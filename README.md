# DVS Image Reconstruction 
Cedric Scheerlinck

This has been developed with ROS version [Kinetic](http://wiki.ros.org/kinetic) (Ubuntu 16.04).

## Install Instructions

Please replace \<YOUR VERSION\> with your [ROS](http://wiki.ros.org/ROS/Installation) version (e.g. kinetic).

First install the [RPG Event Camera Driver](https://github.com/uzh-rpg/rpg_dvs_ros).

Install [catkin tools](http://catkin-tools.readthedocs.org/en/latest/installing.html) and [vcstool](https://github.com/dirk-thomas/vcstool)

    sudo apt install python-catkin-tools python-vcstool
    
Install ROS dependencies

    sudo apt install ros-<YOUR VERSION>-camera-info-manager ros-<YOUR VERSION>-image-view
    
Create a new catkin workspace if needed:

    mkdir -p ~/catkin_ws/
    cd ~/catkin_ws/
    catkin config --init --mkdirs --extend /opt/ros/<YOUR VERSION> --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

Clone this repository:

    cd ~/catkin_ws/src/
    git clone https://github.com/cedric-scheerlinck/dvs_image_reconstruction.git

Clone dependencies:

    vcs-import < dvs_image_reconstruction/dependencies.yaml
    
Build the packages:  

    catkin build davis_ros_driver complementary_filter pure_event_reconstruction
    source ~/catkin_ws/devel/setup.bash

## Downloads
Datasets can be found [here](https://drive.google.com/drive/folders/1Jv73p1-Hi56HXyal4SHQbzs2zywISOvc?usp=sharing).
