# DVS Image Reconstruction 
Cedric Scheerlinck

This has been developed with ROS version [Kinetic](http://wiki.ros.org/kinetic) (Ubuntu 16.04).

## Install Instructions

First install [ROS](http://wiki.ros.org/ROS/Installation) and the [event camera drivers](https://github.com/uzh-rpg/rpg_dvs_ros).

Install [vcstool](https://github.com/dirk-thomas/vcstool)

    sudo apt-get install python-vcstool
    
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

    catkin build pure_event_reconstruction complementary_filter
    source ~/catkin_ws/devel/setup.bash

## Downloads
Datasets can be found [here](https://drive.google.com/drive/folders/1Jv73p1-Hi56HXyal4SHQbzs2zywISOvc?usp=sharing).
