# DVS Image Reconstruction 
Cedric Scheerlinck

## Install Instructions

Install [ROS](http://wiki.ros.org/ROS/Installation) and the [event camera drivers](https://github.com/uzh-rpg/rpg_dvs_ros).
This has been developed with ROS version [Kinetic](http://wiki.ros.org/kinetic) under Ubuntu 16.04.

Clone this repository:

    cd ~/catkin_ws/src/
    git clone https://github.com/cedric-scheerlinck/dvs_image_reconstruction.git

Build the packages:  

    catkin build pure_event_reconstruction complementary_filter
    source ~/catkin_ws/devel/setup.bash

## Downloads
Datasets can be found [here](https://drive.google.com/drive/folders/1Jv73p1-Hi56HXyal4SHQbzs2zywISOvc?usp=sharing).
