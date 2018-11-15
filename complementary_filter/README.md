# Complementary Filter


This package allows image reconstruction by combining events with APS intensity image frames.

## Run Instructions

### Live with DAVIS camera:

    roslaunch complementary_filter davis_mono.launch
    
### From pre-recorded rosbag:
Download a rosbag from the [DVS Image Reconstruction Dataset](https://drive.google.com/drive/folders/1Jv73p1-Hi56HXyal4SHQbzs2zywISOvc?usp=sharing) or the [RPG Event-Camera Dataset](http://rpg.ifi.uzh.ch/davis_data.html).

Open a terminal launch roscore:

    roscore
    
In another terminal:

    rosbag play -l <path-to-rosbag>
    
In another terminal launch the complementary filter:
    
    roslaunch complementary_filter from_rosbag.launch
    
## Reconfigure

![gui_picture](images/reconfigure.png)
