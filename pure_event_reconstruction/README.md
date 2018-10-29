
# Pure Event Reconstruction

This package allows image reconstruction from only events.

## Run Instructions

### Live with DAVIS camera:

    roslaunch pure_event_reconstruction davis_mono.launch
 
to launch DAVIS with APS frames enabled (not used for reconstruction).
    
### From pre-recorded rosbag:
Download a rosbag from the [DVS Image Reconstruction Dataset](https://drive.google.com/drive/folders/1Jv73p1-Hi56HXyal4SHQbzs2zywISOvc?usp=sharing) or the [RPG Event-Camera Dataset](http://rpg.ifi.uzh.ch/davis_data.html).

Open a terminal launch roscore:

    roscore
    
In another terminal:

    rosbag play -l <path-to-rosbag>
    
In another terminal launch the complementary filter:
    
    roslaunch pure_event_reconstruction from_rosbag.launch
    
    
