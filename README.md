# Ros_STIM300
ROS package for STIM300 in C++ 

## Prerequisites
Install eigen3

`sudo apt install libeigen3-dev`
 
or from source here.
http://eigen.tuxfamily.org/index.php?title=Main_Page

To use the STIM300 you have to make sure you have user permission to use it.
If the STIM300 is connected to ttyUSB0 do:

`sudo chmod 666 /dev/ttyUSB0`

## Installation

    cd ~/catkin_ws/src
    git clone -b master https://github.com/vortexntnu/stim300
    cd ~/catkin_ws
    catkin_make  or catkin build drivers-imu_stim300
    
  
## Usage
Run the script using:

    #To run the publisher:
    rosrun drivers-imu_stim300 /dev/ttyUSB0
    
## Documentation

The stim300 driver is based on the github repository from https://github.com/rock-drivers/drivers-imu_stim300. 
This was originally a Rock based driver but is now implemented to work with ROS
    
