# realsense ROS package
## support all realsense camera types and Multiple cameras running simultaneously

### Dependance:
librealsense legacy. you should run its patch script and then make install.
https://github.com/IntelRealSense/librealsense/tree/legacy

### command line format:
roslaunch realsense_ros realsense_ros.launch sNum1:=xxx [sNum2:=xxx]

xxx should be the serial number of realsense camera you want to use
