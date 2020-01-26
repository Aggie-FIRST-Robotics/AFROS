#!/bin/bash
#echo "some data for the file" >> testFile.fil
#echo $CMAKE_PREFIX_PATH
source /opt/ros/melodic/setup.bash
source ~/AFROS/catkin_ws/devel/setup.bash
roslaunch -v  /opt/ros/melodic/launchfile.launch
#echo "some data for the file" >> fileName.file
echo $CMAKE_PREFIX_PATH