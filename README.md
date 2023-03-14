# ROSPlane2
New class repository for git 

First navigate to location of desired workspace should be

clone the repository

git clone git@github.com:MEFlightDynamicsBYU/ROSPlane2.git


ROSPLANE environment variable need to be set up. 
Navigate to your bashrc file

nano ~/.bashrc


Add these lines to the bottom of the .bashrc folder

source ~/ROSPlane2/install/setup.bash

echo "Sourced Rosplane2"

export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:"REPLACE ME WITH PATH LOCATION"/ROSPlane2/install/rosplane2_sim/share/rosplane2_sim

echo "Exported file path"




Navigate to your ROSPlane workspace

cd ROSPlane2

colcon build
