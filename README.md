# ROS2 Install & ROSplane2 Setup

Prerequisites:

- Need to be using Ubuntu 22! Otherwise you need to build everything from source for all the things we’re installing - if you choose that, you are taking on your own risk.
- Make sure the computer you’re working off of has git, and that your github account has an SSH key for that computer (may or may nt be necessary to clone, pull from, and push to the github Tevin set up)

1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
    1. Follow directions from the top, use the ros-humble-desktop option
        1. `sudo apt update && sudo apt install locales`
        2. `sudo locale-gen en_US en_US.UTF-8`
        3. `sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8`
        4. `export LANG=en_US.UTF-8`
        5. `sudo apt install software-properties-common`
        6. `sudo add-apt-repository universe`
        7. `sudo apt update && sudo apt install curl -y`
        8. all one command: `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`
        9. `sudo apt update`
        10. `sudo apt upgrade`
        11. `sudo apt install ros-humble-desktop`
    2. Run “`echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`” to add ROS sources to your bashrc (can do this with nano, vi, etc. if preferred)
2. Install some required ROS packages:
    1. colcon: `sudo apt install python3-colcon-common-extensions`
    2. rqt: `sudo apt install ~nros-humble-rqt*`
    3. rosdep: 
        1. `sudo apt install python3-bloom python3-**rosdep** fakeroot debhelper dh-python`
        2. `sudo **rosdep** init`
        3. **`rosdep** update`
3. Install Gazebo: 
    1. `sudo apt install ros-humble-ros-gz`
    2. Add lines to the .bashrc by running the following:
        1. `echo "export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:$HOME/ROSPlane2/install/rosplane2_sim/share/rosplane2_sim" >> ~/.bashrc`
            1. Make sure to edit that filepath if you cloned ROSPlane2 to a different location!
        2. optional: `echo "echo "Exported ROSPlane2 Gazebo file path"" >> ~/.bashrc`
4. Clone the Github repo: In Home directory, do the following
    1. `git clone https://github.com/MEFlightDynamicsBYU/ROSPlane2.git`
    2. `cd ROSPlane2`
    3. Add some lines to the .bashrc by running the following in the command line: 
        1. `echo "source ~/ROSPlane2/install/setup.bash" >> ~/.bashrc`
            1. Make sure to edit that filepath if you cloned ROSPlane2 to a different location!
        2. optional: `echo "echo "Sourced Rosplane2"" >> ~/.bashrc`
5. Open a new terminal to initialize the source commands and build the package: `colcon build`
6. Launch the simulation: `ros2 launch rosplane2_sim rosplane2_launch.xml`



# ROSPlane2

First navigate to location of desired workspace should be

clone the repository

git clone git@github.com:MEFlightDynamicsBYU/ROSPlane2.git


ROSPLANE environment variable need to be set up. 
Navigate to your bashrc file

nano ~/.bashrc


Add these lines to the bottom of the .bashrc folder

source ~/ROSPlane2/install/setup.bash

echo "Sourced Rosplane2"

export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:"REPLACE ME WITH PATH LOCATION DIRECTORY CONTAINING ROSPLANE2"/ROSPlane2/install/rosplane2_sim/share/rosplane2_sim

echo "Exported file path"




Navigate to your ROSPlane workspace

cd ROSPlane2

colcon build

