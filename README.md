# ROS2 Install & ROSplane2 Setup

1. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and source it.
    `source /opt/ros/humble/setup.bash`
2. Install some required ROS packages:
    1. colcon: `sudo apt install python3-colcon-common-extensions`
    2. rqt: `sudo apt install ~nros-humble-rqt*`
    3. rosdep: 
        1. `sudo apt install python3-bloom python3-**rosdep** fakeroot debhelper dh-python`
        2. `sudo **rosdep** init`
        3. **`rosdep** update`
3. Create a workspace folder.
4. Clone the ROSPlane2 Gitlab repo into the workspace folder:
    `git clone https://github.com/bsutherland333/rosflight2.git`
5. Clone the rosflight2 Gitlab repo into the workspace folder:
    `git clone https://github.com/bsutherland333/rosflight2.git`
6. Build and source the rosflight_msgs packaage in the rosflight2 repo.
    1. `cd rosflight2`
    2. `colcon build --packages-select rosflight_msgs`
    3. `source install/setup.bash`
    4. `cd ..`
7. Build and source the ROSPlane2 workspace: 
    1. `cd ROSPlane2`
    2. `colcon build`
    3. `source install/setup.bash`
8. Launch the simulation: `ros2 launch rosplane2_sim fixedwing.launch.py`
