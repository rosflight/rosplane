# ROSplane2 Workspace Setup
To set up the workspace to run ROSPlane with ROSFlight, do the following:
1. Follow the instructions for setup up ROSFlight found here:
   https://github.com/byu-magicc/rosflight2.git
2. Clone the ROSPlane2 Gitlab repo into the same workspace folder as ROSFlight:
    `git clone https://github.com/byu-magicc/rosflight2.git`
   1. This will have a file structure like this:
        ```bash
      workspace_dir
      ├── rosflight2
      └── ROSPlane2

   ```
3. Enter the ROSPlane2 folder `cd ROSPlane2`
4. Build the repository with `colcon build` -- *rosflight2 must be sourced for this to work.* 
5. Source the `setup.bash` and add it to the `.bashrc` with `echo "source workspace_dir/ROSPlane2/install/setup.bash" >> ~/.bashrc`
6. Ensure that both the `rosflight2` and the `ROSPlane2` are sourced.

# Running ROSPlane SIL

To run ROSPlane in simulation do the following:
1. Launch Gazebo and firmware simulation with `ros2 launch rosflight_sim fixedwing.launch.py`

2. Next, run the rosflight_io node configured for SIL with `ros2 run rosflight rosflight_io --ros-args -p udp:=true`
3. Update the firmware parameters for fixedwing flight with `ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "filename: ROSPlane2/src/rosplane2_sim/config/fixedwing_param.yml"` if this is the first time launching.
4. Write the new parameters to memory for convenience with `ros2 service call /param_write std_srvs/srv/Trigger`
    1. Note: the firmware will time out and not allow takeoff after 100 seconds, so you may need to redo steps one and two.
    2. Occasionally the `RC_ATT_OVERRIDE_CHN` of `RC_THR_OVERRIDE_CHN` are not saved/updated correctly. Ensure these are their correct values by running `ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: RC_THR_OVRD_CHN, value: 5}"` and `ros2 service call /param_set rosflight_msgs/srv/ParamSet "{name: RC_ATT_OVRD_CHN, value: 5}"`
    3. You can then repeat step 4. 
5. Calibrate the IMU to allow the airplane to be armed with `ros2 service call /calibrate_imu std_srvs/srv/Trigger`
6. Then launch ROSPlane with `ros2 launch rosplane2_sim fixedwing.launch.py`
7. Connect a controller with `ros2 run rosflight_utils rc_joy.py --ros-args --remap /RC:=/fixedwing/RC` or if you do not want to use a controller, run `python3 rc_sim.py --ros-args --remap RC:=/fixedwing/RC` while inside `rosflight2/rosflight_utils/src`.
8. Finally, arm the aircraft with `ros2 service call /arm std_srvs/srv/Trigger` and then disable RC override with `ros2 service call /disable_override std_srvs/srv/Trigger`.
9. The aircraft should now be airborne!