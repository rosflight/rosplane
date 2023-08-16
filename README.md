# ROSplane2

[![ROS2 CI](https://github.com/rosflight/rosplane2/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/rosflight/rosplane2/actions/workflows/ros2-ci.yml)

ROSplane2 is a basic fixed-wing autopilot build around ROS2 for use with the ROSflight autopilot. It is a continuation of 
the original [ROSplane](https://github.com/byu-magicc/rosplane) project. It is built according to the methods published 
in Small Unmanned Aircraft: Theory and Practice by Dr. Randy Beard and Dr. Tim McLain. 

## ROSplane2 Workspace Setup
To set up the workspace to run ROSPlane with ROSFlight, do the following:
1. Follow the instructions for setup up ROSFlight found here:
   https://github.com/byu-magicc/rosflight2.git
2. Clone the rosplane2 Gitlab repo into the same workspace folder as ROSFlight:
    `git clone https://github.com/byu-magicc/rosplane2.git`
   1. This will have a file structure like this:
      ```
      workspace_dir
      ├── rosflight2
      └── rosplane2 
      ```

3. Enter the rosplane2 folder `cd rosplane2`
4. Build the repository with `colcon build` -- *rosflight2 must be sourced for this to work.* 
5. Source the `setup.bash` and add it to the `.bashrc` with `echo "source workspace_dir/rosplane2/install/setup.bash" >> ~/.bashrc`
6. Ensure that both the `rosflight2` and the `rosplane2` are sourced.

## Running ROSplane2 SIL

To run ROSplane2 in simulation do the following:
1. Launch Gazebo and firmware simulation with `ros2 launch rosflight_sim fixedwing.launch.py`

2. Next, run the rosflight_io node configured for SIL with `ros2 run rosflight rosflight_io --ros-args -p udp:=true`
3. Update the firmware parameters for fixedwing flight with `ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "filename: rosplane2/src/rosplane2/config/fixedwing_param.yml"` if this is the first time launching.
4. Write the new parameters to memory for convenience with `ros2 service call /param_write std_srvs/srv/Trigger`
   - Note: the firmware will time out and not allow takeoff after 100 seconds, so you may need to redo steps one and two.
5. Calibrate the IMU to allow the airplane to be armed with `ros2 service call /calibrate_imu std_srvs/srv/Trigger`
6. Then launch ROSplane2 with `ros2 launch rosplane2_sim sim.launch.py`, this will run with the default controller. To use the total energy controller add the argument `control_type:=total_energy`.
7. Connect a controller with `ros2 run rosflight_utils rc_joy.py --ros-args --remap /RC:=/fixedwing/RC` or if you do not want to use a controller, run `python3 rc_sim.py --ros-args --remap RC:=/fixedwing/RC` while inside `rosflight2/rosflight_utils/src`.
8. Finally, arm the aircraft with channel 4 and then disable throttle and attitude override with channel 5. If not using a controller use `ros2 service call /arm std_srvs/srv/Trigger` and then disable RC override with `ros2 service call /disable_override std_srvs/srv/Trigger`.
9. The aircraft should now be airborne!

## Running ROSplane2 on hardware.

To run ROSplane2 on hardware:
1. Launch ROSplane2 while rosflight_io is running on the flight controller using `ros2 launch rosplane2 rosplane2.launch.py`
2. Fly the aircraft as you would under regular RC control.
3. When ready for autonomous flight flip throttle/attitude override switch (default channel 5).
4. Fly autonomously!
   - Note: the default autonomous mission is to fly a triangular loop, see path_planner_example.cpp for details.