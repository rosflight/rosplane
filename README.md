# ROSplane

[![ROS2 CI](https://github.com/rosflight/rosplane/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/rosflight/rosplane/actions/workflows/ros2-ci.yml)

ROSplane is a basic fixed-wing autopilot build around ROS2 for use with the ROSflight autopilot. It is a continuation of 
the original [ROSplane](https://github.com/byu-magicc/rosplane) project. It is built according to the methods published 
in Small Unmanned Aircraft: Theory and Practice by Dr. Randy Beard and Dr. Tim McLain. 

## ROSplane Workspace Setup
To set up the workspace to run ROSPlane with ROSFlight, do the following:
1. Follow the instructions for setup up ROSFlight found here:
   https://github.com/byu-magicc/rosflight2.git
2. Clone the rosplane Gitlab repo into the same workspace folder as ROSFlight:
    `git clone https://github.com/rosplane/rosplane.git`
   1. This will have a file structure like this:
      ```
      workspace_dir
      ├── rosflight2
      └── rosplane 
      ```

3. Enter the rosplane folder `cd rosplane`
4. Build the repository with `colcon build` -- *rosflight2 must be sourced for this to work.* 
5. Source the `setup.bash` and add it to the `.bashrc` with `echo "source workspace_dir/rosplane/install/setup.bash" >> ~/.bashrc`
6. Ensure that both the `rosflight2` and the `rosplane` are sourced.

## Running ROSplane SIL

To run ROSplane in simulation do the following:
1. Launch Gazebo and firmware simulation with `ros2 launch rosflight_sim fixedwing.launch.py`

2. Next, run the rosflight_io node configured for SIL with `ros2 run rosflight_io rosflight_io --ros-args -p udp:=true`
3. Update the firmware parameters for fixedwing flight with `ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "filename: rosflight/rosflight_sim/params/fixedwing_firmware.yaml"` if this is the first time launching.
4. Write the new parameters to memory for convenience with `ros2 service call /param_write std_srvs/srv/Trigger`
   - Note: the firmware will time out and not allow takeoff after 100 seconds, so you may need to redo steps one and two.
5. Calibrate the IMU to allow the airplane to be armed with `ros2 service call /calibrate_imu std_srvs/srv/Trigger`
6. Then launch ROSplane with `ros2 launch rosplane_sim sim.launch.py`, this will run with the default controller. To use the total energy controller add the argument `control_type:=total_energy`.
7. Connect a controller with `ros2 run rosflight_utils rc_joy.py --ros-args --remap /RC:=/fixedwing/RC` or if you do not want to use a controller, run `python3 rc_sim.py --ros-args --remap RC:=/fixedwing/RC` while inside `rosflight2/rosflight_utils/src`.
8. Finally, arm the aircraft with channel 4 and then disable throttle and attitude override with channel 5. If not using a controller use `ros2 service call /toggle_arm std_srvs/srv/Trigger` and then disable RC override with `ros2 service call /toggle_overide std_srvs/srv/Trigger`.
9. The aircraft should now be airborne!

## Running ROSplane on hardware.

To run ROSplane on hardware:
1. Launch ROSplane while rosflight_io is running on the flight controller using `ros2 launch rosplane rosplane.launch.py`
2. Fly the aircraft as you would under regular RC control.
3. When ready for autonomous flight flip throttle/attitude override switch (default channel 5).
4. Fly autonomously!

## Flying missions

Autonomous waypoint missions can easily be flown using ROSplane.
The waypoints of a mission are controlled by the `path_planner` node.
These waypoints are sent to the `path_manager` node.
Low level path-following is done by the `path_follower` node.
See "Small Unmanned Aircraft: Theory and Practice" by Dr. Randy Beard and Dr. Tim McLain for more information on the architecture.

### Adding waypoints

ROSplane initializes with no waypoints added to the `path_planner`.
We recommend using a mission .yaml file (an example mission can be found in `rosplane/params/fixedwing_mission.yaml`).
Loading the mission can be done using 

```ros2 service call /load_mission_from_file rosflight_msgs/srv/ParamFile "{filename: FILENAME}"```

where `FILENAME` is the absolute path to the mission .yaml file.
Note that the origin (0,0,0) is placed at the GNSS location where ROSplane was initialized.

> **Important**: All waypoints must include a valid `[X, Y, Z]`, `va_d`, and `lla` values.

Alternatively, you can add a waypoint one at a time by calling the appropriate service

```ros2 service call /add_waypoint rosplane_msgs/srv/AddWaypoint "{w: [X, Y, Z], chi_d: CHI_D, lla: USE_LLA, use_chi: USE_CHI, va_d: VA_D}"```

where `[X, Y, Z]` is the NED position of the waypoint from the origin (in meters) OR the GNSS location of the waypoint (LLA), `CHI_D` is the desired heading at the waypoint, and `VA_D` is the airspeed at the waypoint.
Set the `lla` field to `true` if the waypoint `[X, Y, Z]` field is given in GNSS coordinates and `false` if given in NED coordinates.
Corners in the path are controlled by `USE_CHI`, where a value of `True` will cause ROSplane to use a Dubins path planner and a value of `False` will cause a fillet path planner to be used.
Adding waypoints can be done at any time, even after loading from a file.

Clearing waypoints can be done using `ros2 service call /clear_waypoints std_msgs/srv/Trigger`.

### Publishing Waypoints

The `path_planner` node automatically publishes a small number of waypoints (default is 3) at the beginning of this mission.
This number is controlled by the `num_waypoints_to_publish_at_start` ROS2 parameter. 

Additional waypoints can be published using `ros2 service call /publish_next_waypoint std_srvs/srv/Trigger`.