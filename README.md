<p>
  <a href="https://github.com/rosflight/rosplane/actions/workflows/ros2-ci.yml">
    <img src="https://github.com/rosflight/rosplane/actions/workflows/ros2-ci.yml/badge.svg" alt="ROSplane CI" />
  </a>
</p>

<p align="center">
  <img src="rosplane_logo.png" alt="ROSplane logo" width=500 />
</p>

ROSplane is a basic fixed-wing autopilot build around ROS2 for use with the ROSflight autopilot.
It is a continuation of the original [ROSplane](https://github.com/byu-magicc/rosplane) project.
It is built according to the methods published in *Small Unmanned Aircraft: Theory and Practice* by Dr. Randy Beard and Dr. Tim McLain.

## Installation, Descriptions, Usage
For more information, see the [ROSflight documentation](https://docs.rosflight.org), which contains installation, general descriptions, and tutorials for how to use ROSplane. 

## Quick start (for ROS2 users)

For more detailed instructions, see the documentation link above.

1. Clone this repo into a ROS2 workspace
2. `colcon build`
3. Launch autopilot stack with
  ```bash
  ros2 launch rosplane rosplane.launch.py
  ```
  or 
  ```bash
  ros2 launch rosplane_sim sim.launch.py
  ```
  if in sim.

## Acknowledgements
A special thanks is due to the developers of ROSplane v1.0. Their work allowed for the project in its current form.
These include Gary Ellingson, who did much of the original development into C++ of these algorithms.
Thanks are also due to Judd Mehr and Brian Russel who adapted work for ROSplane.
