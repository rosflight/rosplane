#!/bin/bash

# Create a new tmux session
tmux new-session -d -s rosplane_sim_session

tmux source-file ~/.tmux.conf

# Split the tmux window into 4 panes
tmux split-window -t rosplane_sim_session:0.0 -h
tmux split-window -t rosplane_sim_session:0.0 -v
tmux split-window -t rosplane_sim_session:0.2 -v

# Arrange panes in each corner
tmux select-pane -t rosplane_sim_session:0.0
tmux select-pane -t rosplane_sim_session:0.1
tmux select-pane -t rosplane_sim_session:0.2
tmux select-pane -t rosplane_sim_session:0.3

# tmux send-keys -t rosplane_sim_session:0.0 'echo "Window 1 - Top Left"' C-m
# tmux send-keys -t rosplane_sim_session:0.1 'echo "Window 2 - Bottom Left"' C-m
# tmux send-keys -t rosplane_sim_session:0.2 'echo "Window 3 - Top Right"' C-m
# tmux send-keys -t rosplane_sim_session:0.3 'echo "Window 4 - Bottom Right"' C-m

tmux send-keys -t rosplane_sim_session:0.0 'cd ~/repos/rosflight_ws/' C-m
tmux send-keys -t rosplane_sim_session:0.0 'ros2 launch rosflight_sim fixedwing_sim_io_joy.launch.py aircraft:=anaconda' C-m
tmux send-keys -t rosplane_sim_session:0.2 'ros2 launch rosplane_sim sim.launch.py aircraft:=anaconda' C-m
tmux send-keys -t rosplane_sim_session:0.3 'ros2 run rosflight_sim rc_sim.py --ros-args --remap RC:=/fixedwing/RC' C-m
sleep 5
tmux send-keys -t rosplane_sim_session:0.1 'ros2 service call /calibrate_imu std_srvs/srv/Trigger' C-m

# Attach to the tmux session
tmux attach-session -t rosplane_sim_session

