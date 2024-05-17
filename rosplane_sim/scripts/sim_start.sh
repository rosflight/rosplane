#!/bin/bash

# Function to print help instructions
print_help() {
    echo "Usage: $0 [options]"
    echo
    echo "Options:"
    echo "  -h, --help      Show this help message and exit"
    echo "  -r, --rc_sim    Run the script for no physical transmitter"
}

# Check if -h or --help is passed as the first argument
if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    print_help
    exit 0
fi

rc_sim=$1

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

# Window placement reference:
# rosplane_sim_session:0.0 Top Left
# rosplane_sim_session:0.1 Bottom Left
# rosplane_sim_session:0.2 Top Right
# rosplane_sim_session:0.3 Bottom Right

# TODO add argument to know whether to launch a docker containter using compose. Smart way to extract name of container? Try to reduce number of args.
# TODO Will this be run on the base station? If so, it needs to connect via ssh.

# Send commands to run the sim.
# TODO change to take path argument, and have all windows go to the directory.
tmux send-keys -t rosplane_sim_session:0.0 'cd ~/repos/rosflight_ws/' C-m
# TODO add argument to check for sim or not. (If done rename the whole script)
tmux send-keys -t rosplane_sim_session:0.0 'ros2 launch rosflight_sim fixedwing_sim_io_joy.launch.py aircraft:=anaconda' C-m
# TODO add argument if tuning or not.
tmux send-keys -t rosplane_sim_session:0.2 'ros2 launch rosplane_sim sim.launch.py aircraft:=anaconda' C-m

# Add check to see if rc_sim has been passed an arg
if [ "$rc_sim" == "--rc_sim" ] || [ "$rc_sim" == "-r" ]; then
  tmux send-keys -t rosplane_sim_session:0.3 'ros2 run rosflight_sim rc_sim.py --ros-args --remap RC:=/fixedwing/RC' C-m
fi
sleep 5 # Add a better way to check if calibration can be done. Really find an async way to do this
tmux send-keys -t rosplane_sim_session:0.1 'ros2 service call /calibrate_imu std_srvs/srv/Trigger' C-m

# Attach to the tmux session
tmux attach-session -t rosplane_sim_session

