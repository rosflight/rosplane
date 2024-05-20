#!/bin/bash

# Function to print help instructions
print_help() {
    usage
    echo
    echo "Options:"
    echo "  -h      Show this help message and exit"
    echo "  -s      Run as simulation of ROSflight"
    echo "  -t      Run tuning version of ROSflight"
    echo "  -r      Run the script with simulated transmitter"
    echo "  -a      Aircraft parameter to pass to launch files, default is anaconda"
}

usage() {
    echo "Usage: $0 [-h] [-s] [-t] [-r] [-a] aircraft path/to/ROSflight/workspace"
}

sim=false
tuning=false
rc_sim=false
aircraft='anaconda'

# Parse options using getopts
while getopts ":hstra:" opt; do
    case $opt in
        h)
            
            print_help
            exit 0
            ;;
        s)
            sim=true
            ;;
        t)
            tuning=true
            ;;
        r)
            rc_sim=true
            if ! $sim; then
              echo "Cannot run simulated transitter, while not running a simulation."
              exit 1 
            fi
            ;;
        a)
            aircraft=$OPTARG
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            print_help
            exit 1
            ;;
    esac
done

shift $((OPTIND -1))

# Get the file path
filepath=$1

# Check if filepath is provided
if [ -z "$filepath" ]; then
    echo "Absolute/relative file path is required"
    print_help
fi

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
# TODO Will this be run on the base station? If so, it needs to connect via ssh. I think that this may be less intensive on the network but I could be wrong.

# Send commands to run the sim.
tmux send-keys -t rosplane_sim_session:0.0 "cd $filepath" C-m

if $sim; then
  tmux send-keys -t rosplane_sim_session:0.0 "ros2 launch rosflight_sim fixedwing_sim_io_joy.launch.py aircraft:=$aircraft" C-m
else
  # TODO add functionality if it is not in sim.
  echo "Use -s argument."
  exit 1
fi

tmux send-keys -t rosplane_sim_session:0.2 "cd $filepath" C-m
if $tuning; then
  tmux send-keys -t rosplane_sim_session:0.2 "ros2 launch rosplane_sim sim_tuning.launch.py aircraft:=$aircraft" C-m
else
  tmux send-keys -t rosplane_sim_session:0.2 "ros2 launch rosplane_sim sim.launch.py aircraft:=$aircraft" C-m
fi


tmux send-keys -t rosplane_sim_session:0.3 "cd $filepath" C-m
# Check to see if rc_sim has been passed an arg
if $rc_sim; then
  tmux send-keys -t rosplane_sim_session:0.3 'ros2 run rosflight_sim rc_sim.py --ros-args --remap RC:=/fixedwing/RC' C-m
fi
tmux send-keys -t rosplane_sim_session:0.1 "cd $filepath" C-m
sleep 3 # It doesn't look like there is a clear way to do this better. This will have to do.
tmux send-keys -t rosplane_sim_session:0.1 'ros2 service call /calibrate_imu std_srvs/srv/Trigger' C-m

# Attach to the tmux session
tmux attach-session -t rosplane_sim_session

