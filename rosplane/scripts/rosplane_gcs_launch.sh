#!/bin/bash

# Function to print help instructions
print_help() {
    usage
    echo
    echo "Options:"
    echo "  -h                Show this help message and exit"
    echo "  -s                Run as simulation of ROSflight"
    echo "  -t                Run tuning version of ROSflight"
    echo "  -r                Run the script with simulated transmitter"
    echo "  -a                Aircraft option to pass to launch files"
    echo "  aircraft          The aircraft parameter to be used in launch files, default is anaconda"
    echo "  -b                Start a ROS bag"
    echo "  bag_name          Name of the ROS bag (optional)"
    echo "  -o                Run the tmux session online (on remote host)"
    echo "  user@host         Username and address of remote"
    echo "  -d                Indicate that you should run a docker container of the given name (uses compose, and compose file should be in workspce directory)"
    echo "  container       Name of container"
    echo "  path/to/workspace Path to workspace"
}

usage() {
    echo "Usage: $0 [-h] [-s] [-t] [-r] [-a aircraft] [-b [bag_name]] [-o user@host] [-d docker_container_name] path/to/ROSflight/workspace"
}

sim=false
tuning=false
rc_sim=false
aircraft='anaconda'
online=false
docker=false
bag=false

# Parse options using getopts
while getopts ":hstra:b:o:d:" opt; do
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
        b)
            bag=true
            bag_name=$OPTARG
            ;;
        o)
            online=true
            user_host=$OPTARG
            if $sim; then
              echo "Cannot run simulator on a remote."
              exit 1 
            fi
            ;;
        d)
            docker=true
            container=$OPTARG
            if ! $online; then
              echo "Running docker on the local machine is not supported currently."
            fi
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

# Uncomment this line if you want to use your own tmux config
# tmux source-file ~/.tmux.conf

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

# Setup all panes to ssh into the remote
if $online; then
  tmux send-keys -t rosplane_sim_session:0.0 "ssh $user_host" C-m
  tmux send-keys -t rosplane_sim_session:0.1 "ssh $user_host" C-m
  tmux send-keys -t rosplane_sim_session:0.2 "ssh $user_host" C-m
  tmux send-keys -t rosplane_sim_session:0.3 "ssh $user_host" C-m
fi 

# Send all of the panes to the working directory.
tmux send-keys -t rosplane_sim_session:0.0 "cd $filepath" C-m
tmux send-keys -t rosplane_sim_session:0.1 "cd $filepath" C-m
tmux send-keys -t rosplane_sim_session:0.2 "cd $filepath" C-m
tmux send-keys -t rosplane_sim_session:0.3 "cd $filepath" C-m

# The docker commands assume that the container uses -it commands to create a persistent terminal.

if $docker; then
  tmux send-keys -t rosplane_sim_session:0.0 "docker compose up -d" C-m
  sleep 2
  tmux send-keys -t rosplane_sim_session:0.0 "docker compose exec $container bash" C-m
  tmux send-keys -t rosplane_sim_session:0.1 "docker compose exec $container bash" C-m
  tmux send-keys -t rosplane_sim_session:0.2 "docker compose exec $container bash" C-m
  tmux send-keys -t rosplane_sim_session:0.3 "docker compose exec $container bash" C-m
fi

# Send commands to run the sim.

if $sim; then
  tmux send-keys -t rosplane_sim_session:0.0 "ros2 launch rosflight_sim fixedwing_sim_io_joy.launch.py aircraft:=$aircraft" C-m
else
  tmux send-keys -t rosplane_sim_session:0.0 "ros2 run rosflight_io rosflight_io --ros-args -p port:=/dev/ttyACM0" C-m
fi

if $tuning; then
  if $sim; then
    tmux send-keys -t rosplane_sim_session:0.2 "ros2 launch rosplane_sim sim_tuning.launch.py aircraft:=$aircraft" C-m
  else 
    tmux send-keys -t rosplane_sim_session:0.2 "ros2 launch rosplane_tuning rosplane_tuning.launch.py aircraft:=$aircraft" C-m
  fi 
elif $sim; then
  tmux send-keys -t rosplane_sim_session:0.2 "ros2 launch rosplane_sim sim.launch.py aircraft:=$aircraft" C-m
else
  tmux send-keys -t rosplane_sim_session:0.2 "ros2 launch rosplane rosplane.launch.py aircraft:=$aircraft" C-m
fi


# Check to see if rc_sim has been passed an arg
if $rc_sim; then
  tmux send-keys -t rosplane_sim_session:0.3 'ros2 run rosflight_sim rc.py --ros-args --remap RC:=/fixedwing/RC' C-m
fi

if $sim; then
  sleep 3 # It doesn't look like there is a clear way to do this better. This will have to do.
  tmux send-keys -t rosplane_sim_session:0.1 'ros2 service call /calibrate_imu std_srvs/srv/Trigger' C-m
fi

if $bag; then

  if $online; then
    tmux send-keys -t rosplane_sim_session:0.3 "cd bags" C-m
  fi

  if [ ! -z $bag_name ]; then
    tmux send-keys -t rosplane_sim_session:0.3 "ros2 bag record -a -o $bag_name -e \'|\' -x \'(/rviz|/tf)\'" C-m
  else
    tmux send-keys -t rosplane_sim_session:0.3 "ros2 bag record -a -e \'|\' -x \'(/rviz|/tf)\'" C-m
  fi
fi

# Create another window that is on the local machine.
if $online; then
  tmux new-window -t rosplane_sim_session -n local_env
fi


# Attach to the tmux session
tmux attach-session -t rosplane_sim_session

