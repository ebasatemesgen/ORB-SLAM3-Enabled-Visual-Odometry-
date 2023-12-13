#!/bin/bash

# Function for countdown during sleep
sleep_countdown() {
    secs=$1
    for i in $(seq $secs -1 1); do 
        echo -ne "Sleeping for $i seconds\r"
        sleep 1
    done
    echo -ne '\n'
}

# Function to wait for a process to finish
wait_for_process() {
    pid=$1
    while kill -0 $pid 2> /dev/null; do
        sleep 1
    done
}

# Function to open a new terminal with a countdown and execute a command
countdown() {
    secs=$1
    cmd=$2
    gnome-terminal -- bash -c "echo '$cmd'; $cmd; sleep_countdown $secs; exec bash" &
    pid=$!
    echo $pid >> pids.txt
    wait_for_process $pid
}

# Function to ask a question with a yes/no answer
ask_yes_no() {
    while true; do
        read -p "$1 (y/n): " yn
        case $yn in
            [Yy]* ) return 0;;
            [Nn]* ) echo "You need to play the simulation to continue. Please try again."; continue;;
            * ) echo "Please answer yes or no."; continue;;
        esac
    done
}


# Function to ask a yes/no question without looping
ask_yes_no_for_teleop() {
    while true; do
        read -p "$1 (y/n): " yn
        case $yn in
            [Yy]* ) return 0;;  # Yes
            [Nn]* ) return 1;;  # No
            * ) echo "Please answer yes or no.";;
        esac
    done
}



# SIGINT (Ctrl+C) signal handler
trap 'echo "Closing all terminals..."; kill $(cat pids.txt); rm pids.txt; exit' SIGINT

# Make sure the PID file is empty
> pids.txt

# Launch ORB_SLAM3 in a new terminal with a countdown
countdown 4 "roslaunch orb_slam3_ros spot_mono_inertial.launch"

# Countdown instead of sleep
sleep_countdown 10

# Launch Gazebo simulation in a new terminal with a countdown
countdown 7 "roslaunch spot_config gazebo.launch"

# Keep asking if the simulation was played until user confirms
while true; do
    ask_yes_no "Did you play the Gazebo simulation? In order for the robot to move, you need to go and press the play button on Gazebo."
    if [ $? -eq 0 ]; then
        echo "Continuing with the setup..."
        break
    fi
done





# Ask if they want to teleoperate
ask_yes_no_for_teleop "Do you want to teleoperate?"
if [ $? -eq 0 ]; then
    # Launch teleoperation in a new terminal
    gnome-terminal -- roslaunch champ_teleop teleop.launch &
    echo $! >> pids.txt
else
    # Launch navigate.launch in a new terminal
    gnome-terminal -- roslaunch spot_config navigate.launch &
    echo $! >> pids.txt

    # Countdown instead of sleep
    sleep_countdown 15

    # Inform about autonomous movement and run the Python script
    echo "Running the autonomous movement script..."
    gnome-terminal -- bash -c "echo 'Running the Python script...'; python3 move_map.py; exec bash" &
    echo $! >> pids.txt
fi

# Wait for all opened terminals to close
for pid in $(cat pids.txt); do
    wait_for_process $pid
done
rm pids.txt


# Infinite loop to keep the script running
echo "Press Ctrl+C to exit."
while true; do
    sleep 1
done