
# ORB-SLAM3-Enabled Visual Odometry with Boston Dynamics Spot Robot in Gazebo

![Spot Robot Simulation with ORB-SLAM3](./image/spot_gazebo.gif)

This guide provides instructions for setting up a simulation of the Boston Dynamics Spot robot using a ROS-based setup in Gazebo, enhanced with ORB-SLAM3 visual odometry.

## Part 1: Boston Dynamics Spot Robot Setup

### Prerequisites
- Ensure you have `sudo` privileges on your system.
- ROS must be installed on your system.

### Installation Steps

1. **Install Dependencies:**
   - Create a catkin workspace if you don't already have one:
     ```bash
     cd ~/catkin_ws/src
     git clone -- https://github.com/ebasatemesgen/ORB-SLAM3-Enabled-Visual-Odometry-.git
     ```
   - Install Python ROS dependencies:
     ```bash
     sudo apt install -y python-rosdep
     ```

2. **Install ROS Dependencies:**
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace:**
   ```bash
   catkin_make
   ```

4. **Clone the Gazebo Models and Worlds Collection Repository:**
   ```bash
   git clone https://github.com/leonhartyao/gazebo_models_worlds_collection.git
   ```

5. **Update Environment Variables:**
   - Update the `GAZEBO_MODEL_PATH` and `GAZEBO_RESOURCE_PATH` in your `~/.bashrc` file:
     ```bash
     echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:<path to gazebo_models_worlds_collection repo>/models" >> ~/.bashrc
     echo "export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:<path to gazebo_models_worlds_collection repo>/worlds" >> ~/.bashrc
     source ~/.bashrc
     ```
   - Replace `<path to gazebo_models_worlds_collection repo>` with the actual path.

### Usage

- Run the Spot robot simulation in Gazebo:
  ```bash
  roslaunch spot_gazebo spot.launch world:=<world_file_name>
  ```
- Replace `<world_file_name>` with your desired Gazebo world file name.

### Customization

- You can use any world file from the `gazebo_models_worlds_collection` repository or create custom Gazebo worlds.

## Part 2: ORB-SLAM3-ROS

ORB-SLAM3-ROS is a ROS implementation of ORB-SLAM3 V1.0, focusing on the ROS integration. This package uses `catkin_make` and is tested on Ubuntu 20.04.

### 1. Prerequisites

- **Eigen3:**
  ```bash
  sudo apt install libeigen3-dev
  ```

- **Pangolin:**
  ```bash
  cd ~
  git clone https://github.com/stevenlovegrove/Pangolin.git
  cd Pangolin
  mkdir build && cd build
  cmake ..
  make
  sudo make install
  ```

- **OpenCV:**
  - Check your OpenCV version (minimum 3.0 required):
    ```bash
    python3 -c "import cv2; print(cv2.__version__)"
    ```
  - Follow the OpenCV installation instructions if a newer version is needed.

- **(Optional) Hector-Trajectory-Server:**
  - Install to visualize real-time camera/imu trajectories:
    ```bash
    sudo apt install ros-[DISTRO]-hector-trajectory-server
    ```

### 2. Installation

- Build the ORB-SLAM3-ROS package:
  ```bash
  cd ~/catkin_ws
  catkin_make
  ```

## Part 3: Automating Node Launch with a Bash Script

The provided Bash script automates the process of launching various ROS nodes and simulations for the Spot robot setup. This script simplifies the process of starting the simulation and necessary nodes, ensuring a smooth and integrated experience.

### Using the Bash Script

1. **Location of the Script**: Ensure the script `play.sh` (or the name you've given it) is located in a suitable directory and has execute permissions. If not, use `chmod +x play.sh` to make it executable.

2. **Running the Script**: Execute the script by navigating to its directory and running:
   ```bash
   ./play.sh
    ```
This will sequentially start the necessary ROS nodes and open new terminal windows for each component.


## Recording and Visualizing Odometry Data

### Recording Odometry Data

Use the following command to record odometry data from the `/odom` topic into a ROS bag:

```bash
rosbag record -O my_odom_data.bag /odom
```

Visualizing Odometry Data
The odometry_visualizer.py script visualizes the robot's odometry data. It plots the trajectory based on the X and Y positions recorded in the ROS bag file.

Run the script after recording the data:

```bash
python3 odometry_visualizer.py
```


## Using Real Robot Odometry Data

If you have odometry data from a real Boston Dynamics Spot robot, you can visualize this data using a similar approach.

### Running the ROS Bag with Real Robot Data

To visualize the real robot's odometry data, use a pre-recorded ROS bag file named `real_spot_data.bag`:

```bash
rosbag play real_spot_data.bag
```

This command plays back the recorded data, which can be visualized or processed further as needed.

### Testing on the Actual Robot

To run tests on the actual Boston Dynamics Spot robot, use the following launch command:

```bash
roslaunch spot_config simulated_spot.launch
```

This launch file is configured to interface with the real Spot robot, allowing you to see the tests or experiments I did in a real-world environment.



## Credits and References

- ORB-SLAM3 Original Repository: [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- CHAMP Quadruped Robot: [chvmp/champ](https://github.com/chvmp/champ)
- CHAMP Teleoperation Package: [chvmp/champ_teleop](https://github.com/chvmp/champ_teleop)
- Spot Robot ROS Integration: [chvmp/spot_ros](https://github.com/chvmp/spot_ros) (branch: gazebo)
- Additional Robot Models: [chvmp/robots](https://github.com/chvmp/robots)
```

