# Line Follower Robot
### Description
This repository contains code to run a small line follower robot from a Raspberry Pi 4 Model B on Ubuntu 22.04. The ROS 2 version used is Humble. For controlling the DC motors and reading data from the IR sensors, the Raspberry Pi GPIO library is used.

### Installation
```bash
# Install GPIO
pip install RPi.GPIO

# Setup and build workspace
mkdir -p line_follower/src
cd line_follower/src
git clone https://github.com/grayson-arendt/Line-Follower.git
cd ..
colcon build
source install/setup.bash
```

### Running Robot
```bash
ros2 launch robot_pkg robot_launch.py
```

Once the launch file is called, the robot will now follow a black line on a white surface.



