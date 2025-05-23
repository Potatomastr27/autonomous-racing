# autonomous-racing

## Dependancies
- Ubuntu 22.04
- ROS2 Humble
- 41012/pfms-support (included in the repo)

## Simulation Dependancies
- Gazebo for ros2 humble
- Rviz2

## Description
This is a package designed to autonomously drive an ackermann car around a race track made up of traffic cones. 

### System Features:
- The car utilizes a 180 degree lidar sensor with a 30m range on the front to automatically find the cones in the environment.
- The system then sorts the cones into pairs and calculates the midpoint of each pair.
- The pairs are used as the control points for a cubic spline which is used to smoothly interpolate between them.
- The spline is used to generate the next goal for the vehicle controller so it can calculate the steering angle similar to the pure pursuit algorithm
- A config file is provided so you can try changing some of the values in there to see how the system reacts
- Some unit tests are used to validate certain aspects of the system
- The control module will output where it thinks the cones are (orange cylinders) and where the midpoints of the track are (red cubes).

![rviz simulator moving](media/moving_sim.png)

## Installation
Install ros2 humble following this guide: https://docs.ros.org/en/humble/Installation.html 

Install gazebo:
```bash
sudo apt update
sudo apt install ros-humble-xacro ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-msgs ros-humble-controller-manager ros-humble-rqt-robot-steering ros-humble-robot-localization ros-humble-gazebo-ros2-control ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster ros-humble-diff-drive-controller ros-humble-imu-tools ros-humble-gazebo-ros-pkgs ros-humble-joint-state-publisher-gui ros-humble-joint-state-publisher 
sudo apt install python3-colcon-common-extensions
sudo apt install ros-dev-tools
```

Clone repo and download & install dependancies:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Potatomastr27/autonomous-racing.git
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ./src/autonomous-racing/pipes/pipes_3.0.5-humble_amd64.deb
sudo ldconfig
```

Build packages:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## First Launch
Run:
```bash
ros2 launch autonomous-racing start.launch.py gui:=true
```
This will open up rviz and launch gazebo.

While this is happening you will see a car with no textures on rviz (this is fine).

Gazebo will start downloading models in the background, you will know it is done when the gazebo gui opens up and you can see the race track.

Once this happens you can relaunch the program and you should see something like this on rviz:

![rviz simulator](media/idle_sim.png)

## How to use
You can run the control module + simulator using:
```bash
ros2 launch autonomous-racing start.launch.py
```
You can add an option to open a gazebo gui as well so that you can edit the simulation environment
```bash
ros2 launch autonomous-racing start.launch.py gui:=true
```
You can run the control module individually with:
```bash
ros2 launch autonomous-racing main.launch.py
```

Once you want it to start driving run this command in a different terminal:
```bash
ros2 service call /orange/mission std_srvs/srv/SetBool "{data: true}"
```

You should see the car start moving like so:

![rviz simulator moving](media/moving_sim.png)