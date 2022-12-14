# InHomeDelivery
In home delivery robot for Mobile Robotics II at UML

## Usage
Usage assumes you have ROS humble installed, these instructions should be followed both on the raspberry pi and controlling computer.  
If you have not read and followed the instructions from 'src\camera\README.md' please do so before continuing.
```shell
# source your ros setup script
source /opt/ros/humble/setup.bash # in linux
# --or--
call c:\dev\ros2_humble\local_setup.bat # in windows

# build packages
colcon build # build all packages
# --or-- 
colcon build --packages-select motors imu camera delivery_robot_interfaces # build robot packages
# --or--
colcon build --packages-select controller localization delivery_robot_interfaces # build controller packages

# source install scripts
source install/setup.bash # in linux
# --or--
call install\setup.bat # in windows

# call appropriate launch file
ros2 launch launch/robot_launch.py # on robot
# --or-- 
ros2 launch launch/controller_launch.py # on computer

# To monitor the nodes (NOTE: ensure both ros and local setup are called in these windows)
rviz2 # to monitor topics visually
# --or--
rqt # to send service requests
```

## Packages
* camera - This package controls the camera, capable of capturing images and detecting aruco markers in those images.
* controller - This package acts as a controller, currently it has compatibility for xbox controllers but can be expanded.
* delivery_robot_interfaces - This package contains srv and msg files required by the System
* imu - This package interacts with the IMU(WT901) and provides basic odometry messages
* localization - This package takes in marker information from the camera and odometry information from the IMU and Motors and localizes the robot on a pre-determined map.
* motors - This package interacts with the motors, this package is compatible with the L298N motor driver and any motor using a 2 pin encoder and provides basic odometry messages.

## Node layout
Running on Raspberry PI
* Camera node
* Motor driver node

Running on PC
* Localization node
* Controller node
* Rviz
* Rqt

