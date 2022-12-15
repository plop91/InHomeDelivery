# IMU
Ian Sodersjerna  
11/2/2022

## Usage
1. build and run node
From the workspace root(top level dir for most projects) run the following commands.
```shell
# First source your work space, this can be different per environment so utilize the proper call for your environment.

# Then run the following to build the module
colcon build --packages-select imu

# On the robot
ros2 run imu imu 
```