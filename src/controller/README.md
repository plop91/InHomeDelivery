# Controller
Ian Sodersjerna  
11/2/2022

## Usage
1. build and run node
From the workspace root(top level dir for most projects) run the following commands.
```shell
# First source your work space, this can be different per environment so utilize the proper call for your environment.

# Then run the following to build the module and its requirements
colcon build --packages-select controller

# On the computer with the controller plugged in.
ros2 run controller controller # NOTE: to properly end the program the controller must be unplugged.
```