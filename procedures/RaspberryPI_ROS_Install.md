# Install ROS on a Raspberry PI running Ubuntu 22.04LTS(64-bit)
Ian Sodersjerna
## Procedure

1. ROS installation  
    ```shell
    # run package update
    sudo apt update
    sudo apt upgrade
   
    # make sure locale is correct
    locale
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale
    
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt upgrade
    
    sudo apt install ros-humble-desktop
    # or
    sudo apt install ros-humble-ros-base
    
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=91" >> ~/.bashrc
    ```
2. test installation
    ```shell
     ros2 run demo_nodes_cpp talker
     #and
     ros2 run demo_nodes_py listener
    ```
3. install rqt
    ```shell
    sudo apt install ~nros-humble-rqt*
    ```

4. PIP install
    ```shell
    sudo apt install python3-pip python3-dev
    pip3 install -U pip setuptools argcomplete
    ```
5. install colcon
    ```shell
    sudo apt install python3-colcon-common-extensions
    ```
6. install opencv2
    ```shell
    sudo apt install libopencv-dev python3-opencv
    ```
7. install rosdep
   ```shell
   sudo apt install python3-rosdep2
   rosdep update
   ```
8. after importing library
```shell
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
```