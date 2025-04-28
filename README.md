
# ENPM673 Turtlebot Perception Challenge

## How to build / install the `enpm673_final_proj` ROS2 package?

``` shell
# first checkout the git repo
git clone https://github.com/TommyChangUMD/ENPM673_turtlebot_perception_challenge.git
cd ENPM673_turtlebot_perception_challenge/

# build and install the package
source /opt/ros/humble/setup.bash 
colcon build --symlink-install --packages-select enpm673_final_proj
```

## How to run the program?
### Python version:

``` shell
source install/setup.bash
ros2 run enpm673_final_proj enpm673_final_proj_main.py 
```
The source code files are located at:
  - [enpm673_final_proj/enpm673_module/enpm673_final_proj.py](enpm673_final_proj/enpm673_module/enpm673_final_proj.py)
  - [enpm673_final_proj/scripts/enpm673_final_proj_main.py](enpm673_final_proj/scripts/enpm673_final_proj_main.py)

### C++ version:

``` shell
source install/setup.bash
ros2 run enpm673_final_proj cpp_enpm673_final_proj 
```
The source code files are located at: 
  - [enpm673_final_proj/src/enpm673_final_proj.cpp](enpm673_final_proj/src/enpm673_final_proj.cpp)

## How start the Gazebo simulation?
``` shell
source install/setup.bash
source  /usr/share/gazebo/setup.bash      ## this step may not be needed
ros2 launch enpm673_final_proj enpm673_world.launch.py "verbose:=true"
```
The Gazebo world file is located at:
  - [enpm673_final_proj/worlds/enpm673.world](enpm673_final_proj/worlds/enpm673.world)

[<img src=screenshots/gazebo.png
    width="80%" 
    style="display: block; margin: 0 auto"
    />](screenshots/gazebo.png)

## How stop the Gazebo simulation?
Instead of closing the Gazebo window, just hit control-c from the console to send an "interrupt signal" (SIGINT) to the entire chain of processes.

## How to bring up the camera image?
One way is to use `rqt`'s image viewer to display the `/camera/image_raw` topic:

``` shell
source /opt/ros/humble/setup.bash 
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

[<img src=screenshots/rqt_image_view.png
    width="40%" 
    style="display: block; margin: 0 auto"
    />](screenshots/rqt_image_view.png)


## How to manually drive the Turtlebot?
We can use the `teleop_twist_keyboard` program to write angular and linear speeds to the `/cmd_vel` topic.
``` shell
source /opt/ros/humble/setup.bash 
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
- Press `x` and `c` to reduce linear and angular speeds.
- To move around, use the keys below:
```
Moving around:
   u    i    o
   j    k    l
   m    ,    .
```
[test_drive.webm](https://github.com/TommyChangUMD/ENPM673_turtlebot_perception_challenge/assets/114546517/a4fba8b3-2f71-4628-9336-015fc453e512)

# Troubleshoot
## Gazebo: Objects have no shadow

The Gazebo software that ships with Ubuntu 22.04 is version 11.10,
which is two years old. Unfortunately, this version has a few issues
with non-NVIDIA graphics cards. As a result, objects do not cast
shadows. Newer versions of Gazebo (version 11.14 or above) work much
better.

Upgrade instructions to Gazebo version 11.14 will be provided later.

## Start from scratch using a ROS2 Humble Docker container

If your Linux system (or Windows WSL2) is not running Ubunu 22.04, you can still try things out using a pre-built Docker container.
Follow the steps below to have a ROS2 Humble + Gazebo running in just a few mintues.
```shell
 sudo apt-get update
 sudo apt install python3-rocker
 rocker --privileged --x11 --user -- osrf/ros:humble-desktop 'bash -c "sudo apt update; sudo apt install -y terminator; terminator"'
```
This will open a new ROS2 Humble terminal.  Now, we just need to install `gazebo` and `turtlebot3` packages.  
Run on this new ROS2 Humble terminal:
```shell
sudo apt -y install ros-humble-gazebo* ros-humble-turtlebot3*
```
That's it.  We now have a working ROS2 Humble + Gazebo system. 

Note: You may want to add the `--home` option to the `rocker` command so you can access your host machine's home directory. Keep in mind that your ~/.bashrc file will be executed when you start the container.
