
# ENPM673 Turtlebot Perception Challenge

## How to build / install the `enpm673_final_proj` ROS2 package?

``` shell
source /opt/ros/humble/setup.bash 
colcon build --symlink-install --packages-select enpm673_final_proj
source install/setup.bash
```

## How to run the program?
### Python version:

``` shell
ros2 run enpm673_final_proj enpm673_final_proj_main.py 
```
The source code files are located at:
  - *`enpm673_final_proj/enpm673_module/enpm673_final_proj.py`*
  - *`enpm673_final_proj/scripts/enpm673_final_proj_main.py`*

### C++ version:

``` shell
ros2 run enpm673_final_proj cpp_enpm673_final_proj 
```
The source code files are located at: 
  - *`enpm673_final_proj/src/enpm673_final_proj.cpp`*

## How start the Gazebo simulation?
``` shell
ros2 launch enpm673_final_proj enpm673_world.launch.py "verbose:=true"
```
The Gazebo world file is located at:
  - `enpm673_final_proj/worlds/enpm673.world`

[<img src=screenshots/gazebo.png
    width="80%" 
    style="display: block; margin: 0 auto"
    />](screenshots/gazebo.png)


## How to bring up the camera image?
``` shell
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

[<img src=screenshots/rqt_image_view.png
    width="40%" 
    style="display: block; margin: 0 auto"
    />](screenshots/rqt_image_view.png)


## How to manually drive the Turtlebot?
``` shell
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
