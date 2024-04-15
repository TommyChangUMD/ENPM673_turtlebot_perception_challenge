
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

## How to bring up the camera image?
``` shell
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```


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

