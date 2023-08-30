# Setup

## Option 1

```bash
cd <Firmware_clone>
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch
```

Change line 16 in <Firmware_clone>/launch/posix_sitl.launch to 
```bash
<arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/iris_fpv_cam/iris_fpv_cam.sdf"/>
```

Change line 19 in detection.py to 
```python
self.image_sub = rospy.Subscriber("/iris/usb_cam/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
```

## Option 2

```bash
cd <Firmware_clone>
no_sim=1 make px4_sitl_default gazebo
```
in first terminal

```bash
cd <Firmware_clone>
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world
```
in second terminal

# Scripts

Run detection.py and then navigation.py
