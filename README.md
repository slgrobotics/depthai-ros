### my fork with properly published 3D boundary boxes

OAK-D Lite camera setup:

https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md

Cloning and building:
```
mkdir -p ~/depthai_ws/src
cd ~/depthai_ws/src
git clone https://github.com/slgrobotics/depthai-ros.git

cd ~/depthai_ws
export MAKEFLAGS="-j 1"
colcon build --parallel-workers=1 --executor sequential
# takes 10..15 minutes
```
Running it:
```
cd ~/depthai_ws
source install/setup.bash
ros2 launch depthai_filters spatial_bb.launch.py
```
Run `rviz2` with the following parameter file to start with:
```
https://github.com/slgrobotics/depthai_rospi/tree/main/rviz
```

![Screenshot from 2025-05-13 21-39-40](https://github.com/user-attachments/assets/fd6c5c40-aacd-4704-82a8-28845a2c333a)

**Note:**
- OAK-D cameras require ~1 Amp current from 5 V power source and _absolutely_ need a "[power T-Tap](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md#important-power-consumption-and-usb-connection-requirements)".
- The network load in the above example is 700 Mbits/sec, your WiFi is unlikely to bear that. See [this guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/WiFi_Logger_Visualizer.md).
- You can add *MarkerArray* to visualize *spatial_bb* topic in RViz2 the way Luxonis intended it originally.

For more info see https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md#spatial-examples

-------------------

Original README follows:

# Depthai ROS Repository
Hi and welcome to the main depthai-ros respository! Here you can find ROS related code for OAK cameras from Luxonis. Don't have one? You can get them [here!](https://shop.luxonis.com/)

You can find the newest documentation [here](https://docs.luxonis.com/software/ros/depthai-ros/)
