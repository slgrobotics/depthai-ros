### my fork with properly published 3D boundary boxes

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
ros2 launch depthai_filters spatial_bb.launch.py
```
Run `rviz2` with the following parameter file to start with:
```
https://github.com/slgrobotics/depthai_rospi/tree/main/rviz
```

![Screenshot from 2025-05-13 21-39-40](https://github.com/user-attachments/assets/fd6c5c40-aacd-4704-82a8-28845a2c333a)

Original README follows:

# Depthai ROS Repository
Hi and welcome to the main depthai-ros respository! Here you can find ROS related code for OAK cameras from Luxonis. Don't have one? You can get them [here!](https://shop.luxonis.com/)

You can find the newest documentation [here](https://docs.luxonis.com/software/ros/depthai-ros/)
