# seek_driver
Seek ROS Driver. This is a rudimentary ROS driver to run a Seek thermal camera leveraging the Seek SDK.
Tested with the Seek SDK 3.8.0.0 using a S304SP Mosaic Core Starter Kit and ROS melodic

This package provides two primary functions:
- It publishes the display Image, filtered Image, thermography image and telemetry data at a set interval
- It provides a service to restart the camera

# Dependencies
Seek SDK [Tested with 3.8.0.0]

ROS [Tested with Melodic]

OpenCV [Tested with 4.5.1]

# Use

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/howde-robotics/seek_driver.git
cd ..
catkin_make
source devel/setup.sh
roscore
```
Plug in your Seek Camera.
In a new terminal
```
cd ~/catkin_ws
source devel/setup.sh
roslaunch seek_driver seek_driver.launch
```
In a new terminal
```
cd ~/catkin_ws
source devel/setup.sh
rviz
```

Navigate to the 'add' at the bottom left, select 'By Topic'. Under `seek_camera/displayImage` select 'Image'. Select 'Okay'.

the rviz window should looking similar to this:

![ros_seek](https://user-images.githubusercontent.com/38704785/112682748-effb2280-8e46-11eb-964f-3178d762d4d9.png)


