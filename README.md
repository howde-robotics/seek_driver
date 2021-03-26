# seek_driver
Seek ROS Driver. This is a rudimentary ROS driver to run a Seek thermal camera leveraging the Seek SDK.
Tested with the Seek Linux SDK 3.8.0.0 using a S304SP Mosaic Core Starter Kit and ROS Melodic

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

# ROS Information
## Topics Published
### Imagery

Imagery is published using `cv_bridge` and `image_transport`

`/seek_camera/displayImage` Is the display Image

`/seek_camera/filteredImage` Is the filtered image

`/seek_camera/temperatureImageCelcius` Is the thermography image

The data types of the images are variable (BGRA32 for display image, 16 bit unsigned int for filtered image, 32 bit float for thermography). If inspected using `rostopic echo`, they will appear to be 8 bit unsigned int values. This is an artifact of the `image_transport` library. Once the images are decoded on the other side, they will return to their native formats.

### Telemetry
Telemetry information is published using a custom msg defined in msgs, over the topic `/seek_camera/telemetry`



## Services Provided
This node also provides a ROS service to restart the camera is if was disconnect or experiencing errors. This srv is defined in srvs and available under `seek_driver/restartCamera`

An example use. Launch the ROS node with the seek camerea **NOT** plugged in.
```
roslaunch seek_driver seek_driver.launch
```

This will show a ROS error and no information will be streamed. Plug in the camera and call the restart service

```
rosservice call /seek_camera/restart_seek
```
The camera should now be operational

# Decoding Imagery
Decoding imagery is very simple after it has been published using `cv_bridge`. A subscriber callback of the following variety is effective:
```
void SeekListener::displayDataCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat displayImage = cv_bridge::toCvShare(msg, "bgra8")->image;
  //Do something with the displayImage, which is a gain controlled
  //and colored version of the filtered Data
}
```
The second parameter of `toCvShare` is the `cv_bridge` encoding string. For display imagery, it is `"bgra8"`, for thermography images is `"32FC1"` and for filtered imagery is `"mono16"`
