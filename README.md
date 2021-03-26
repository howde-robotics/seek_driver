# seek_driver
Seek ROS Driver
This is a rudimentary ROS driver to run a Seek thermal camera leveraging the Seek SDK.
Tested with the Seek SDK 3.8.0.0 using a S304SP Mosaic Core Starter Kit and ROS melodic

This package provides two primary functions:
- It publishes the display Image, filtered Image, thermography image at a set interval
- It provides a service to restart the camera

# Dependencies
Seek SDK [Tested with 3.8.0.0]
ROS [Tested with Melodic]
OpenCV [Tested with 4.5.1]

