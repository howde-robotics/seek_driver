// Author(s): Daniel Bronstein (dbronste@andrew.cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#pragma once

// std libraries
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>  
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "seek_driver/temperatureImage.h"

// Include other header files
#include <seekware.h>
#include <opencv2/opencv.hpp>

/**
 * @brief This is the driver for the Seek camera.
 * The class operates on a timer, collecting images from the 
 * camera and pumping them into messages for the rest of the
 * system.
 * 
 * On init: Camera is found and opened
 * 
 * On timer loop: Camera image is captured, pumped into a message
 * 
 * @see Seek API documentation for explanations of the function
 * calls  
 * 
 */
class SeekDriver
{
public:
  /**
   * @brief Construct a new Seek Driver object. This is where the
   * camera is found and activated. If no camera is found, 
   * the class member will remain a NULL pointer. Various errors can
   * occur while opening the camera, all of which should log to the
   * ROS_ERROR stream and TODO: send custom error message to watchdog.
   * 
   * Once the camera hardware is located, a call to stop the camera
   * happens first. This can help if the camrea was incorrectly close
   * last time and needs to be rebooted.
   * 
   * Next the camera is opened and space is allocated for the image
   * returns in CV::mats of varying types.
   * 
   * Then the seek camera settings are set, a LUT is chosen, 
   * and 
   * 
   */
  SeekDriver();

  /**
   * @brief Destroy the Seek Driver object. Seek camera
   * needs to be closed manually, or else it may have issues
   * on re-boot
   * 
   */
  ~SeekDriver();

private:
  /////////////////////// State variables ///////////////////////

  sw* camera_;

  //Container for temperature values from camera
  std::vector<float> thermographyData_;

  //Container for semi-processed sensor readings from the camera.
  //NOTE: Although this sounds like the 'typical' IR image return, it
  //is NOT. Its the semi-processed direct sensor reading with no gain
  //control. If you try and view the image immediately it will look weird
	std::vector<unsigned short> filteredData_;

  //Same as above but with an extra row appended to the bottom
  //of the data containing camera telemetry data. See Seek API documentation
	std::vector<unsigned short> filteredWithTelemetryData_;

  //This is the gain controlled IR camera returns, what you
  //typically think of when you see a IR image
	std::vector<unsigned int> displayData_;

  int frameCount_ = 0;
  
  /**
   * Display Data returned from Seek is noted as ARGB888
   * in the Seek API. The encoding for CV_8UC4 (8 bit, 4 channel)
   * is BGRA32. While these may seem different, the ARGB888 is
   * little endian notation, thus the channels require no mixing
   * and can be put directly in a cv matrix of type CV_8UC4. Alpha
   * channels is fully saturated for native seek returns, will
   * only be used if you implement a custom LUT
   * 
   * In this implementation, the cv matrix is pointed directly
   * to the image buffer, so the matrix itself doesn't need to
   * be updated every run loop
   */
  cv::Mat displayImageMatrix_;

  cv::Mat thermographyImageMatrix_;
  /////////////////////// calculation variables ///////////////////////

 

  /**
   * @brief main loop over the ros timer that does this, process what, and does whaaat
   *
   * This is pretty straightforward: 
   */
  void run();

  /**
   * @brief Used to process the Seek return codes, which are returned
   * by most calls to the seek API. Checks if an error has occured,
   * and will print to ROS_ERROR. If there is nothing abnormal
   * with the return code, then this call is silent
   * 
   * @param returnError Seek API Return Error Code
   * @param context Context that the error occurred in e.g. 
   * finding camera, closing camera
   */
  void processReturnCode(sw_retcode returnError, std::string context);

  // ROS and node related variables
  ros::NodeHandle nh_, private_nh_;
  ros::Timer timer_;
  double timerFreq_;

  image_transport::Publisher displayImagePub_;//uses cv_bridge
  ros::Publisher thermographyImagePub_;//uses sensor_msgs/channelFloat32

  // Boiler plate ROS functions
  void initRos();
  void timerCallback(const ros::TimerEvent& e);

};
