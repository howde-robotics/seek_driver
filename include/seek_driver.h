// Author(s): Daniel Bronstein (dbronste@andrew.cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#pragma once

// std libraries
#include <iostream>
#include <vector>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//my srvs and msgs
#include "seek_driver/restartCamera.h"
#include "seek_driver/telemetryData.h"

// Include other header files
#include <seekware.h>
#include <opencv2/opencv.hpp>

/**
 * @brief This is the driver for the Seek camera.
 * The class operates on a timer, collecting images from the
 * camera and pumping them into messages for the rest of the
 * system.
 *
 * On init: ROS publishers/subscribers/timers/callbacks are initialized
 *    Camera is initialized
 *
 * On timer loop: Camera images are captured, pumped into messages
 *
 * @see Seek API documentation for explanations of the function
 * calls
 *
 */
class SeekDriver
{
public:

  /**
   * @brief Construct a new Seek Driver object. This initilizes
   ros publishers/subscribers/services and then attempts to initialize
   the Seek camera.
   *
   */
  SeekDriver();

  /**
   * @brief Destroy the Seek Driver object. Seek camera
   needs to be closed manually, or else it may have issues
   on re-boot
   *
   */
  ~SeekDriver();

private:
  /////////////////////// State variables ///////////////////////

  //Pointer to the camera object. At all times, a non-NULL state should
  //correspond to an active, valid camera, NULL the inverse
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

  //Pixel-wise temperature values estimated from the raw bolometer
  //readings and the internal temperature diode
  cv::Mat temperatureImageMatrix_;

  //Partially filtered sensor array readings. NOT gain controlled
  //NOT temperature values.
  cv::Mat filteredImageMatrix_;

  /////////////////////// calculation variables ///////////////////////



  /**
   * @brief The main loop that is run at a chosen frequency.
   If there is a camera present (i.e. if camera_ is not NULL),
   the following happens:
   1. filtered data, telemetry, thermography and display data
   are exfiltrated from the camera
   2. A header is made using the current ROS time, NOT the camera
   time
   3. filtered data, telemetry, thermography and display data
   are published to their respective channels
   4. The frame count is incremeneted
   */
  void run();

  /**
   * @brief This is the callback function for the restart_seek
   service. If the camera currently exists (i.e. is not NULL)
   then the camera is closed. Then the camera and data containers
   are re-initilized
   *
   * @param req Empty request, defined in seek_drive/srv
   * @param resp Boolean response of success, defined in seek_driver/src
   * @return true if camera is found and reinitilized without errors
   * @return false otherwise
   */
  bool restartCameraServiceCB(seek_driver::restartCameraRequest& req,
                              seek_driver::restartCameraResponse& resp);

  /**
   * @brief Used to process the Seek return codes, which are returned
   * by most calls to the seek API. Checks if an error has occured,
   * and will print to ROS_ERROR. If there is nothing abnormal
   * with the return code, then this call is silent
   *
   * @param returnError Seek API Return Error Code
   * @param context Context that the error occurred in e.g.
   * finding camera, closing camera
   * @return true If returnError != SW_RETCODE_NONE
   * @return false If there is a returncode error
   */
  bool checkForError(sw_retcode returnError, std::string context);

  /**
   * @brief Attempts to find a single camera and initialize it
   If an error occurs during this process, the class member
   camera_ remains NULL. Goes through the following initialization
   sequence, checking for errors after each call to the Seek API:
   1. Attempts to find the camera
   2. Attempts to stop the camera (this helps if it was improperly
   closed last run)
   3. Attempts to open the camera
   4. Instantiates the vectors that will capture seek returns
   5. Attempts to enable and restart camera timer
   6. Attempts to set the camera LUT
   7. Instantiates the CV::mats that will be used to hold and send
   the image frames
   8. Sets the camera_ member variable to point to the newly
   initialized camera.
   *
   * @return true If camera found and initialized successfuly
   * @return false otherwise
   */
  bool initCamera();

  // ROS and node related variables
  ros::NodeHandle nh_, private_nh_;
  ros::Timer timer_;
  double timerFreq_;

  //Publishers
  image_transport::Publisher displayImagePub_;//uses cv_bridge
  image_transport::Publisher temperatureImagePub_;//uses cv_bridge
  ros::Publisher telemetryPub_;//uses custom msg
  image_transport::Publisher filteredImagePub_;//uses cv_bridge

  //Restart service
  ros::ServiceServer restartService_;

  /**
   * @brief Used to gather params, advertise publishers
   set subscribers, instantiate timers, advertise services
   *
   */
  void initRos();

  /**
   * @brief Timer call back with a frequency defined by
   timerFreq_. Forwards directly to run()
   *
   * @param e ROS timer event
   */
  void timerCallback(const ros::TimerEvent& e);

};
