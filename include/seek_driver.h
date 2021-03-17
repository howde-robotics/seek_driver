// Author(s): Daniel Bronstein (dbronste@andrew.cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

// always add include guard for all .h files
#pragma once

// std libraries
#include <iostream>
#include <vector>
#include <cmath>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

// Include other header files
#include "seekware.h"
#include <opencv2/opencv.hpp>

/**
 * @brief Explain briefly what this class does, e.g. it takes in this input, does this process and output this
 *
 * This part is now the detailed explanation of the class, you can write entire length essays (highly encouraged btw)
 * Explain what algo you use, any important details
 *
 * @see PythonNode
 * @return publish this topic of type
 */
class SeekDriver
{
public:
  SeekDriver();
  ~SeekDriver();

private:
  /////////////////////// State variables ///////////////////////


  /////////////////////// calculation variables ///////////////////////

 

  // put your most important function and other helper functions here
  /**
   * @brief main loop over the ros timer that does this, process what, and does whaaat
   *
   * talk more here plsss
   * Seriously, if this is your most important function you better be writing essays here
   */
  void run();

  // ROS and node related variables
  ros::NodeHandle nh_, private_nh_;
  ros::Timer timer_;
  double timerFreq_;

  // Boiler plate ROS functions
  void initRos();
  void timerCallback(const ros::TimerEvent& e);

};
