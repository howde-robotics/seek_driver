// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

// include in the cpp node file should only be the .h file
// any additional include should go to .h file
#include "seek_driver.h"

// convention is to put Constructor at the very top
SeekDriver::SeekDriver() : private_nh_("~")
{
  // put boiler plate ROS stuff in this function
  initRos();

  // initialise variables if you need to and add explanation to magic numbers

}

// convention is to put the most important function right after the constructor
void SeekDriver::run()
{
  // do really cool and important stuff here
  ROS_INFO("In Run");
  // publish the result of your calculation / algo
  // vehicleStatsPub_.publish(vehicleStatsMsg_);
}

// put boiler plate ros function at the bottom
void SeekDriver::timerCallback(const ros::TimerEvent& e)
{
  this->run();
}


void SeekDriver::initRos()
{
  // param syntax is: param name in launch, variable name in source code, and default value
  // private_nh_.param<int>("myVar1_", myVar1_, 1);
  // private_nh_.param<double>("myVar2_", myVar2_, 5.0);
  private_nh_.param<double>("timerFreq_", timerFreq_, 20.0);  // Hz

  // set subscriber
  // odomSub_ = nh_.subscribe("/odom", 1, &SeekDriver::odomCallback, this);

  // set publisher
  // vehicleStatsPub_ = nh_.advertise<std_msgs::String>("/vehicle_status", 10);

  // set timers
  timer_ = nh_.createTimer(ros::Rate(timerFreq_), &SeekDriver::timerCallback, this);
}

SeekDriver::~SeekDriver() {
  
}

// keep the main function as clean as possible
int main(int argc, char** argv)
{
  ros::init(argc, argv, "seek_driver");
  SeekDriver node;
  while (ros::ok())
    ros::spinOnce();
  return 0;
}
/*
 * My Questions:
 *  - Parameters in the launch files associated with a node are public?
 *    And the default initialized done by private_nh is private?
 *  - Use of timer vs sleep in main function
 */