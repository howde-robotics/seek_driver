// Author(s): Daniel Bronstein (dbronste@andrew.cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#include "seek_driver.h"

SeekDriver::SeekDriver() : private_nh_("~"), camera_(NULL)
{
  initRos();

  sw_retcode mostRecentReturn;//hold return codes to process

  int numCamsToFind = 1;
  sw* cameraList[numCamsToFind];
  int numCamsFound = 0;
  mostRecentReturn = Seekware_Find(cameraList, numCamsToFind, &numCamsFound);
  processReturnCode(mostRecentReturn, "Finding Camera");

  if (numCamsFound == 0 || !camera_) {
		ROS_ERROR("Seek camera not found");//TODO: Replace with custom message
    return;
	}

  //grab first (only) camera from camera list
  camera_ = cameraList[0];

  //Stop camera first, in case failed close last time
	mostRecentReturn = Seekware_Stop(camera_);
	processReturnCode(mostRecentReturn, "Stopping Camera pre-Open");

  //Alright now open it
	mostRecentReturn = Seekware_Open(camera_);
	processReturnCode(mostRecentReturn, "Opening Camera");

  int totalNumPixels = camera_->frame_rows * camera_->frame_cols;

  //initialize image buffers
  thermographyData_ = std::vector<float>(totalNumPixels);
  filteredData_ = std::vector<unsigned short>(totalNumPixels);
  filteredWithTelemetryData_ = std::vector<unsigned short>(totalNumPixels + camera_->frame_cols);
  displayData_ =  std::vector<unsigned int>(totalNumPixels);

  //reset and start timer TODO: I believe this sets it to zero, need to
  //adjust returns to get in line with the wall clock
  int enable = 1;
	Seekware_SetSettingEx(camera_, SETTING_ENABLE_TIMESTAMP, &enable, sizeof(enable));
	Seekware_SetSettingEx(camera_, SETTING_RESET_TIMESTAMP, &enable, sizeof(enable));

  //using ptr version requires no updating in run loop
  displayImageMatrix_ = cv::Mat(camera_->frame_rows, 
    camera_->frame_cols, CV_8UC4, displayData_.data());

  frameCount_ = 0;

}

void SeekDriver::run()
{
  //if we have a camera (remember, its a pointer)
  if (camera_){
    //Grab image, process return code
    sw_retcode returnErrorCode = Seekware_GetImageEx(camera_, 
                    filteredWithTelemetryData_.data(), 
                    thermographyData_.data(), 
                    displayData_.data());
    processReturnCode(returnErrorCode, "Getting Image from Camera");

    std_msgs::Header head;
    head.seq = frameCount_;
    head.stamp = ros::Time::now();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(head, 
      "bgr8", displayImageMatrix_).toImageMsg();
  
    displayImagePub_.publish(msg);
    ++frameCount_;
  }
}

void SeekDriver::processReturnCode(sw_retcode returnError, std::string context)
{
  if (returnError != SW_RETCODE_NONE) {
      std::ostringstream errStringStream;
      errStringStream << "Seek Failed in context: " << context
      << " \n\tReturn Code: " <<  returnError;

      //TODO: Replace with custom message
      ROS_ERROR("%s", errStringStream.str().c_str());
    }
}

// put boiler plate ros function at the bottom
void SeekDriver::timerCallback(const ros::TimerEvent& e)
{
  this->run();
}


void SeekDriver::initRos()
{
  //default doesn't need to be over 9.0 since that is camera frame rate
  //limit
  private_nh_.param<double>("timerFreq_", timerFreq_, 9.0);  // Hz

  // set subscriber
  // odomSub_ = nh_.subscribe("/odom", 1, &SeekDriver::odomCallback, this);

  // set publisher
  image_transport::ImageTransport it(nh_);
  displayImagePub_ = it.advertise("seek_camera/displayImage", 10);
  // vehicleStatsPub_ = nh_.advertise<std_msgs::String>("/vehicle_status", 10);

  // set timers
  timer_ = nh_.createTimer(ros::Rate(timerFreq_), &SeekDriver::timerCallback, this);
}

SeekDriver::~SeekDriver() {
  if (camera_ != NULL) {
		Seekware_Close(camera_);
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seek_driver");
  SeekDriver node;
  while (ros::ok())
    ros::spinOnce();
  return 0;
}
