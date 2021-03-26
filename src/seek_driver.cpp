// Author(s): Daniel Bronstein (dbronste@andrew.cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

#include "seek_driver.h"

SeekDriver::SeekDriver() : private_nh_("~"), camera_(NULL)
{
	initRos();

	initCamera();

}

void SeekDriver::run()
{

	//if we have a camera
	if (camera_){

		//Grab image, process return code
		sw_retcode returnErrorCode = Seekware_GetImageEx(camera_, 
										filteredWithTelemetryData_.data(), 
										thermographyData_.data(), 
										displayData_.data());
		checkForError(returnErrorCode, "Getting Image from Camera");

		//published message header
		std_msgs::Header head;
		head.seq = frameCount_;
		head.stamp = ros::Time::now();

		//publish display image
		sensor_msgs::ImagePtr displayMsg = cv_bridge::CvImage(head, 
			"bgra8", displayImageMatrix_).toImageMsg();
		displayImagePub_.publish(displayMsg);

		//publish thermography image
		sensor_msgs::ImagePtr tempMsg = cv_bridge::CvImage(head, 
			"32FC1", temperatureImageMatrix_).toImageMsg();
		temperatureImagePub_.publish(tempMsg);

		//extract telemetry data
		size_t camera_pixels = camera_->frame_cols * camera_->frame_rows;
		seek_driver::telemetryData telData;
		telData.header = head;
		telData.field_count = *reinterpret_cast<unsigned int*>(&filteredWithTelemetryData_[camera_pixels]);
		telData.temp_diode_mv = *reinterpret_cast<unsigned short*>(&filteredWithTelemetryData_[camera_pixels + 2]);
		telData.env_temp = *reinterpret_cast<float*>(&filteredWithTelemetryData_[camera_pixels + 3]);
		telData.internal_timestamp_micros = *reinterpret_cast<unsigned long*>(& filteredWithTelemetryData_[camera_pixels + 5]);
		telemetryPub_.publish(telData);

		//publish filtered image data
		sensor_msgs::ImagePtr filteredMsg = cv_bridge::CvImage(head, 
			"mono16", filteredImageMatrix_).toImageMsg();
		filteredImagePub_.publish(filteredMsg);

		++frameCount_;
	}
}

bool SeekDriver::initCamera() 
{
	sw_retcode mostRecentReturn;//hold return codes to process
	bool isErr;

	int numCamsToFind = 1;
	sw* cameraList[numCamsToFind];
	int numCamsFound = 0;
	mostRecentReturn = Seekware_Find(cameraList, numCamsToFind, &numCamsFound);
	isErr = checkForError(mostRecentReturn, "Finding Camera");

	if (isErr) {
		return false;
	}

	if (numCamsFound == 0) 
	{
		ROS_ERROR("Seek camera not found");//TODO: Replace with custom message
		return false;
	}

	//temporary pointer to camera in case initialization fails partway through
	//Class variable set at end if initialization is successful
	//This keeps camera_ NULL if initialization fails
	sw* tempCamera;

	//grab first (only) camera from camera list
	tempCamera = cameraList[0];

	//Stop camera first, in case failed close last time
	mostRecentReturn = Seekware_Stop(tempCamera);
	isErr = checkForError(mostRecentReturn, "Stopping Camera pre-Open");
	if (isErr) {
		return false;
	}

	//Alright now open it
	mostRecentReturn = Seekware_Open(tempCamera);
	isErr = checkForError(mostRecentReturn, "Opening Camera");
	if (isErr) {
		return false;
	}

	int totalNumPixels = tempCamera->frame_rows * tempCamera->frame_cols;

	//initialize image buffers
	thermographyData_ = std::vector<float>(totalNumPixels);
	filteredData_ = std::vector<unsigned short>(totalNumPixels);
	filteredWithTelemetryData_ = std::vector<unsigned short>(totalNumPixels + tempCamera->frame_cols);
	displayData_ =  std::vector<unsigned int>(totalNumPixels);

	//reset and start timer TODO: I believe this sets it to zero, need to
	//adjust returns to get in line with the wall clock
	int enable = 1;
	mostRecentReturn = Seekware_SetSettingEx(tempCamera, SETTING_ENABLE_TIMESTAMP, &enable, sizeof(enable));
	isErr = checkForError(mostRecentReturn, "Enabling timestamp");
	if (isErr) {
		Seekware_Close(tempCamera);
		return false;
	}
	mostRecentReturn = Seekware_SetSettingEx(tempCamera, SETTING_RESET_TIMESTAMP, &enable, sizeof(enable));
	isErr = checkForError(mostRecentReturn, "Resetting timestamp");
	if (isErr) {
		Seekware_Close(tempCamera);
		return false;
	}

	//Setting lookup table for gain controlled display image
	mostRecentReturn = Seekware_SetSetting(tempCamera, SETTING_ACTIVE_LUT, SW_LUT_WHITE_NEW);
	isErr = checkForError(mostRecentReturn, "Setting LUT");
	if (isErr) {
		Seekware_Close(tempCamera);
		return false;
	}

	//WOOHOO camera initialized successfuly

	//using ptr version requires no updating in run loop
	displayImageMatrix_ = cv::Mat(tempCamera->frame_rows, 
	tempCamera->frame_cols, CV_8UC4, displayData_.data());

	temperatureImageMatrix_ = cv::Mat(tempCamera->frame_rows,
	tempCamera->frame_cols, CV_32FC1, thermographyData_.data());

	filteredImageMatrix_ = cv::Mat(tempCamera->frame_rows, 
	tempCamera->frame_cols, CV_16UC1, filteredWithTelemetryData_.data());

	frameCount_ = 0;

	//succesful
	camera_ = tempCamera;
	return true;
}

bool SeekDriver::restartCameraServiceCB(seek_driver::restartCameraRequest& req, 
															seek_driver::restartCameraResponse& resp)
{
	sw_retcode mostRecentReturn;//hold return codes to process
	
	//if there is a camera, close it out
	if (camera_)
	{
		mostRecentReturn = Seekware_Close(camera_);
		checkForError(mostRecentReturn, "Closing camera on restart");
		camera_ = NULL;
	}
	
	resp.success = initCamera();
	return resp.success;
}

bool SeekDriver::checkForError(sw_retcode returnError, std::string context)
{
	if (returnError != SW_RETCODE_NONE) 
	{
		std::ostringstream errStringStream;
		errStringStream << "Seek Failed in context: " << context
		<< " \n\tReturn Code: " <<  returnError;

		//TODO: Replace with custom message
		ROS_ERROR("%s", errStringStream.str().c_str());

		return true;
	} else {
		return false;
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
	private_nh_.param<double>("timerFreq_", timerFreq_, 9.0);  // Hz

	// set subscriber

	// set publisher
	image_transport::ImageTransport it(nh_);
	displayImagePub_ = it.advertise("seek_camera/displayImage", 10);
	
	temperatureImagePub_ = it.advertise("seek_camera/temperatureImageCelcius", 10);

	telemetryPub_ = nh_.advertise<seek_driver::telemetryData>("seek_camera/telemetry",10);

	filteredImagePub_ = it.advertise("seek_camera/filteredImage", 10);

	//setup service servers
	restartService_ = nh_.advertiseService("seek_camera/restart_seek", &SeekDriver::restartCameraServiceCB, this);
	
	// set timers
	timer_ = nh_.createTimer(ros::Rate(timerFreq_), &SeekDriver::timerCallback, this);
}

SeekDriver::~SeekDriver() {
	if (camera_) 
	{
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
