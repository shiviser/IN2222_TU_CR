#pragma once

// standard includes
#include <iostream>

// third party includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/init.h>
#include <camera_srvs/CameraInfo.h>
#include <comm_lib/ImageSubscriber.h>
#include <vector>
#include "detector_classes/HSV_Param.h"
#include "detector_classes/Shape.h"


//max number of objects to be detected in frame
#define MAX_NUM_OBJECTS	50
//minimum and maximum object area
#define MIN_OBJECT_AREA	(40*40)
#define MAX_OBJECT_AREA	(FRAME_HEIGHT*FRAME_WIDTH/1.5)

#define NR_OF_COLOURS	4

//default capture width and height
#define FRAME_WIDTH	640
#define FRAME_HEIGHT	480

#define CENTER_BOX_SIZE		180
#define CENTER_BOX_X_MIN	((FRAME_WIDTH-CENTER_BOX_SIZE)/2)
#define CENTER_BOX_X_MAX	(FRAME_WIDTH-CENTER_BOX_X_MIN)
#define CENTER_BOX_Y_MIN	((FRAME_HEIGHT-CENTER_BOX_SIZE)/2)
#define CENTER_BOX_Y_MAX	(FRAME_HEIGHT-CENTER_BOX_Y_MIN)

class ShapesDetector {
private:
	comminterface::ImageSubscriber* img_subs;
	

	
	
	string intToString(int number);
	void morphOps(Mat &thresh);
	bool scan_colour(Mat HSV_img,HSV_Param Filt_Type, int thres);
	
public:
	cv::Mat img;
	cv::Mat img_info;
	vector<Shape> Shapes;
	//
	ShapesDetector(ros::NodeHandle *n);
	
	bool get_img(bool debug);
	bool find_shapes(int thres, bool debug, vector<HSV_Param> HSV_Types);
	void display_shapes(int thres);
	// destructor
	~ShapesDetector();
};
