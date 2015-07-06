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

//Calibration Values for each colour
#define RED_H_MIN_2		0
#define RED_H_MAX_2		3
#define RED_H_MIN		170//121
#define RED_H_MAX		186//237
#define RED_S_MIN		72//76
#define RED_S_MAX		255
#define RED_V_MIN		100//72
#define RED_V_MAX		255

#define BLUE_H_MIN		77
#define BLUE_H_MAX		123
#define BLUE_S_MIN		82
#define BLUE_S_MAX		255
#define BLUE_V_MIN		64
#define BLUE_V_MAX		255

#define GREEN_H_MIN		50
#define GREEN_H_MAX		72
#define GREEN_S_MIN		106
#define GREEN_S_MAX		255
#define GREEN_V_MIN		0
#define GREEN_V_MAX		255

#define YELLOW_H_MIN	29
#define YELLOW_H_MAX	41
#define YELLOW_S_MIN	115
#define YELLOW_S_MAX	255
#define YELLOW_V_MIN	129
#define YELLOW_V_MAX	255

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
