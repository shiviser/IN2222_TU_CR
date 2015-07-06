#include "ros/ros.h"
#include "shape_detect_srvs/shape.h"
#include "shape_detect_srvs/shape_vec.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include <cstdlib>
#include <iostream>

#include <sstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "main.h"
#include "detector_classes/HSV_Param.h"
#include "detector_classes/Shape.h"
#include "detector_classes/shapes_detector.h"

using namespace std;
using namespace cv;
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

vector<HSV_Param> HSV_Types;
ShapesDetector *glob_shapes_detector;

//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

void init_HSV_Types() {
	HSV_Param Temp1(red);
	HSV_Param Temp2(green);
	HSV_Param Temp3(blue);
	HSV_Param Temp4(yellow);

	HSV_Types.push_back(Temp1);
	HSV_Types.push_back(Temp2);
	HSV_Types.push_back(Temp3);
	HSV_Types.push_back(Temp4);
}


void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}

void createTrackbars(){
	//create window for trackbars


	namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );


}

bool srv_call_shapes(shape_detect_srvs::shape_vec::Request  &req, shape_detect_srvs::shape_vec::Response &res) {
  shape_detect_srvs::shape temp_shape;

  glob_shapes_detector->find_shapes(req.filter, DEBUG, HSV_Types);

  for(int i=0; i<glob_shapes_detector->Shapes.size() ; i++) {
	temp_shape.xPos = glob_shapes_detector->Shapes[i].getXPos();
  	temp_shape.yPos = glob_shapes_detector->Shapes[i].getYPos();
	
	temp_shape.colour = glob_shapes_detector->Shapes[i].getColour();
	temp_shape.shape = glob_shapes_detector->Shapes[i].getShape();
	
	
	/*switch(glob_shapes_detector->Shapes[i].getColour()) {
		case red:
			temp_shape.colour = "red";
			break;
		case blue:
			temp_shape.colour = "blue";
			break;
		case green:
			temp_shape.colour = "green";
			break;
		case yellow:
			temp_shape.colour = "yellow";
			break;
		default:
			break;
	}

	switch(glob_shapes_detector->Shapes[i].getShape()) {
		case triangle_sh:
			temp_shape.shape = "triangle";
			break;
		case square_sh:
			temp_shape.shape = "square";
			break;
		case circle_sh:
			temp_shape.shape = "circle";
			break;		
		default:
			break;
	}*/
	
	
  	res.shapes.push_back(temp_shape);
  }

  ROS_INFO("Shape_vec requested");
  return true;
}


int main(int argc, char **argv)
{
  	float x_pos, y_pos, z_pos;

	//Matrix to store each frame of the webcam feed
	cv::Mat HSV;
	cv::Mat threshold;


  	ros::init(argc, argv, "shape_detector");

  	ros::NodeHandle n;

	ShapesDetector shapes_detector(&n);
	glob_shapes_detector = &shapes_detector;
  
  	//shape service
  	ros::ServiceServer service = n.advertiseService("/shape_detector/shapes", srv_call_shapes);
  	//ROS_INFO("Ready!!");

	

#ifdef CALIBRATION_MODE
	createTrackbars();
#endif

	init_HSV_Types();

	while(ros::ok()){
#ifdef CALIBRATION_MODE
		//if in calibration mode, we track objects based on the HSV slider values.
		shapes_detector.get_img(DEBUG);
		cvtColor(shapes_detector.img,HSV,COLOR_BGR2HSV);
		inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
		imshow("Calibration",threshold);
		imshow("Image",shapes_detector.img);

		waitKey(30);
#else
		//ros::spin();
		ros::spinOnce();
		waitKey(30);
		glob_shapes_detector->find_shapes(40, DEBUG, HSV_Types);
#endif
	}

  
  return 0;
}
