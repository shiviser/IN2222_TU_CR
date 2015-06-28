#pragma once

// standard includes
#include <iostream>

// third party includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/init.h>
#include <camera_srvs/CameraInfo.h>
#include <comm_lib/ImageSubscriber.h>


class ShapesDetector {
private:
	comminterface::ImageSubscriber* img_subs;

	int mWidth;
	int mHeight;
	cv::Mat projectionMat;
	cv::Mat intrinsicsMat;
	cv::Mat extrinsicsMat;

	//
	void init_calibration_data(ros::NodeHandle *n);

	//
	void get_screen_to_3Dpoints(const cv::Point2i& positions2D, cv::Point3d& positions3D, double z);

public:
	//
	ShapesDetector(ros::NodeHandle *n);

    //
    cv::Mat to_gray(const cv::Mat& img_bgr, bool debug=false);

	//finds polygons and filters squares, triangles and circles
    std::vector<unsigned int> get_shapes(const cv::Mat& img_bw, std::vector<std::vector<cv::Point> > &shape_positions, std::vector<cv::Vec3f>& circles, int mode=CV_RETR_TREE, int method=CV_CHAIN_APPROX_SIMPLE, bool debug=false);

	std::vector<unsigned int> get_colours(const std::vector<std::vector<cv::Point> >& shape_positions, const cv::Mat& image, bool debug=false);
    
	unsigned int get_circle_colour(const cv::Vec3f& circle, const cv::Mat& image, bool debug);

	cv::Point get_middle(const std::vector<cv::Point> position);

	std::vector<int> get_object(int thres, bool debug);

	// destructor
	~ShapesDetector();
};
