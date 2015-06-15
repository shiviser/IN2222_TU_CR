#include "shapes_detector.h"


ShapesDetector::ShapesDetector(ros::NodeHandle *n) {
	init_calibration_data(n);

	img_subs = new comminterface::ImageSubscriber("/camera/image");
	img_subs->start();
};


cv::Mat ShapesDetector::to_gray(const cv::Mat& img_bgr, bool debug) {
    // todo if already b/w return it

    // Create a new matrix to hold the gray image
    cv::Mat img_gray;
 
    // convert RGB image to gray
    cv::cvtColor(img_bgr, img_gray, CV_BGR2GRAY);
    
    if (debug) {
        cv::namedWindow("Gray Image", CV_WINDOW_AUTOSIZE);
        cv::imshow("Gray Image", img_gray);
		 cv::waitKey(0);
    }

    return img_gray;
};


std::vector<unsigned int> ShapesDetector::get_shapes(const cv::Mat& img_gray, std::vector<std::vector<cv::Point> > &shape_positions, std::vector<cv::Vec3f>& circles, int mode, int method, bool debug) {
	cv::Mat img_bw; // = ShapesDetector::threshold_gray(img_gray, 80, cv::THRESH_BINARY, true);
	cv::Canny(img_gray, img_bw, 0, 50, 3); 

	/* cv::namedWindow("B/W Image", CV_WINDOW_AUTOSIZE);
        cv::imshow("B/W Image", img_bw);
        cv::waitKey(0);
		 */

    // extract the contours
    std::vector<std::vector<cv::Point> > contours;
    findContours(img_bw.clone(), contours, mode, method);

	std::vector<std::vector<cv::Point> > polygons;
    polygons.resize(contours.size());

	std::vector<unsigned int> shape_types;

	std::cout << "contours.size: " << contours.size() << std::endl;

	double maxContourSize = 0;

    for (size_t i = 0; i < contours.size(); i++) {
        cv::approxPolyDP(cv::Mat(contours[i]), polygons[i], 4, true);
 
		if (std::fabs(cv::contourArea(contours[i])) < 5000) //TODO
		    continue;

		if(polygons[i].size() == 3) {
			shape_positions.push_back(polygons[i]);
			shape_types.push_back(3);

			//to find appropriate size
			if(std::fabs(cv::contourArea(contours[i])) > maxContourSize)
			maxContourSize = std::fabs(cv::contourArea(polygons[i]));
		}
		else if(polygons[i].size() == 4) {
			//TODO: check if really square or not necessary because they aren't going to be any other big rectangles?

			shape_positions.push_back(polygons[i]);
			shape_types.push_back(4);

			//to find appropriate size
			if(std::fabs(cv::contourArea(contours[i])) > maxContourSize)
			maxContourSize = std::fabs(cv::contourArea(polygons[i]));
		}
    }

	if(debug)std::cout << "shapes: " << shape_positions.size() << std::endl;


	//std::vector<cv::Vec3f> circles;
	cv::HoughCircles(img_gray, circles, CV_HOUGH_GRADIENT, 1, 50, 100, 50, 0, 0 );

	if(debug) std::cout << "circles: " << circles.size() << std::endl;

	std::cout << "maxContourSize: " << maxContourSize << std::endl;

		// visual debugging
    if (debug) {
        cv::RNG rng(12345);

        cv::Mat drawing;
        cvtColor(img_gray, drawing, CV_GRAY2BGR);
        for (unsigned int i = 0; i < shape_positions.size(); i++) {
				cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				cv::drawContours(drawing, shape_positions, i, color, 3, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        }

		for( size_t i = 0; i < circles.size(); i++ )
		{
			 cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			 int radius = cvRound(circles[i][2]);
			 // draw the circle center
			 cv::circle( drawing, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
			 // draw the circle outline
			 cv::circle( drawing, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
		}

        // Show in a window
        cv::namedWindow("Polygons", CV_WINDOW_AUTOSIZE);
        cv::imshow("Polygons", drawing);
        cv::waitKey(0);
    }
	
    return shape_types;
};


std::vector<unsigned int> ShapesDetector::get_colours(const std::vector<std::vector<cv::Point> >& shape_positions, const cv::Mat& image, bool debug) {
	std::vector<unsigned int> colours;
	for (size_t i_shape = 0; i_shape < shape_positions.size(); i_shape++)
    {
        //bounding box for contour
        cv::Rect roi = cv::boundingRect(shape_positions[i_shape]); 

        //create mask
        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::drawContours(mask, shape_positions, i_shape, cv::Scalar(255), CV_FILLED); 

		// extract shape
        cv::Mat imageROI;
        image.copyTo(imageROI, mask);

		cv::Scalar mean = cv::mean(imageROI,mask);

		if(debug) std::cout << "Scalar: " << mean << std::endl; //BGR - Format !!

		//values for the colours may change when using the robotCam
		cv::Scalar red (61,57,150,0);//			(0,0,255,0);
		cv::Scalar green (40,115,34);//		(0, 255,0,0);
		cv::Scalar blue (129,84,48);//		(255,0,0,0);
		cv::Scalar yellow (84,155,143);//		(0,255,255,0);

		double norm_blue = cv::norm(mean-blue);
		double norm_green = cv::norm(mean-green);
		double norm_red = cv::norm(mean-red);
		double norm_yellow = cv::norm(mean-yellow);

		double norms[] = {norm_blue, norm_green, norm_red, norm_yellow};
		double min = 300;
		int index = 0;
		for(unsigned int i_n = 0; i_n < 4; i_n++) {
			if(norms[i_n] < min) {
				min = norms[i_n];
				index = i_n;
			}
		}
		if(debug) {
			switch(index)
			{
			case 0:
				std::cout << "BLUE" << std::endl;
				break;
			case 1:
				std::cout << "GREEN" << std::endl;
				break;
			case 2:
				std::cout << "RED" << std::endl;
				break;
			case 3:
				std::cout << "YELLOW" << std::endl;
				break;
			}
		}

		colours.push_back(index);
    }

	return colours;
};


std::vector<unsigned int> ShapesDetector::get_circle_colours(const std::vector<cv::Vec3f>& circles, const cv::Mat& image, bool debug) {
	std::vector<unsigned int> colours;
	for (size_t i = 0; i < circles.size(); i++)
    {
		cv::Mat roi = image(cv::Range(circles[i][1]-circles[i][2], circles[i][1]+circles[i][2]+1), cv::Range(circles[i][0]-circles[i][2], circles[i][0]+circles[i][2]+1));
		cv::Mat1b mask(roi.rows, roi.cols);
		cv::Scalar mean = cv::mean(roi, mask);

		if(debug) std::cout << "Scalar: " << mean << std::endl; //BGR - Format !!

		//values for the colours may change when using the robotCam
		cv::Scalar red (61,57,150,0);//			(0,0,255,0);
		cv::Scalar green (40,115,34);//		(0, 255,0,0);
		cv::Scalar blue (129,84,48);//		(255,0,0,0);
		cv::Scalar yellow (84,155,143);//		(0,255,255,0);

		double norm_blue = cv::norm(mean-blue);
		double norm_green = cv::norm(mean-green);
		double norm_red = cv::norm(mean-red);
		double norm_yellow = cv::norm(mean-yellow);

		double norms[] = {norm_blue, norm_green, norm_red, norm_yellow};
		double min = 300;
		int index = 0;
		for(unsigned int i_n = 0; i_n < 4; i_n++) {
			if(norms[i_n] < min) {
				min = norms[i_n];
				index = i_n;
			}
		}
		if(debug) {
			switch(index)
			{
			case 0:
				std::cout << "BLUE" << std::endl;
				break;
			case 1:
				std::cout << "GREEN" << std::endl;
				break;
			case 2:
				std::cout << "RED" << std::endl;
				break;
			case 3:
				std::cout << "YELLOW" << std::endl;
				break;
			}
		}

		colours.push_back(index);
    }

	return colours;
};


cv::Point ShapesDetector::get_middle(const std::vector<cv::Point> position) {

	cv::Point middle(0,0);
	for(unsigned int i = 0; i < position.size(); i++) {
		middle = middle + position[i];
	}
		middle.x = middle.x * (1 / position.size());
		middle.y = middle.y * (1 / position.size());

	return middle;
};


void ShapesDetector::get_all_shapes(std::vector<cv::Point>& positions2D, std::vector<cv::Point3d>& positions3D, std::vector<unsigned int>& colours, std::vector<unsigned int>& shape_types, bool debug) {
	cv::Mat img_bgr;
	img_subs->getImageData(img_bgr);

	cv::Mat img_gray = to_gray(img_bgr, debug);

	std::vector<std::vector<cv::Point> > shape_positions;
	std::vector<cv::Vec3f> circles;

	shape_types = get_shapes(img_gray, shape_positions, circles, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, debug);

	int size = shape_positions.size() + circles.size();
	colours.reserve(size);
	positions2D.reserve(size);

	if (shape_positions.size() != 0) {
		std::vector<unsigned int> shape_colours = get_colours(shape_positions, img_bgr, debug);
		for (unsigned int i_shape = 0; i_shape < shape_positions.size(); i_shape++) {
			positions2D.push_back(get_middle(shape_positions[i_shape]));
			colours.push_back(shape_colours[i_shape]);
		}
	}
	if (circles.size() != 0) {
		std::vector<unsigned int> circle_colours = get_circle_colours(circles, img_bgr, debug);
		for (unsigned int i = 0; i < circles.size(); i++) {
			positions2D.push_back(cv::Point(circles[i][0], circles[i][1]));
			shape_types.push_back(1);
			colours.push_back(circle_colours[i]);
		}
	}

	// 2d to 3d position
	for(unsigned int  i = 0; i < positions2D.size(); i++) {
		cv::Point3d cur_position3D;
		get_screen_to_3Dpoints(positions2D[i], cur_position3D, 0.0);
		positions3D.push_back(cur_position3D);
	}

};


void ShapesDetector::init_calibration_data(ros::NodeHandle *n) {
	device_msgs::CameraCalibData data;
	ros::ServiceClient client = n->serviceClient<camera_srvs::CameraInfo>("/camera/camerainfo");
	camera_srvs::CameraInfo srv;
	if (client.call(srv)) {
		mWidth = srv.response.response.width;
		mHeight = srv.response.response.height;

		std::cout << "mWidth  mHeight from service " << mWidth << " " << mHeight << std::endl;

		intrinsicsMat = cv::Mat(3, 3, cv::DataType<double>::type);
		std::cout << "intrinsicsMat from service " << std::endl;
		double *input = (double *) (intrinsicsMat.data);
		for (int i = 0; i < 9; ++i) {
			input[i] = srv.response.response.K[i];
		}
		std::cout << "intrinsicsMat " << intrinsicsMat << std::endl;

		extrinsicsMat = cv::Mat(4, 4, cv::DataType<double>::type);
		std::cout << "extrinsicsMat from service " << std::endl;
		double *input1 = (double *) (extrinsicsMat.data);
		for (int i = 0; i < 16; ++i) {
			input1[i] = srv.response.response.E[i];
		}
		std::cout << "extrinsicsMat " << extrinsicsMat << std::endl;

		projectionMat = cv::Mat(3, 4, cv::DataType<double>::type);
		std::cout << "projectionMat from service " << std::endl;
		double *input2 = (double *) (projectionMat.data);
		for (int i = 0; i < 12; ++i) {
			input2[i] = srv.response.response.P[i];

		}
		std::cout << "projectionMat " << projectionMat << std::endl;

	}
	else {
		ROS_ERROR("Failed to call service CAMERA_0_INFO");
		throw;
	}

};


void ShapesDetector::get_screen_to_3Dpoints(const cv::Point2i &screenPoint, cv::Point3d &point3D, double z) {
	static double fx = intrinsicsMat.at<double>(0, 0);
	static double fy = intrinsicsMat.at<double>(1, 1);
	static double cx = intrinsicsMat.at<double>(0, 2);
	static double cy = intrinsicsMat.at<double>(1, 2);

	static double r11 = extrinsicsMat.at<double>(0, 0);
	static double r12 = extrinsicsMat.at<double>(0, 1);
	static double r13 = extrinsicsMat.at<double>(0, 2);
	static double r21 = extrinsicsMat.at<double>(1, 0);
	static double r22 = extrinsicsMat.at<double>(1, 1);
	static double r23 = extrinsicsMat.at<double>(1, 2);
	static double r31 = extrinsicsMat.at<double>(2, 0);
	static double r32 = extrinsicsMat.at<double>(2, 1);
	static double r33 = extrinsicsMat.at<double>(2, 2);

	static double px = extrinsicsMat.at<double>(0, 3);
	static double py = extrinsicsMat.at<double>(1, 3);
	static double pz = extrinsicsMat.at<double>(2, 3);

	static double Xs;
	static double Ys;

	//z=0.0;
	//std::cout << "z " << z << std::endl;
	Xs = screenPoint.x;
	Ys = screenPoint.y;
	point3D.x =
			((cx * r32 - Xs * r32 + fx * r12) * (cy * pz - Ys * pz + cy * r33 + fy * py + fy * r23 - Ys * z * r33)) /
			(Xs * fy * r21 * r32 - Xs * fy * r22 * r31 - Ys * fx * r11 * r32 + Ys * fx * r12 * r31 +
			 cy * fx * r11 * r32 - cy * fx * r12 * r31 - cx * fy * r21 * r32 + cx * fy * r22 * r31 +
			 fx * fy * r11 * r22 - fx * fy * r12 * r21) -
			((cy * r32 - Ys * r32 + fy * r22) * (cx * pz - Xs * pz + cx * r33 + fx * px + fx * r13 - Xs * z * r33)) /
			(Xs * fy * r21 * r32 - Xs * fy * r22 * r31 - Ys * fx * r11 * r32 + Ys * fx * r12 * r31 +
			 cy * fx * r11 * r32 - cy * fx * r12 * r31 - cx * fy * r21 * r32 + cx * fy * r22 * r31 +
			 fx * fy * r11 * r22 - fx * fy * r12 * r21);
	point3D.y =
			((cy * r31 - Ys * r31 + fy * r21) * (cx * pz - Xs * pz + cx * r33 + fx * px + fx * r13 - Xs * z * r33)) /
			(Xs * fy * r21 * r32 - Xs * fy * r22 * r31 - Ys * fx * r11 * r32 + Ys * fx * r12 * r31 +
			 cy * fx * r11 * r32 - cy * fx * r12 * r31 - cx * fy * r21 * r32 + cx * fy * r22 * r31 +
			 fx * fy * r11 * r22 - fx * fy * r12 * r21) -
			((cx * r31 - Xs * r31 + fx * r11) * (cy * pz - Ys * pz + cy * r33 + fy * py + fy * r23 - Ys * z * r33)) /
			(Xs * fy * r21 * r32 - Xs * fy * r22 * r31 - Ys * fx * r11 * r32 + Ys * fx * r12 * r31 +
			 cy * fx * r11 * r32 - cy * fx * r12 * r31 - cx * fy * r21 * r32 + cx * fy * r22 * r31 +
			 fx * fy * r11 * r22 - fx * fy * r12 * r21);
	point3D.z = z;
};


ShapesDetector::~ShapesDetector() {
//	intrinsicsMat.deallocate();
//	extrinsicsMat.deallocate();
//	projectionMat.deallocate();
};