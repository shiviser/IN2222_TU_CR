#include "shapes_detector.h"

ShapesDetector::ShapesDetector(ros::NodeHandle *n) {

	img_subs = new comminterface::ImageSubscriber("/camera/image");
	img_subs->start();
};

string ShapesDetector::intToString(int number) {
	
	std::stringstream ss;
	ss << number;
	return ss.str();
}


void ShapesDetector::morphOps(Mat &thresh) {

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(6,6));

	erode(thresh,thresh,erodeElement);
	//erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	//dilate(thresh,thresh,dilateElement);
}

bool ShapesDetector::get_img(bool debug) {
	if(debug) {
		string filename = "/home/cogrob/catkin_cogsys-ros/src/shape_detect/test_pictures/test_img3.jpg";
    		img = cv::imread( filename, CV_LOAD_IMAGE_COLOR);
	
		if(!img.data ) { // Check for invalid input
        		cout <<  "Could not open or find the image" << std::endl ;
       	 		return false;
    		}
	} else {
		img_subs->getImageData(img);
	}

	return true;
}

bool ShapesDetector::find_shapes(int thres, bool debug, vector<HSV_Param> HSV_Types) {
	
	cv::Mat HSV;

	get_img(debug);
	
	if(img.data) {
		
		Shapes.clear();
		img_info = img.clone();

		cvtColor(img,HSV,COLOR_BGR2HSV);


		for(int i=0; i < NR_OF_COLOURS ; i++) {
			scan_colour( HSV,HSV_Types[i], thres);
		}

		display_shapes(thres);
	}
}

bool ShapesDetector::scan_colour(Mat HSV_img,HSV_Param Filt_Type, int thres) {
	cv::Mat threshold;
	int xPos,yPos;
	int x_min_box = thres;
	int x_max_box = FRAME_WIDTH-thres;
	int y_min_box = thres;
	int y_max_box = FRAME_HEIGHT-thres;
	Shape tempShape;
	vector<Shape> tempShapes;

	//initiallize temp shape with color
	tempShape.setColour(Filt_Type.getColour());
	
	//initiallize box constraints
	if(thres < 0) {
		x_min_box = CENTER_BOX_X_MIN;
		x_max_box = CENTER_BOX_X_MAX;
		y_min_box = CENTER_BOX_Y_MIN;
		y_max_box = CENTER_BOX_Y_MAX;	
	}

	cv::inRange(HSV_img,Filt_Type.getHSVmin(),Filt_Type.getHSVmax(),threshold);

	if(Filt_Type.getColour() == red) {
		cv::Mat temp;
		cv::Scalar HSV_min = Filt_Type.getHSVmin();
		cv::Scalar HSV_max = Filt_Type.getHSVmax();

		HSV_min.val[0] = RED_H_MIN_2;
		HSV_max.val[0] = RED_H_MAX_2;
		
		cv::inRange(HSV_img,HSV_min,HSV_max,temp);
		bitwise_or(threshold, temp, threshold);
		
		//cv::imshow("red threshold",threshold);
		//waitKey(50);
	}

	morphOps(threshold);

	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	cv::Mat approx;
	cv::Mat edges_map;
	vector<Vec4i> hierarchy;
	vector<Vec3f> circles;
	//find contours of filtered image using openCV findContours function
	cv::findContours(threshold,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//cv::Canny(threshold, edges_map, 5, 250, 7, true);
	//cv::HoughCircles(edges_map, circles, CV_HOUGH_GRADIENT, 1, 100, 180, 30 ,20, 210);
	
	//use moments method to find the position
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){
					
					approxPolyDP(Mat(contours[index]), approx, arcLength(Mat(contours[index]), true)*0.04, true);
					drawContours(img_info, approx, -1, Scalar(0,255,0), 5, 4);

					xPos = moment.m10/area;
					yPos = moment.m01/area;
					
					tempShape.setXPos(xPos);
					tempShape.setYPos(yPos);

					int corners = approx.size().height;

					if(corners == 3) {
						tempShape.setShape(triangle_sh);
					} else if(corners == 4) {
						tempShape.setShape(square_sh);
					} else if(corners > 5) {
						tempShape.setShape(circle_sh);
					}
			
					if(corners >= 3) {
						tempShapes.push_back(tempShape);
					}

				}


			}

		}
	}
	
	/*//ROS_INFO("Circles Detected: %d",circles.size());
	
	for( size_t i = 0; i < circles.size(); i++ ) {
		tempShape.setXPos(cvRound(circles[i][0]));
		tempShape.setYPos(cvRound(circles[i][1]));

		tempShape.setRadius(cvRound(circles[i][2]));
	
		tempShape.setShape(circle_sh);

		tempShapes.push_back(tempShape);
	}*/
	
	for( size_t i = 0; i < tempShapes.size(); i++ ) {
		int xPos = tempShapes[i].getXPos();
		int yPos = tempShapes[i].getYPos();

		if((xPos < x_max_box && xPos > x_min_box) && (yPos < y_max_box && yPos > y_min_box)) {
			Shapes.push_back(tempShapes[i]);		
		}
	}
	
	return true;
}

void ShapesDetector::display_shapes(int thres) {
	string col, shape;
	int x_min_box = thres;
	int x_max_box = FRAME_WIDTH-thres;
	int y_min_box = thres;
	int y_max_box = FRAME_HEIGHT-thres;

	if(thres < 0) {
		x_min_box = CENTER_BOX_X_MIN;
		x_max_box = CENTER_BOX_X_MAX;
		y_min_box = CENTER_BOX_Y_MIN;
		y_max_box = CENTER_BOX_Y_MAX;	
	}


	//draw midpoint of image
	//cv::circle(frame,cv::Point(FRAME_WIDTH/2,FRAME_HEIGHT/2),10,Scalar(0,180,255));

	for(int i=0;i<Shapes.size();i++) {
		cv::circle(img_info,cv::Point(Shapes.at(i).getXPos(),Shapes.at(i).getYPos()),10,Scalar(0,0,255));
		cv::putText(img_info,intToString(Shapes.at(i).getXPos())+ " , " + intToString(Shapes.at(i).getYPos()),cv::Point(Shapes.at(i).getXPos(),Shapes.at(i).getYPos()+20),1,1,Scalar(0,0,0));
		
		switch(Shapes[i].getColour()) {
			case red: col="Red"; break;
			case blue: col="Blue"; break;
			case green: col="Green"; break;
			case yellow: col="Yellow"; break;
			default: break;
		}

		switch(Shapes[i].getShape()) {
			case square_sh: shape="Square"; break;
			case circle_sh:
				shape="Circle";
				//circle( img_info, cv::Point(Shapes.at(i).getXPos(),Shapes.at(i).getYPos()), Shapes.at(i).getRadius(), Scalar(255,255,255), 3, 5, 0 );
				break;
			case triangle_sh: shape="Triangle"; break;
			default: break;
		}

		cv::putText(img_info,col+shape,Point(Shapes.at(i).getXPos(),Shapes.at(i).getYPos()-30),1,2,Scalar(0,0,0));
	}
	
	//draw theshold lines
	line( img_info, Point(x_min_box,y_min_box), Point(x_min_box,y_max_box), cv::Scalar( 0, 0, 0 ), 2, 8 );
	line( img_info, Point(x_min_box,y_max_box), Point(x_max_box,y_max_box), cv::Scalar( 0, 0, 0 ), 2, 8 );
	line( img_info, Point(x_max_box,y_max_box), Point(x_max_box,y_min_box), cv::Scalar( 0, 0, 0 ), 2, 8 );
	line( img_info, Point(x_max_box,y_min_box), Point(x_min_box,y_min_box), cv::Scalar( 0, 0, 0 ), 2, 8 );
	
	cv::imshow("Detected Shapes",img_info);
	waitKey(50);
}

ShapesDetector::~ShapesDetector() {

};
