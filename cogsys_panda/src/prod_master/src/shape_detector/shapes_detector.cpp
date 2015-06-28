#include "bot_controller.h"


ShapesDetector::ShapesDetector(ros::NodeHandle *n) {
        shape_detector_client = n->serviceClient<shape_detect_srvs::shape_vec>("/shape_detector/shapes");
};


DShapes ShapesDetector::getCenterShape() {
	shape_detect_srv.request.filter = -1;

	if (shape_detector_client.call(cambot_pose_srv)) {
        	return shape_detect_srv.response.shapes;
    	}
   	else {
        	ROS_ERROR("Failed to call service /cambot_control/get_pose");
        	throw;
    	}
}


DShapes ShapesDetector::getAllShapes(int thres) {
	shape_detect_srv.request.filter = thres;

	if (shape_detector_client.call(cambot_pose_srv)) {
        	return shape_detect_srv.response.shapes;
    	}
   	else {
        	ROS_ERROR("Failed to call service /cambot_control/get_pose");
        	throw;
    	}	
}

ShapesDetector::~ShapesDetector() {

};
