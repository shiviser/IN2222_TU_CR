#include "shapes_detector.h"


ShapesDetector::ShapesDetector(ros::NodeHandle *n) {
        shape_detector_client = n->serviceClient<shape_detect_srvs::shape_vec>("/shape_detector/shapes");
};


bool ShapesDetector::get_center_shape(Shape& found_shape) {
	std::vector<Shape> shapes = get_shapes(-1);

	if (shapes.empty()) return false;

	found_shape = shapes[0];
	return true;
//	shape_detect_srv.request.filter = -1;
//
//	if (shape_detector_client.call(shape_detect_srv)) {
//		// TODO: take 1st element of array
//		std::vector<Shape> shapes;
//
//		for (auto it = shape_detect_srv.response.shapes.begin(); it != shape_detect_srv.response.shapes.end(); ++it) {
//			Shape cur_shape;
//			cur_shape.xPos = it->xPos;
//			cur_shape.yPos = it->yPos;
//			cur_shape.colour = it->colour;
//			cur_shape.shape = it->shape;
//
//			shapes.push_back(cur_shape);
//		}
//
//		return shapes;
//	}
//	else {
//		ROS_ERROR("Failed to call service /shape_detector/shapes");
//		throw;
//	}
}


std::vector<Shape> ShapesDetector::get_shapes(int thres) {
	shape_detect_srv.request.filter = thres;

	if (shape_detector_client.call(shape_detect_srv)) {
		std::vector<Shape> shapes;

		for (auto it = shape_detect_srv.response.shapes.begin(); it != shape_detect_srv.response.shapes.end(); ++it) {
			Shape cur_shape;
			cur_shape.xPos = it->xPos;
			cur_shape.yPos = it->yPos;
			cur_shape.colour = it->colour;
			cur_shape.shape = it->shape;

			shapes.push_back(cur_shape);
		}

		return shapes;
    	}
   	else {
        	ROS_ERROR("Failed to call service /shape_detector/shapes");
        	throw;
    	}	
}

ShapesDetector::~ShapesDetector() {

};
