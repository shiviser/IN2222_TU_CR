#pragma once

#include <ros/ros.h>
#include <shape_detect_srvs/shape_vec.h>
#include <shape_detect_srvs/shape.h>


struct Shape {
    int xPos;
    int yPos;
    int colour;
    int shape;
};


class ShapesDetector {
private:
    ros::ServiceClient shape_detector_client;
    shape_detect_srvs::shape_vec shape_detect_srv;

public:
    // constructor
    ShapesDetector(ros::NodeHandle *n);

    //
    bool get_center_shape(Shape& found_shape);

    //
    std::vector<Shape> get_shapes(int thres=0);

    // destructor
    ~ShapesDetector();

};
