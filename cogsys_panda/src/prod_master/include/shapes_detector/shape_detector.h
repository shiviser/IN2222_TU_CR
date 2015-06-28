#pragma once

/*
 * code inspired from demo_cogsys_cpp/src/main.cpp
 */


#include <ros/ros.h>
#include <shape_detect_srvs/shape_vec.h>
#include <shape_detect_srvs/shape.h>

typedef shape_detect_srvs::shape_vec DShapes;

class ShapesDetector {
private:
    ros::ServiceClient shape_detector_client;
    DShapes shape_detect_srv;

public:
    // constructor
    ShapesDetector(ros::NodeHandle *n);

    //
    DShapes getCenterShape();

    //
    DShapes getAllShapes(int thres);

    // destructor
    ~ShapesDetector();

};
