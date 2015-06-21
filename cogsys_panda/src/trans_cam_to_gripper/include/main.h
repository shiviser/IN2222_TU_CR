#pragma once

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <prod_master_srvs/ComputedTransformation.h>


ros::ServiceClient transformation_client;
prod_master_srvs::ComputedTransformation transformation_srv;


void compute_transformation(const std_msgs::String::ConstPtr& msg);


int main(int argc, char **argv);