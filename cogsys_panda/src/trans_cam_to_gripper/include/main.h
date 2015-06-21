#pragma once

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <trans_cam_to_gripper_srvs/ComputeTransformation.h>


bool compute_transformation(trans_cam_to_gripper_srvs::ComputeTransformation::Request & req,
                            trans_cam_to_gripper_srvs::ComputeTransformation::Response & res);


int main(int argc, char **argv);