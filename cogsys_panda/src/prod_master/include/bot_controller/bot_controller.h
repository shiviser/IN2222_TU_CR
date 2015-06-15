#pragma once

/*
 * code inspired from demo_cogsys_cpp/src/main.cpp
 */


#include <ros/ros.h>
#include <robot_control_srvs/MoveToCS.h>
#include <robot_control_srvs/GetPose.h>
#include <gripper_control_srvs/OpenGripper.h>
#include <gripper_control_srvs/CloseGripper.h>
#include <speech_recognition_srvs/TurnOff.h>
#include <speech_recognition_srvs/TurnOn.h>

class BotController {
private:
    ros::ServiceClient gripperbot_client;
    robot_control_srvs::MoveToCS gripperbot_srv;

    ros::ServiceClient opengripper_client;
    gripper_control_srvs::OpenGripper opengripper_srv;
    ros::ServiceClient closegripper_client;
    gripper_control_srvs::CloseGripper closegripper_srv;

    ros::ServiceClient cambot_client;
    robot_control_srvs::MoveToCS cambot_srv;

	ros::ServiceClient cambot_pose_client;
	robot_control_srvs::GetPose cambot_pose_srv;

	ros::ServiceClient gripperbot_pose_client;
	robot_control_srvs::GetPose gripperbot_pose_srv;

    ros::ServiceClient mic_on_client;
    speech_recognition_srvs::TurnOn mic_on_srv;
    ros::ServiceClient mic_off_client;
    speech_recognition_srvs::TurnOff mic_off_srv;

public:
    // constructor
    BotController(ros::NodeHandle *n);

	//
	bool getcambotpose(float& x, float& y, float& z, std::string effector="camera");

	//
	bool getgripperbotpose(float& x, float& y, float& z, std::string effector="gripper");

    //
    bool movecambot(float x, float y, float z, std::string effector="camera");

    //
    bool movegripperbot(float x, float y, float z, std::string effector="gripper");

    //
    std::vector<bool> movetohome(bool cambot, bool gripperbot);

    //
    bool opengripper();

    //
    bool closegripper();

    //
    void turn_mic_off();

    //
    void turn_mic_on();

    // destructor
    ~BotController();

};
