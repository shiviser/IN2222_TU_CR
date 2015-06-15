#include "bot_controller.h"


BotController::BotController(ros::NodeHandle *n) {
        // bot clients
        gripperbot_client = n->serviceClient<robot_control_srvs::MoveToCS>("/gripperbot_control/move_to_cs");
        cambot_client = n->serviceClient<robot_control_srvs::MoveToCS>("/cambot_control/move_to_cs");

	//bot pose clients
	cambot_pose_client = n->serviceClient<robot_control_srvs::GetPose>("/cambot_control/get_pose");
	gripperbot_pose_client = n->serviceClient<robot_control_srvs::GetPose>("/gripperbott_control/get_pose");

        // gripper controlling client
        opengripper_client = n->serviceClient<gripper_control_srvs::OpenGripper>("/gripper_control/open_gripper");
        closegripper_client = n->serviceClient<gripper_control_srvs::CloseGripper>("/gripper_control/close_gripper");

        // microphone controller
        mic_on_client = n->serviceClient<speech_recognition_srvs::TurnOn>("/speech_recognition/turn_on");
        mic_off_client = n->serviceClient<speech_recognition_srvs::TurnOff>("/speech_recognition/turn_off");
};

bool BotController::getcambotpose(float& x, float& y, float& z, std::string effector) {
	cambot_pose_srv.request.effector = effector;
	if (cambot_pose_client.call(cambot_pose_srv)) {
        if (cambot_pose_srv.response.success) {
            std::cout << "Approach successful" << std::endl;
		x = cambot_pose_srv.response.x;
		y = cambot_pose_srv.response.x;
		z = cambot_pose_srv.response.x;
            return true;
        }
        else {
            std::cout << "Approach failed to get cambot pose" << std::endl;
            return false;
        }
    }
    else {
        ROS_ERROR("Failed to call service /cambot_control/get_pose");
        throw;
    }
};

bool BotController::getgripperbotpose(float& x, float& y, float& z, std::string effector) {
	gripperbot_pose_srv.request.effector = effector;
	if (gripperbot_pose_client.call(cambot_pose_srv)) {
        if (gripperbot_pose_srv.response.success) {
            std::cout << "Approach successful" << std::endl;
		x = gripperbot_pose_srv.response.x;
		y = gripperbot_pose_srv.response.x;
		z = gripperbot_pose_srv.response.x;
            return true;
        }
        else {
            std::cout << "Approach failed to get gripperbot pose" << std::endl;
            return false;
        }
    }
    else {
        ROS_ERROR("Failed to call service /gripperbot_control/get_pose");
        throw;
    }
};

bool BotController::movecambot(float x, float y, float z, std::string effector) {
    cambot_srv.request.x = x;
    cambot_srv.request.y = y;
    cambot_srv.request.z = z;
    cambot_srv.request.effector = effector;

    if (cambot_client.call(cambot_srv)) {
        if (cambot_srv.response.success) {
            std::cout << "Approach successful" << std::endl;
            return true;
        }
        else {
            std::cout << "Approach failed to " << x << " " << y << " " << z << std::endl;
            return false;
        }
    }
    else {
        ROS_ERROR("Failed to call service /cambot_control/move_to_cs");
        throw;
    }
};


bool BotController::movegripperbot(float x, float y, float z, std::string effector) {
    gripperbot_srv.request.x = x;
    gripperbot_srv.request.y = y;
    gripperbot_srv.request.z = z;
    gripperbot_srv.request.effector = effector;

    if (gripperbot_client.call(gripperbot_srv)) {
        if (gripperbot_srv.response.success) {
            std::cout << "Approach successful" << std::endl;
            return true;
        }
        else {
            std::cout << "Approach failed to " << x << " " << y << " " << z << std::endl;
            return false;
        }
    }
    else {
        ROS_ERROR("Failed to call service /gripperbot_control/move_to_cs");
        throw;
    }
};


std::vector<bool> BotController::movetohome(bool cambot, bool gripperbot) {
    // home position
    static float home_x = 0.25;
    static float home_y = 0.00;
    static float home_z = 0.25;

    std::vector<bool> success;

    if (cambot) {
        std::cout << "Moving Camera Robot to Home" << std::endl;
        bool success_cambot = movecambot(home_x, home_y, home_z);
        success.push_back(success_cambot);
    }

    if (gripperbot) {
        std::cout << "Moving Gripper Robot to Home" << std::endl;
        bool success_gripperbot = movegripperbot(home_x, home_y, home_z);
        success.push_back(success_gripperbot);
    }

    return success;
};


bool BotController::opengripper() {
    std::cout << "Opening Gripper" << std::endl;
    if (opengripper_client.call(opengripper_srv)) {
        if (opengripper_srv.response.success) {
            std::cout << "OpenGripper successful" << std::endl;
            return true;
        }
        else {
            std::cout << "OpenGripper failed" << std::endl;
            return false;
        }
    }
    else {
        ROS_ERROR("Failed to call service /gripper_control/open_gripper");
        throw;
    }
};


void BotController::turn_mic_off() {
    mic_off_client.call(mic_off_srv);
};


void BotController::turn_mic_on() {
    mic_on_client.call(mic_on_srv);
};


bool BotController::closegripper() {
    std::cout << "Closing Gripper" << std::endl;
    if (closegripper_client.call(closegripper_srv)) {
        if (closegripper_srv.response.success) {
            std::cout << "CloseGripper successful" << std::endl;
            return true;
        }
        else {
            std::cout << "CloseGripper failed" << std::endl;
            return false;
        }
    }
    else {
        ROS_ERROR("Failed to call service /gripper_control/close_gripper");
        throw;
    }
};


BotController::~BotController() {
    // gracefully shutdown all clients
    gripperbot_client.shutdown();
    cambot_client.shutdown();
    opengripper_client.shutdown();
    closegripper_client.shutdown();
    mic_on_client.shutdown();
    mic_off_client.shutdown();
};
