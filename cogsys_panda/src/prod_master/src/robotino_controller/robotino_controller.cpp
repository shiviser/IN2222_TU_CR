#include "robotino_controller.h"

RobotinoController::RobotinoController(ros::NodeHandle *n) {
        // bot clients
//        fetch_client = n->serviceClient<robotino_controller::Fetch>("/robotino_controller_node/fetch");

	rotate_client = n->serviceClient<robotino_controller::Rotate>("/robotino_controller_node/rotate");

	moveToCam_client = n->serviceClient<robotino_controller::MoveToCambot>("/robotino_controller_node/move_to_cambot");

	moveToGripper_client = n->serviceClient<robotino_controller::MoveToGripper>("/robotino_controller_node/move_to_gripper");
}

//bool RobotinoController::fetch(std::vector<std::vector<int>> objects) {
//    fetch_srv.request.objects = objects;
//
//    if (fetch_client.call(fetch_srv)) {
//        if (fetch_srv.response.success) {
//            std::cout << "Fetching successful" << std::endl;
//            return true;
//        }
//        else {
//            std::cout << "Fetching objects failed" << std::endl;
//            return false;
//        }
//    }
//    else {
//        ROS_ERROR("Failed to call service /robotino_control/fetch");
//        throw;
//    }
//};

int RobotinoController::rotate() {
    if (rotate_client.call(rotate_srv)) {
        if (rotate_srv.response.success == 0) {
            std::cout << "Rotation successful" << std::endl;
            return 0;
        }
	else if(rotate_srv.response.success == 1) {
		std::cout << "Already rotated by 360 degrees" << std::endl;
		return 1;
	}
        else {
            std::cout << "Rotating failed" << std::endl;
            return -1;
        }
    }
    else {
        ROS_ERROR("Failed to call service /robotino_control/rotate");
        throw;
    }
};


bool RobotinoController::move_to_cambot() {
	if (moveToCam_client.call(moveToCam_srv)) {
		if (moveToCam_srv.response.success) {
		    std::cout << "Moved to Cambot" << std::endl;
		    return true;
		}
		else {
		    std::cout << "Moving to Cambot failed" << std::endl;
		    return false;
		}
    	}
   	else {
        	ROS_ERROR("Failed to call service /robotino_control/move_to_cambot");
        	throw;
    	}
};

bool RobotinoController::move_to_gripperbot() {
	if (moveToGripper_client.call(moveToGripper_srv)) {
		if (moveToGripper_srv.response.success) {
		    std::cout << "Moved to Gripperbot" << std::endl;
		    return true;
		}
		else {
		    std::cout << "Moving to Gripperbot failed" << std::endl;
		    return false;
		}
    	}
   	else {
        	ROS_ERROR("Failed to call service /robotino_control/move_to_gripperbot");
        	throw;
    	}
};