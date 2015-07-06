#include "robotino_controller.h"

RobotinoController::RobotinoController(ros::NodeHandle *n) {
	// bot clients
	fetch_client = n->serviceClient<robotino_controller::GetObjects>("/robotino/getObjects");

	rotate_client = n->serviceClient<robotino_controller::Rotate>("/robotino/rotate");

	moveToCam_client = n->serviceClient<robotino_controller::MoveToCam>("/robotino/moveToCamera");

	moveToGripper_client = n->serviceClient<robotino_controller::MoveToGripper>("/robotino/moveToGripper");
}

bool RobotinoController::fetch(std::vector<robotino_controller::shape> objects) {
    fetch_srv.request.shoppingList = objects;

    if (fetch_client.call(fetch_srv)) {
        if (fetch_srv.response.success) {
            std::cout << "Fetching successful" << std::endl;
            return true;
        }
        else {
            std::cout << "Fetching objects failed" << std::endl;
            return false;
        }
    }
    else {
        ROS_ERROR("Failed to call service /robotino/getObjects");
        throw;
    }
};

int RobotinoController::rotate() {
//	rotate_srv.request.slots = 0;

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
        ROS_ERROR("Failed to call service /robotino/rotate");
        throw;
    }
};


bool RobotinoController::move_to_cambot() {
//	moveToCam_srv.request.dummy = 0;

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
        	ROS_ERROR("Failed to call service /robotino/moveToCamera");
        	throw;
    	}
};

bool RobotinoController::move_to_gripperbot() {
//	moveToGripper_srv.request.dummy = 0;

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
        	ROS_ERROR("Failed to call service /robotino/moveToGripper");
        	throw;
    	}
};