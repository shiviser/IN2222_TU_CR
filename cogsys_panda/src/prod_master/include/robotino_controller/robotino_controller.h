#include <ros/ros.h>
#include <robotino_controller/GetObjects.h>
#include <robotino_controller/Rotate.h>
#include <robotino_controller/MoveToCam.h>
#include <robotino_controller/MoveToGripper.h>
#include <robotino_controller/shape.h>


class RobotinoController {
private:

	//for fetching:
   	 ros::ServiceClient fetch_client;
   	 robotino_controller::GetObjects fetch_srv;

	//for rotating:
	ros::ServiceClient rotate_client;
	robotino_controller::Rotate rotate_srv;

	//for moving to cambot:
	ros::ServiceClient moveToCam_client;
	robotino_controller::MoveToCam moveToCam_srv;

	//for moving to gripperbot:
	ros::ServiceClient moveToGripper_client;
	robotino_controller::MoveToGripper moveToGripper_srv;

	
public:
	RobotinoController(ros::NodeHandle *n);

	bool fetch(std::vector<robotino_controller::shape> objects); //TODO: shape/colour as number or string?

	int rotate();

	bool move_to_cambot();

	bool move_to_gripperbot();

	
};

