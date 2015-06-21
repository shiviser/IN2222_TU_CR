// all other includes go inside this header
#include "main.h"


bool compute_transformation(trans_cam_to_gripper_srvs::ComputeTransformation::Request & req,
                            trans_cam_to_gripper_srvs::ComputeTransformation::Response & res) {

    tf::TransformListener listener;

    while (ros::ok()) {

        tf::StampedTransform stamped_transform;
        try {
            /*
          get the pose of the object w.r.t gripperbot_base frame
        */
            listener.waitForTransform("TARGET_BOX_0", "gripperbot_base",
                                      ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("TARGET_BOX_0", "gripperbot_base",
                                     ros::Time(0), stamped_transform);
            float x, y, z;
            x = stamped_transform.getOrigin().getX();
            y = stamped_transform.getOrigin().getY();
            z = stamped_transform.getOrigin().getZ();

            // if(z < 0) z = -z;
            if (x < 0) {
                std::cout << "x: " << x << " is negative so skipped." << std::endl;
                continue;
            }

            // taking care of gripper dimension
            // x -= 0.15;

            // respond
            res.x = 1.0f;
            res.y = 2.0f;
            res.z = 3.0f;
            ROS_INFO("sending back response: [%f] [%f] [%f]", (float) res.x, (float) res.y, (float) res.z);
            return true;

        }
        catch (tf::TransformException ex) {
            std::cout << "No brodcast received yet .. " << std::endl;
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
    }

}


int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, "trans_cam_to_gripper");
        ros::NodeHandle n;

        ros::ServiceServer service = n.advertiseService("/trans_cam_to_gripper/compute_transformation", compute_transformation);
        ROS_INFO("Ready to to compute transformation.");
        ros::spin();

        return 0;
    }

    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    return 1;
}
