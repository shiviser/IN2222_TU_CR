// all other includes go inside this header
#include "main.h"


void compute_transformation(const std_msgs::String::ConstPtr& msg) {
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
            transformation_srv.request.x = x;
            transformation_srv.request.y = y;
            transformation_srv.request.z = z;
            ROS_INFO("sending back response: [%f] [%f] [%f]", (float) x, (float) y, (float) z);

            if (transformation_client.call(transformation_srv)) {
                break;
            }
            else {
                ROS_ERROR("Failed to call service /prod_master_srvs/computed_transformation");
            }

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

        transformation_client = n.serviceClient<prod_master_srvs::ComputedTransformation>("/prod_master_srvs/computed_transformation");

        ros::Subscriber sub = n.subscribe("/prod_master/start_trans_listen", 1, compute_transformation);
        ROS_INFO("Waiting to get to compute transformation instruction.");
        ros::spin();

        return 0;
    }

    catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }

    return 1;
}
