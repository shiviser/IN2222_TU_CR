/** BEGIN LICENSE ************************************************************/
//    Copyright 2014 fortiss GmbH
//    All rights reserved.
//
//    Any use, distribution or replication without a written permission
//    is prohibited.
//
//    The name "fortiss GmbH" must not be used to endorse or promote
//    products derived from the source code without prior written permission.
//
//    You agree to indemnify, hold harmless and defend fortiss GmbH from
//    and against any loss, damage, claims or lawsuits, including attorney's
//    fees that arise or result from your use the software.
//
//    THIS SOFTWARE IS PROVIDED "AS IS" AND "WITH ALL FAULTS", WITHOUT ANY
//    TECHNICAL SUPPORT OR ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING,
//    BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
//    FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. ALSO, THERE IS NO
//    WARRANTY OF NON-INFRINGEMENT, TITLE OR QUIET ENJOYMENT. IN NO EVENT
//    SHALL COGNITION FACTORY OR ITS SUPPLIERS BE LIABLE FOR ANY DIRECT,
//    INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
//    STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
//    IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//    POSSIBILITY OF SUCH DAMAGE.
/** END LICENSE **************************************************************/

/*
* Tracking
* started by Julia & Shiv
*/

// this inlcude has to be on the top since it contains tf and tf has to be before QApplication
#include "prod_master.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
//#include <vector>
//#include <camera_srvs/CameraInfo.h>

#include <ros/ros.h>
//#include <ros/package.h>
//#include <ros/init.h>
#include <iostream>
#include <QApplication>
#include <stdexcept>
#include <exception>
#include <signal.h>
//#include <detector/DetectorThread.h>
//#include <comm_lib/ActorsSubscriber.h>
//#include <comm_lib/util/Util.h>
//#include <comm_lib/CommonDefs.h>
//#include <comm_lib/ActorsPublisher.h>
#include <comm_lib/ImageSubscriber.h>
#include <robot_control_srvs/MoveToCS.h>
#include <gripper_control_srvs/OpenGripper.h>
#include <gripper_control_srvs/CloseGripper.h>
//#include <tf/LinearMath/Vector3.h>
//#include <tf/tf.h>
#include "shapes_detector.h"
#include "bot_controller.h"

#include "main.h"



using namespace std;

/*ros::ServiceClient gripperbot_client;
robot_control_srvs::MoveToCS gripperbot_srv;

ros::ServiceClient cambot_client;
robot_control_srvs::MoveToCS cambot_srv;

ros::ServiceClient opengripper_client;
gripper_control_srvs::OpenGripper opengripper_srv;

ros::ServiceClient closegripper_client;
gripper_control_srvs::CloseGripper closegripper_srv; */


void broadcast_loc_wrt_cambot(const cv::Point3d position3D) {

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(position3D.x/1000.0,
                                         position3D.y/1000.0,
                                         position3D.z/1000.0));
        tf::Quaternion q;
        q.setX(0.0);
        q.setY(0.0);
        q.setZ(0.0);
        q.setW(1.0);
        transform.setRotation(q);

        std::cout << "broadcasted" << std::endl;

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "cambot_wrist", "TARGET_BOX_0"));

};


int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, "marker_grabber");

        ros::NodeHandle n;

        /*gripperbot_client = n.serviceClient<robot_control_srvs::MoveToCS>("/gripperbot_control/move_to_cs");
        cambot_client = n.serviceClient<robot_control_srvs::MoveToCS>("/cambot_control/move_to_cs");
        opengripper_client = n.serviceClient<gripper_control_srvs::OpenGripper>("/gripper_control/open_gripper");
        closegripper_client = n.serviceClient<gripper_control_srvs::CloseGripper>("/gripper_control/close_gripper"); */

        ProdMaster prod_master(&n);
        BotController bot(&n);
        ShapesDetector shapes_detector(&n);

        tf::TransformListener listener;
//        DetectorThread detector;
//        detector.start();

//        cv::Mat detectImg;
//        cv::Mat colorSegImg;
//        cv::namedWindow("detector", 1);
//        //namedWindow("seg", 1);
//        //namedWindow("cont", 1);
//
        cv::Mat img;
        int getImgFlag = 23;
        cv::namedWindow("video", 1);
        int k;

        comminterface::ImageSubscriber imgSub("/camera/image");
        imgSub.start();

////        detector.getDetectorPtr()->setColourSegDebug(true);
//
//
        while (ros::ok()) {

//            detector.getDetectorPtr()->getOutputImg(detectImg);
//            detector.getDetectorPtr()->getColourSegmentedImg(colorSegImg);

//            imshow("detector",  detectImg);
//            imshow("seg", colorSegImg);

            std::cout << "Main loop" << std::endl;

//            std::cin >> getImgFlag;

            if (getImgFlag == 23) {

                //"UI" maybe too simple ;)
                unsigned int wantedShape = 1;
                unsigned int wantedColour = 2;
                std::cout << "Please enter the number corresponding to the type of shape!" << std::endl <<
                "1 (circle), 3 (triangle), 4 (square)" << std::endl;
                std::cin >> wantedShape;
                std::cout << "Please enter the number corresponding to the colour" << std::endl <<
                "0 (blue), 1 (green), 2 (red), 3 (yellow)" << std::endl;
                std::cin >> wantedColour;

                bool found = false;
                cv::Point position;
                cv::Point3d position3D;

                //move Gripperbot to position that doesn't collide with Cambot
                //get gripperbot pose
//		float x_gp, y_gp, z_gp;
//		bot.getgripperbotpose(x_gp, y_gp, z_yp);
//		//e.g. just move gripperbot back
//		int limit = 0.15; //maybe different value
//		if(x_gp > limit) bot.movegripperbot(0.15, y_gp, z_gp) //that position is hopefully possible ;)
//		bool moving = true;
//		while(moving) {
//			float x_p, y_p, z_p;
//			float eps = 0.02; //to tolerate some error
//			bot.getgripperbotpose(x_p, y_p, z_p);
//			if(((x_gp - eps) < x_p < (x_gp+eps)) && ((y_gp - eps) < y_p < (y_gp+eps)) && ((z_gp - eps) < z_p < (z_gp+eps)))
//				moving = false;
//			//wait for some time?
//		}
                // bot bots to home
                bot.movetohome(false, true);

                //Position 1
                float x1 = 0.23;
                float y1 = -0.1;
                float z1 = 0.43;
                bot.movecambot(x1, y1, z1);
//		bool moving = true;
//		while(moving) {
//			float x_p, y_p, z_p;
//			float eps = 0.02; //to tolerate some error
//			bot.getcambotpose(x_p, y_p, z_p);
//			if(((x1 - eps) < x_p < (x1+eps)) && ((y1 - eps) < y_p < (y1+eps)) && ((z1 - eps) < z_p < (z1+eps))) moving = false;
//			//wait for some time?
//		}

                //get Shapes + Colours
                // TODO
                sleep(1);
                bool img_udpated = false;
                while(!img_udpated)
                    img_udpated = imgSub.getImageData(img);
//                cv::imshow("video", img);
//                cv::waitKey(0);
                std::vector<cv::Point> positions1;
                std::vector<cv::Point3d> positions3D1;
                std::vector<unsigned int> colours1;
                std::vector<unsigned int> shape_types1;
             //   shapes_detector.get_all_shapes(positions1, positions3D1, colours1, shape_types1, false);

                std::cout << shape_types1.size() << std::cout;

                //compare shapes with user input
                for (size_t i = 0; i < positions1.size(); i++) {
                    if (colours1[i] == wantedColour && shape_types1[i] == wantedShape) {
                        position = positions1[i];
                        position3D = positions3D1[i];
                        found = true;
                    }
                }
//                if (found) {
//                    std::cout << "object found in image1 at: " << position << std::endl;
//
//                    //TODO get 3D coordinates + move cambot to secure position + move gripperbot to shape
//                }
                if (!found) {
                    //Position 2
                    float x2 = 0.23;
                    float y2 = 0.09;
                    float z2 = 0.43;
                    bot.movecambot(x2, y2, z2);
//                    moving = true;
//                    while (moving) {
//                        float x_p, y_p, z_p;
//                        float eps = 0.01; //to tolerate some error
//                        bot.getcambotpose(x_p, y_p, z_p);
//                        if (((x2 - eps) < x_p < (x2 + eps)) && ((y2 - eps) < y_p < (y2 + eps)) &&
//                            ((z2 - eps) < z_p < (z2 + eps)))
//                            moving = false;
//                        //wait for some time?
//                    }

                    //get shapes
                    // TODO
                    sleep(1);
                    img_udpated = false;
                    while(!img_udpated)
                        img_udpated = imgSub.getImageData(img);
//                    cv::imshow("video", img);
//                    cv::waitKey(0);
                    std::vector<cv::Point> positions2;
                    std::vector<cv::Point3d> positions3D2;
                    std::vector<unsigned int> colours2;
                    std::vector<unsigned int> shape_types2;
                   // shapes_detector.get_all_shapes(positions2, positions3D2, colours2, shape_types2, false);

                    std::cout << shape_types2.size() << std::cout;

                    //compare shapes with user input
                    for (size_t i = 0; i < positions1.size(); i++) {
                        if (colours2[i] == wantedColour && shape_types2[i] == wantedShape) {
                            position = positions2[i];
                            position3D = positions3D2[i];
                            found = true;
                        }
                    }
                }

                if (found) {
                    std::cout << "object found in image2 at: " << position << "3D: " << position3D << std::endl;


//                    tf::StampedTransform stamped_transform;
                        /*
                      get the pose of the object w.r.t gripperbot_base frame
                    */
//                        listener.waitForTransform("target", "gripperbot_base",
//
//                            ros::Time(0), ros::Duration(1.0));

                    broadcast_loc_wrt_cambot(position3D);

//                    listener.lookupTransform("target", "gripperbot_base",
//                                                 ros::Time(0), stamped_transform);
//                        float x = 4,y = 4,z = 4;
//                    std::cin >> x;
//                    std::cin >> y;
//                    std::cin >> z;
//                        x = stamped_transform.getOrigin().getX();
//                        y = stamped_transform.getOrigin().getY();
//                        z = stamped_transform.getOrigin().getZ();
//                        if(z < 0) z = -z;
//                        if(x < 0) continue;
//                        x -= 0.15;
                        /*
                            move the gripperbot using the obtained object position
                        */
//                        bot.movegripperbot(x, y, z);


                    //TODO get 3D coordinates + move cambot to secure position + move gripperbot to shape

                }
                else {
                    std::cout << "the object couldn't be found " << std::endl;
                }
            }

            k = cv::waitKey(100);

            if ((int) (k & 0xFF) == 27)
                break;

        }

//        detector.stopThread();
//
//        detectImg.release();
//            colorSegImg.release();

//        cv::destroyWindow("detector");
//        cv::destroyWindow("seg");
        ros::spinOnce();

    return 1;

}
catch (std::exception &e) {

    std::cout << e.what() << endl;
}

return 0;
};



