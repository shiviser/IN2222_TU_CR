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
* 
* started by Suraj Nair
*/
#include <tf/transform_listener.h>
#include <QApplication>
#include <stdexcept>
#include <exception>
#include <signal.h>
#include "main.h"
#include <comm_lib/ActorsSubscriber.h>
#include <comm_lib/util/Util.h>
#include <robot_control_srvs/MoveToCS.h>
#include <gripper_control_srvs/OpenGripper.h>
#include <gripper_control_srvs/CloseGripper.h>
#include <opencv2/opencv.hpp>
#include <speech_recognition_srvs/TurnOff.h>
#include <speech_recognition_srvs/TurnOn.h>
#include <speech_recognition_msgs/RecognizedSpeech.h>

using namespace std;

ros::ServiceClient gripperbot_client;
robot_control_srvs::MoveToCS gripperbot_srv;

ros::ServiceClient cambot_client;
robot_control_srvs::MoveToCS cambot_srv;

ros::ServiceClient opengripper_client;
gripper_control_srvs::OpenGripper opengripper_srv;

ros::ServiceClient closegripper_client;
gripper_control_srvs::CloseGripper closegripper_srv;

comminterface::ActorsSubscriber* mSub;

ros::Subscriber sub_speech;

ros::ServiceClient mic_on_client;
speech_recognition_srvs::TurnOn mic_on_srv;

ros::ServiceClient mic_off_client;
speech_recognition_srvs::TurnOff mic_off_srv;

void  opengripper()
{
    std::cout << "Opening Gripper" << std::endl;
    if (opengripper_client.call(opengripper_srv))
    {
        if(opengripper_srv.response.success)
            std::cout << "OpenGripper successfull " <<std::endl;
        else
            std::cout << "OpenGripper failed" << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service /gripper_control/open_gripper");
        throw;
    }
}

void closegripper()
{
    std::cout << "Closing Gripper" << std::endl;
    if (closegripper_client.call(closegripper_srv))
    {
        if(closegripper_srv.response.success)
            std::cout << "CloseGripper successfull " <<std::endl;
        else
            std::cout << "CloseGripper failed" << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service /gripper_control/close_gripper");
        throw;
    }
}

void movecambot(float x, float y, float z)
{
    /*

      PUT YOUR CODE HERE
      Implement a function to move the cambot to position x,y,z (similar to movegripperbot)

    */
    cambot_srv.request.x=x;
    cambot_srv.request.y=y;
    cambot_srv.request.z=z;
    cambot_srv.request.effector="camera";

    if (cambot_client.call(cambot_srv))
    {
        if(cambot_srv.response.success)
            std::cout << "Approach successfull " <<std::endl;
        else
            std::cout << "Approach failed to " << x << " " << y << " " << z << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service /cambot_control/move_to_cs");
        throw;
    }
}

void movegripperbot(float x, float y, float z)
{
    gripperbot_srv.request.x=x;
    gripperbot_srv.request.y=y;
    gripperbot_srv.request.z=z;
    gripperbot_srv.request.effector="gripper";

    if (gripperbot_client.call(gripperbot_srv))
    {
        if(gripperbot_srv.response.success)
            std::cout << "Approach successfull " <<std::endl;
        else
            std::cout << "Approach failed to " << x << " " << y << " " << z << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service /gripperbot_control/move_to_cs");
        throw;
    }
}

void home()
{
    static float home_x=0.30;
    static float home_y=0.00;
    static float home_z=0.34;

    gripperbot_srv.request.x=home_x;
    gripperbot_srv.request.y=home_y;
    gripperbot_srv.request.z=home_z;

    std::cout << "Moving Robot to Home" << std::endl;
    movegripperbot(home_x,home_y,home_z);

}

void grasp(float &x, float &y, float &z)
{

    float bottom,top;
    bottom=z;
    top=z+0.05;

    std::cout << "Approaching object top" << std::endl;
    z=top;
    movegripperbot(x,y,z);

    //    std::cout << "Approaching object" << std::endl;
    //    z=bottom;
    //    moverobot(x,y,z);

    //    closegripper();

    //    std::cout << "Approaching top" << std::endl;
    //    z=top;
    //    moverobot(x,y,z);

    //    x = 0;
    //    y = -0.3;
    //    z = 0.3;
    //    moverobot(x,y,z);

    //    opengripper();

    //    home();


}

void turn_mic_off()
{
    mic_off_client.call(mic_off_srv);
}

void analyzeSpeech(const speech_recognition_msgs::RecognizedSpeechConstPtr &speech)
{
    std::string s = speech->speech;

    if (s.find("YELLOW") != string::npos)
    {
        turn_mic_off();
//        grasp("BOX", "YELLOW");
    }
}


void turn_mic_on()
{
    mic_on_client.call(mic_on_srv);
}

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "demo_cogsys_cpp");

        cv::Mat demoImg(480,640, CV_8UC3);
        cv::namedWindow("demo", 1);

        demoImg=cv::Scalar(0,0,0);

        cv::putText(demoImg, "Press ESC to quit app", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
        cv::putText(demoImg, "Press g to grasp GREEN object", cv::Point(30,70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
        cv::putText(demoImg, "Please implement your own activation login for grasping", cv::Point(30,100), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, 	CV_AA);
        cv::putText(demoImg, "using natural language (pressing 'g' is only for testing)", cv::Point(30,130), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
        cv::putText(demoImg, "Press m to activate microphone", cv::Point(30,160), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);

        mSub = new comminterface::ActorsSubscriber("/object_detector/objects_data");
        mSub->start();

        int k;

        ros::NodeHandle n;

        gripperbot_client = n.serviceClient<robot_control_srvs::MoveToCS>("/gripperbot_control/move_to_cs");
        cambot_client = n.serviceClient<robot_control_srvs::MoveToCS>("/cambot_control/move_to_cs");
        opengripper_client = n.serviceClient<gripper_control_srvs::OpenGripper>("/gripper_control/open_gripper");
        closegripper_client = n.serviceClient<gripper_control_srvs::CloseGripper>("/gripper_control/close_gripper");

        sub_speech = n.subscribe("/speech_recognition/output", 1, analyzeSpeech);
        mic_on_client = n.serviceClient<speech_recognition_srvs::TurnOn>("/speech_recognition/turn_on");
        mic_off_client = n.serviceClient<speech_recognition_srvs::TurnOff>("/speech_recognition/turn_off");

        home();
        movecambot(0.3,0,0.4);
        movecambot(0.215,0,0.4);

        tf::TransformListener listener;

        bool graspObject = false;
        while(true)
        {

            if(graspObject)
            {
                std::string type="BOX";
                std::string prop="GREEN";
                int id=0;
                std::stringstream ss_object_name;
                ss_object_name << prop << "_" << type << "_" << id;

                tf::StampedTransform stamped_transform;
                try{
                    /*
                  get the pose of the object w.r.t gripperbot_base frame
                */
                    listener.waitForTransform(ss_object_name.str(), "gripperbot_base",
                                              ros::Time(0), ros::Duration(1.0));
                    listener.lookupTransform(ss_object_name.str(), "gripperbot_base",
                                             ros::Time(0), stamped_transform);
                    float x,y,z;
                    x = stamped_transform.getOrigin().getX();
                    y = stamped_transform.getOrigin().getY();
                    z = stamped_transform.getOrigin().getZ();
                    if(z < 0) z = -z;
                    if(x < 0) continue;
                    x -= 0.15;
                    /*
                        move the gripperbot using the obtained object position
                    */
                    grasp(x,y,z);
                }
                catch (tf::TransformException ex){
                    std::cout << "NO " << prop  << " " << type << "found to grasp" << std::endl;
                    ros::Duration(1.0).sleep();
                }
                //                graspObject=false;
            }
            imshow("demo",  demoImg);

            k = cv::waitKey(100);

            if((int)(k & 0xFF) == 27)
                break;

            if((int)(k & 0xFF) == 'g')
                graspObject=true;

            if((int)(k & 0xFF) == 'm')
                turn_mic_on();
        }

        demoImg.release();

        mSub->stopThread();

        cv::destroyWindow("demoImg");

        gripperbot_client.shutdown();
        cambot_client.shutdown();
        opengripper_client.shutdown();
        closegripper_client.shutdown();
        mic_on_client.shutdown();
        mic_off_client.shutdown();
        delete mSub;

        return 1;

    }
    catch (std::exception& e)
    {

        std::cout << e.what() << endl;
    }

    return 0;
};



