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
#include <robot_control_srvs/MoveToCS.h>
#include <gripper_control_srvs/OpenGripper.h>
#include <gripper_control_srvs/CloseGripper.h>
#include <opencv2/opencv.hpp>
#include <speech_recognition_srvs/TurnOff.h>
#include <speech_recognition_srvs/TurnOn.h>
#include <speech_recognition_msgs/RecognizedSpeech.h>
#include <world_model_srvs/GetTransform.h>

using namespace std;

/*Declaring the clients and message containers for the connection to cambot*/
ros::ServiceClient cambot_client;
robot_control_srvs::MoveToCS cambot_srv;

ros::ServiceClient gripperbot_client;
robot_control_srvs::MoveToCS gripperbot_srv;

ros::ServiceClient world_model_client;
world_model_srvs::GetTransform world_model_srv;

ros::Subscriber sub_speech;

ros::ServiceClient mic_on_client;
speech_recognition_srvs::TurnOn mic_on_srv;

ros::ServiceClient mic_off_client;
speech_recognition_srvs::TurnOff mic_off_srv;

ros::ServiceClient opengripper_client;
gripper_control_srvs::OpenGripper opengripper_srv;

ros::ServiceClient closegripper_client;
gripper_control_srvs::CloseGripper closegripper_srv;

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
    }
}


void movegripperbot(float x, float y, float z)
{
    /*creating the service request*/
    gripperbot_srv.request.x=x;
    gripperbot_srv.request.y=y;
    gripperbot_srv.request.z=z;
    gripperbot_srv.request.effector="gripper";

    /*calling the move service*/
    if (gripperbot_client.call(gripperbot_srv))
    {
        /*checking for success of the move*/
        if(gripperbot_srv.response.success)
            std::cout << "Approach successfull " <<std::endl;
        else
            std::cout << "Approach failed to " << x << " " << y << " " << z << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service /robot_control/move_to_cs");
        throw;
    }
}
/*
    Moves the cambot to position x,y,z
    The service definitions have been defined in the robot_control_srvs/srv/MoveToCS.srv file
*/

void movecambot(float x, float y, float z)
{
    /*creating the service request*/
    cambot_srv.request.x=x;
    cambot_srv.request.y=y;
    cambot_srv.request.z=z;
    cambot_srv.request.effector="wrist";

    /*calling the move service*/
    if (cambot_client.call(cambot_srv))
    {
        /*checking for success of the move*/
        if(cambot_srv.response.success)
            std::cout << "Approach successfull " <<std::endl;
        else
            std::cout << "Approach failed to " << x << " " << y << " " << z << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service /robot_control/move_to_cs");
        throw;
    }
}


/*
    Function to approach an object located at position (x,y,z).
*/
void grasp(float &x, float &y, float &z)
{

    float bottom,top;
    bottom=z;
    top=z+0.05;

    std::cout << "Approaching object top" << std::endl;
    z=top;
    movegripperbot(x,y,z);

}

/*
    Function to pick up an object named <object_name>
*/
void pick(std::string object_name)
{

    world_model_srv.request.base_frame = "gripperbot_base";
    world_model_srv.request.target_frame = object_name;

    if(world_model_client.call(world_model_srv))
    {
        if(world_model_srv.response.success)
        {
            if(world_model_srv.response.pose.position.x < 0)
                return;
            if(world_model_srv.response.pose.position.z < 0)
                world_model_srv.response.pose.position.z = -world_model_srv.response.pose.position.z;
            movegripperbot(world_model_srv.response.pose.position.x, world_model_srv.response.pose.position.y, world_model_srv.response.pose.position.z+0.1);
            opengripper();
            movegripperbot(world_model_srv.response.pose.position.x, world_model_srv.response.pose.position.y, world_model_srv.response.pose.position.z);
            closegripper();
            movegripperbot(world_model_srv.response.pose.position.x, world_model_srv.response.pose.position.y, world_model_srv.response.pose.position.z+0.1);
        }
        else
        {
            ROS_ERROR("Transform between object and gripperbot_base doesnt exist");
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /world_model/get_transform");
    }

}

/*
    Function to place the currently grasped object on the object named <container_name>
*/
void place(std::string container_name)
{

    world_model_srv.request.base_frame = "gripperbot_base";
    world_model_srv.request.target_frame = container_name;

    if(world_model_client.call(world_model_srv))
    {
        if(world_model_srv.response.success)
        {
            if(world_model_srv.response.pose.position.x < 0)
                return;
            if(world_model_srv.response.pose.position.z < 0)
                world_model_srv.response.pose.position.z = -world_model_srv.response.pose.position.z;

            movegripperbot(world_model_srv.response.pose.position.x, world_model_srv.response.pose.position.y, world_model_srv.response.pose.position.z+0.1);
            movegripperbot(world_model_srv.response.pose.position.x, world_model_srv.response.pose.position.y, world_model_srv.response.pose.position.z);
            opengripper();
            movegripperbot(world_model_srv.response.pose.position.x, world_model_srv.response.pose.position.y, world_model_srv.response.pose.position.z+0.1);
        }
        else
        {
            ROS_ERROR("Transform between object and gripperbot_base doesnt exist");
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /world_model/get_transform");
    }

}

void turn_mic_off()
{
    std::cout << "mic off" << std::endl;
    std::cout << mic_off_client.call(mic_off_srv) << std::endl;
}


/*
    function for analyzing the recognized speech and commanding the robot based on it
*/
void analyzeSpeech(const speech_recognition_msgs::RecognizedSpeechConstPtr &speech)
{
    std::string s = speech->speech;

    std::cout << "received command" << s << std::endl;
    if (s.find("green") != string::npos)
    {
        turn_mic_off();
        pick("GREEN_BOX_0");
        place("GREEN_CONTAINER_0");
    }
    else if (s.find("pink") != string::npos)
    {
        turn_mic_off();
        pick("RED_BOX_0");
        place("RED_CONTAINER_0");
    }


    turn_mic_off();

}


void turn_mic_on()
{
    std::cout << "mic on" << std::endl;
    std::cout << mic_on_client.call(mic_on_srv) << std::endl;
}

/*
  Modify this implementation to pick GREEN_BOX_0 and place it in the GREEN_CONTAINER_0, grasp RED_BOX_0 and place it in the RED_CONTAINER_0

  Hint: Use the 'pick' and 'place' functions you implemented.

*/
int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "demo_cogsys_viz_cpp");

        cv::Mat demoImg(480,640, CV_8UC3);
        cv::namedWindow("demo", 1);

        demoImg=cv::Scalar(0,0,0);

        cv::putText(demoImg, "Press ESC to quit app", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
        cv::putText(demoImg, "Press g to pick GREEN ball and place it in GREEN container", cv::Point(30,70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
        cv::putText(demoImg, "Press r to pick RED ball and place it in RED container", cv::Point(30,70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
        cv::putText(demoImg, "Press p to pick GREEN ball", cv::Point(30,70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
        cv::putText(demoImg, "Press m to turn on mic", cv::Point(30,70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);

        int k;

        ros::NodeHandle n;

        world_model_client = n.serviceClient<world_model_srvs::GetTransform>("/world_model/get_transform");

        /*creating the client for moving the cambot to a given cartesian pose*/
        gripperbot_client = n.serviceClient<robot_control_srvs::MoveToCS>("/gripperbot_control/move_to_cs");
        cambot_client = n.serviceClient<robot_control_srvs::MoveToCS>("/cambot_control/move_to_cs");
        opengripper_client = n.serviceClient<gripper_control_srvs::OpenGripper>("/gripper_control/open_gripper");
        closegripper_client = n.serviceClient<gripper_control_srvs::CloseGripper>("/gripper_control/close_gripper");

        sub_speech = n.subscribe("/speech_recognition/output", 100, analyzeSpeech);
        mic_on_client = n.serviceClient<speech_recognition_srvs::TurnOn>("/speech_recognition/turn_on");
        mic_off_client = n.serviceClient<speech_recognition_srvs::TurnOff>("/speech_recognition/turn_off");

        /*move the cambot to the position, where the bag file was recorded from*/
        movecambot(0.15,0,0.45);

        tf::TransformListener listener;

        bool pickPlaceGreen = false;
        bool pickPlaceRed = false;
        bool graspObject = false;

        while(ros::ok())
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
                    /* get the pose of the object w.r.t gripperbot_base frame */
                    listener.waitForTransform(ss_object_name.str(), "gripperbot_base",
                                              ros::Time(0), ros::Duration(1.0));
                    listener.lookupTransform(ss_object_name.str(), "gripperbot_base",
                                             ros::Time(0), stamped_transform);
                    float x,y,z;
                    x = stamped_transform.getOrigin().getX();
                    y = stamped_transform.getOrigin().getY();
                    z = stamped_transform.getOrigin().getZ();
                    /*checking for invalid/unreachable/noisy pose estimated from the camera*/
                    if(z < 0) z = -z;
                    if(x < 0) continue;
                    /* move the gripperbot using the obtained object position */
                    grasp(x,y,z);
                }
                catch (tf::TransformException ex){
                    std::cout << "NO " << prop  << " " << type << "found to grasp" << std::endl;
                    ros::Duration(1.0).sleep();
                }
                graspObject = false;
            }
            if(pickPlaceGreen)
            {
                pick("GREEN_BOX_0");
                place("GREEN_CONTAINER_0");
                pickPlaceGreen = false;
                pickPlaceRed = true;
            }
            if(pickPlaceRed)
            {
                pick("RED_BOX_0");
                place("RED_CONTAINER_0");
                pickPlaceRed = false;
                pickPlaceGreen = true;
            }

            imshow("demo",  demoImg);

            k = cv::waitKey(100);

            if((int)(k & 0xFF) == 27)
                break;

            if((int)(k & 0xFF) == 'p')
                graspObject=true;

            if((int)(k & 0xFF) == 'g')
                pickPlaceGreen=true;

            if((int)(k & 0xFF) == 'r')
                pickPlaceRed=true;

            if((int)(k & 0xFF) == 'm')
                turn_mic_on();

            ros::spinOnce();
        }

        demoImg.release();

        cv::destroyWindow("demoImg");

        cambot_client.shutdown();
        gripperbot_client.shutdown();
        opengripper_client.shutdown();
        closegripper_client.shutdown();
        world_model_client.shutdown();
        mic_on_client.shutdown();
        mic_off_client.shutdown();

        return 1;

    }
    catch (std::exception& e)
    {

        std::cout << e.what() << endl;
    }

    return 0;
};



