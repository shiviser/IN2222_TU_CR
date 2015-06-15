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

#include <tf/transform_listener.h>
#include <QApplication>
#include <stdexcept>
#include <exception>
#include <signal.h>
#include "main.h"
#include <robot_control_srvs/MoveToCS.h>
#include <opencv2/opencv.hpp>

using namespace std;

/*Declaring the clients and message containers for the connection to cambot*/
ros::ServiceClient cambot_client;
robot_control_srvs::MoveToCS cambot_srv;

void movegripperbot(float x, float y, float z)
{
    /*

      PUT YOUR CODE HERE
      Implement a function to move the gripperbot to position x,y,z (similar to movecambot)

    */







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
    cambot_srv.request.effector="camera";

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
        ROS_ERROR("Failed to call service /cambot_control/move_to_cs");
        throw;
    }
}


/*
    Function to grasp an object located at position (x,y,z).
*/
void grasp(float &x, float &y, float &z)
{

    /*
        PUT YOUR CODE HERE!
        An example of the steps required:
        1. move to an approach pose (e.g. 5cm above the object)
        2. move down to the object
        3. move up to the approach pose

    */










}


int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "demo_cogsys_viz_cpp");

        cv::Mat demoImg(480,640, CV_8UC3);
        cv::namedWindow("demo", 1);

        demoImg=cv::Scalar(0,0,0);

        cv::putText(demoImg, "Press ESC to quit app", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
        cv::putText(demoImg, "Press g to grasp GREEN object", cv::Point(30,70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);

        int k;

        ros::NodeHandle n;

        /*creating the client for moving the cambot to a given cartesian pose*/
        cambot_client = n.serviceClient<robot_control_srvs::MoveToCS>("/cambot_control/move_to_cs");

        /*move the cambot to the position, where the bag file was recorded from*/
        movecambot(0.215,0,0.4);

        tf::TransformListener listener;

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
                    /*checking for invalid/unreachable/noisy pose estimated from the camera*/
                    if(z < 0) z = -z;
                    if(x < 0) continue;
                    /*
                      move the gripperbot using the obtained object position
                    */
                    grasp(x,y,z);
                }
                catch (tf::TransformException ex){
                    std::cout << "NO " << prop  << " " << type << "found to grasp" << std::endl;
                    ros::Duration(1.0).sleep();
                }
                graspObject=false;
            }

            imshow("demo",  demoImg);

            k = cv::waitKey(100);

            if((int)(k & 0xFF) == 27)
                break;

            if((int)(k & 0xFF) == 'g')
                graspObject=true;

        }

        demoImg.release();

        cv::destroyWindow("demoImg");

        cambot_client.shutdown();

        return 1;

    }
    catch (std::exception& e)
    {

        std::cout << e.what() << endl;
    }

    return 0;
};



