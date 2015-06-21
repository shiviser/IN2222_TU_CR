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

int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, "marker_grabber");

        ros::NodeHandle n;

        ProdMaster prod_master(&n);
        BotController bot(&n);
        ShapesDetector shapes_detector(&n);

        while (ros::ok()) {

               // bot.movetohome(false, true);

              //  float x1 = 0.214;
                //float y1 = 0.172;
                //float z1 = 0.3;

float x1 = 0.0;
float y1 = -0.39;
float z1 = 0.3;

                bot.movecambot(x1, y1, z1);

		vector<int> object = shapes_detector.get_object(75, true);

		cout << object[0] << " " << object[1] << endl;
        }

        ros::spinOnce();

    return 1;

}
catch (std::exception &e) {

    std::cout << e.what() << endl;
}

return 0;
};



