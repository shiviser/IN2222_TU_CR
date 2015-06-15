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
* started by Suraj Nair
*/
#include <QApplication>
#include <stdexcept>
#include <exception>
#include <signal.h>
#include <detector/DetectorThread.h>
#include "main.h"
#include <comm_lib/ActorsSubscriber.h>
#include <comm_lib/util/Util.h>
#include <comm_lib/CommonDefs.h>

using namespace std;

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "object_detector");

        DetectorThread detector;
        detector.start();

        cv::Mat detectImg;
        cv::Mat colorSegImg;
        namedWindow("detector", 1);
        //namedWindow("seg", 1);
        //namedWindow("cont", 1);

        int k;
        detector.getDetectorPtr()->setColourSegDebug(true);


        while(ros::ok())
        {

            detector.getDetectorPtr()->getOutputImg(detectImg);
            detector.getDetectorPtr()->getColourSegmentedImg(colorSegImg);

            imshow("detector",  detectImg);
            imshow("seg", colorSegImg);

            //std::cout << "Main loop" << std::endl;

            k = waitKey(100);

            if((int)(k & 0xFF) == 27)
                break;
        }

        detector.stopThread();

        detectImg.release();
        colorSegImg.release();

        destroyWindow("detector");
        destroyWindow("seg");

        return 1;

    }
    catch (std::exception& e)
    {

        std::cout << e.what() << endl;
    }

    return 0;
};



