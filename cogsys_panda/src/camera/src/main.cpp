/** BEGIN LICENSE ************************************************************/
//    Copyright 2014 fortiss gmbh
//    All rights reserved.
//
//    Any use, distribution or replication without a written permission
//    is prohibited.
//
//    The name "fortiss gmbh" must not be used to endorse or promote
//    products derived from the source code without prior written permission.
//
//    You agree to indemnify, hold harmless and defend fortiss gmbh from
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

#include <QApplication>
#include <stdexcept>
#include <exception>
#include <signal.h>
#include <camera/ConfigData.h>
#include <camera/camInput/CamOpenCV.h>
#include <camera/main.h>
#include <comm_lib/ImagePublisher.h>
#include <comm_lib/ImageSubscriber.h>
using namespace std;
using namespace caminput;

int main(int argc, char** argv)
{
	
    try
    {
        ros::init(argc, argv, "camera");

        ConfigData config;
        config.parse();

        Cam* camin = new CamOpenCV(config.CAM_ID, config.CAMERA_INFO_SERVICE_NAME);

        cv::Mat img;

        comminterface::ImagePublisher imgPub(config.CAMERA_IMAGE_TOPIC_NAME);

        comminterface::ImageSubscriber imgSub(config.CAMERA_IMAGE_TOPIC_NAME);
        imgSub.start();

        int k;

        if(config.SHOW_DISPLAY)
        {
            cv::namedWindow("video", 1);
        }

        while(ros::ok())
        {
            camin->getImageCopy(img);
            imgPub.setImageData(img);

            if(config.SHOW_DISPLAY)
            {
                imgSub.getImageData(img);
                cv::imshow("video", img);
            }
            k = waitKey(20);
            if(k != -1)
                break;
        }

        if(config.SHOW_DISPLAY)
        {
            destroyWindow("video");
        }

        delete camin;

        return 1;
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << endl;
    }

}




