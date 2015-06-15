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
#include <camera/camInput/Cam.h>
#include <ros/package.h>

namespace caminput
{

Cam::Cam(int id, std::string service_name)
{
    mId = id;
    mServiceName=service_name;

    std::string filename = ros::package::getPath("camera") + "/config/cam.xml";

    loadIntrinsics(filename);
    loadExtrinsics(filename);

    cv::Mat intrinsics_pad(3,4, DataType<double>::type);

    for(int i=0; i < intrinsics.rows; ++i)
    {
        for(int j=0; j < intrinsics.cols; ++j)
        {
            intrinsics_pad.at<double>(i,j) = intrinsics.at<double>(i,j);
        }
    }
    intrinsics_pad.at<double>(0,3) = 0.0;
    intrinsics_pad.at<double>(1,3) = 0.0;
    intrinsics_pad.at<double>(2,3) = 0.0;

    projection = intrinsics_pad * extrinsics;
    std::cout << "projection extrinsics matrix" <<std::endl;
    std::cout << projection << std::endl;

    ros::NodeHandle _n;
    mCalibInfoService = _n.advertiseService(mServiceName, &Cam::cameraInfo_callback, this);

}

Cam::~Cam()
{
    mCalibInfoService.shutdown();
}


void Cam::loadIntrinsics(std::string filename)
{
    FileStorage fs(filename, FileStorage::READ);
    if(fs.isOpened())
    {
        fs["intrinsics"] >> intrinsics;
        fs.release();
        std::cout << "camera intrinsics matrix: " << intrinsics << std::endl;
    }
    else
    {
        std::cout << "Could not open file " << filename << std::endl;
        throw;
    }

}

void Cam::loadExtrinsics(std::string filename)
{
    FileStorage fs(filename, FileStorage::READ);
    if(fs.isOpened())
    {
        fs["extrinsics"] >> extrinsics;
        fs.release();
        std::cout << "camera extrinsics matrix: " << extrinsics << std::endl;
    }
    else
    {
        std::cout << "Could not open file " << filename << std::endl;
        throw;
    }
}

bool Cam::cameraInfo_callback(camera_srvs::CameraInfo::Request &req, camera_srvs::CameraInfo::Response &res)
{
    res.response.width=mWidth;
    res.response.height=mHeight;
    res.response.encoding="BGR";

    double *input = (double*)(intrinsics.data);
    for(int i=0;i<9;++i)
    {
        res.response.K[i]=input[i];
        std::cout << "res.response.K[i] " << res.response.K[i] << std::endl;
    }

    double *input1 = (double*)(extrinsics.data);
    for(int i=0;i<16;++i)
    {
        res.response.E[i]=input1[i];
    }


    double *input2 = (double*)(projection.data);
    for(int i=0;i<12;++i)
    {
        res.response.P[i]=input2[i];
    }

    return true;
}

}



