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

#include "DetectorThread.h"

using namespace std;

DetectorThread::DetectorThread()
{
    //Tracker instance
    mDetector = new detection::Detector();

    mDetectorState = DetectorThread::detect;

    ros::NodeHandle _n;
    mDetectorStateService = _n.advertiseService("DETECTOR_STATE_SERVICE", &DetectorThread::state_callback, this);

    runThread = true;

}

DetectorThread::~DetectorThread()
{
    runThread = false;

    usleep(100000);

    mDetectorStateService.shutdown();

    delete mDetector;

    usleep(100000);

    wait(100);
    if(isRunning())
    {
        terminate();
        wait(); // needed for UNIX systems
    }
}

bool DetectorThread::state_callback(perception_srvs::PerceptionStateService::Request &req, perception_srvs::PerceptionStateService::Response &res)
{
    DetectorState state;
    if(req.request == perception_msgs::PerceptionState::STANDBY)
        state=DetectorThread::standby;
    else if(req.request == perception_msgs::PerceptionState::ACTIVE)
        state=DetectorThread::detect;
    else
        state=DetectorThread::standby;

    setDetectorState(state);
}

void DetectorThread::run()
{

    DetectorState state;
    while(runThread)
    {
        getDetectorState(state);
        switch (state)
        {
        case DetectorThread::standby:
            mDetector->standBy();
            break;

        case DetectorThread::detect:
            mDetector->detectObjects();
            break;

        default:
            mDetector->standBy();
            break;

        }

        QThread::msleep(10);
    }
}


