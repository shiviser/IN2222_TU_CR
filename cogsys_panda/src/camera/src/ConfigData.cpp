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
#include <camera/ConfigData.h>
#include <ros/package.h>

using namespace std;

ConfigData::ConfigData()
{

}

ConfigData::~ConfigData()
{

}

void ConfigData::parse()
{
	
    QString filename((ros::package::getPath("camera") + "/config/config.ini").c_str());
    QFileInfo config(filename);

    if(!config.exists())
    {
        std::cout<<"error reading config.ini file"<<std::endl;
        throw;
    }

    QSettings iniFile(filename, QSettings::IniFormat);

    iniFile.beginGroup("CAMERA");
    CAM_ID = iniFile.value("CAM_ID", 0).toInt();
    CAMERA_IMAGE_TOPIC_NAME = iniFile.value("CAMERA_IMAGE_TOPIC_NAME", "").toString().toStdString();
    CAMERA_INFO_SERVICE_NAME = iniFile.value("CAMERA_INFO_SERVICE_NAME", "").toString().toStdString();
    SHOW_DISPLAY = iniFile.value("SHOW_DISPLAY", false).toBool();
    std::cout << "CAM_ID " << CAM_ID << std::endl;
    std::cout << "CAMERA_IMAGE_TOPIC_NAME " << CAMERA_IMAGE_TOPIC_NAME << std::endl;
    std::cout << "CAMERA_INFO_SERVICE_NAME " << CAMERA_INFO_SERVICE_NAME << std::endl;
    std::cout << "SHOW_DISPLAY " << (int)SHOW_DISPLAY << std::endl;
    iniFile.endGroup();
    
}

