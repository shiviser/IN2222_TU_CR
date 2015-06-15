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
#include <camera/camInput/CamOpenCV.h>

namespace caminput
{

    CamOpenCV::CamOpenCV(int id, std::string service_name):
    Cam(id, service_name)
	{
		cam.open(id);

		cam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
		cam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

		cv::Mat img;
		cam >> img;

		std::cout <<" cam ptr " << &cam << std::endl;

		if(!cam.isOpened())  // check if we succeeded
			throw;

		mWidth  = img.size().width;
		mHeight = img.size().height;

	}

	CamOpenCV::~CamOpenCV()
	{
		cam.release();
	}

	void CamOpenCV::start()
	{

	}

	void CamOpenCV::stop()
	{

	}

	void CamOpenCV::getImageCopy(cv::Mat& inputImage)
	{

		cam >> inputImage;
	}

}