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
#include <camera/camInput/CamGigE.h>
#include <stdlib.h>

namespace caminput
{

	CamGigE::CamGigE(int id):
			Cam(id)
	{
		mWidth = XSIZE;
		mHeight = YSIZE;

		xshift = 0;
		yshift = 0;
		exposure = 2*1000;
		gain = 300;
		isCapturing = false;
		framerate = 15.0f;

		mId = id;

		g_type_init ();
		camera = arv_camera_new(NULL);
		if(camera == NULL)
		{
			throw "No camera found";
		}

		arv_camera_set_region(camera, xshift, yshift, mWidth, mHeight);
		arv_camera_set_exposure_time(camera, exposure);
		arv_camera_set_gain(camera, gain);

		arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_MONO_8);
		//arv_camera_set_pixel_format(camera, ARV_PIXEL_FORMAT_BAYER_BG_8);


		width = (int)mWidth;
		height = (int)mHeight;
		arv_camera_get_region (camera, &xshift, &yshift, &width, &height);
		payload = arv_camera_get_payload (camera);

		stream = arv_camera_create_stream (camera, NULL, NULL);
		if(stream == NULL) {
			throw "Cannot create stream";
		}
		g_object_set(stream, "packet-timeout", (unsigned) 20 * 1000,
							 "frame-retention", (unsigned) 100 * 1000,
							 NULL);
		for(int i = 0; i < 4; i++)
			arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL));

		arv_camera_set_acquisition_mode(camera, ARV_ACQUISITION_MODE_CONTINUOUS);
		arv_camera_set_frame_rate(camera, framerate);

		rawdata = new unsigned char[width*height];


	}

	CamGigE::~CamGigE()
	{

		arv_camera_stop_acquisition(camera);
		isCapturing = false;

		arv_camera_stop_acquisition(camera);
		g_object_unref(stream);
		g_object_unref(camera);


	}

	void CamGigE::start()
	{
		if(!isCapturing)
			arv_camera_start_acquisition (camera);

		isCapturing = true;

	}

	void CamGigE::stop()
	{
		arv_camera_stop_acquisition(camera);
		isCapturing = false;
	}

	unsigned char* CamGigE::grab(double timeout)
	{
		ArvBuffer *buffer;

		if(timeout < 0)
			timeout = +INFINITY;

		while(timeout > 0)
		{
			buffer = arv_stream_pop_buffer(stream);

			if (buffer != NULL) {
				memcpy(rawdata, (unsigned char*) buffer->data, mWidth*mHeight);
				arv_stream_push_buffer(stream, buffer);

				return rawdata;
			}

			usleep(1000);
			timeout -= 0.001f;

		}

		throw "Camera capture timeout";

	}

	void CamGigE::getImageCopy(cv::Mat& inputImage)
	{

	}

}



