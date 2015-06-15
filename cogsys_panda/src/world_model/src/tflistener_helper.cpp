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

#include "tflistener_helper.h"

TFListenerHelper::TFListenerHelper()
{
    tf_server = nh.advertiseService("/world_model/get_transform", &TFListenerHelper::tf_service_callback, this);
}

/*
  Implement this callback function for the tf_service server.
  This service takes the base_frame and target_frame as arguments, queries ros::tf for the transformation between these frames, and returns the result of this query. 

  req - data structure containing the transformation request. (explained in file world_model_srvs/srv/GetTransform.srv)
  res - data structure containing the result of the transformation query to ros::tf. (explained in file world_model_srvs/srv/GetTransform.srv)
*/

bool TFListenerHelper::tf_service_callback(world_model_srvs::GetTransformRequest &req, world_model_srvs::GetTransformResponse &res)
{
    /*
	PUT YOUR CODE HERE!!
    */
    tf::StampedTransform stamped_transform;
    try{
        /*wait for transform to be published for 1.0sec*/
        listener.waitForTransform(req.target_frame, req.base_frame,
                                  ros::Time(0), ros::Duration(1.0));
        /*get the transform*/
        listener.lookupTransform(req.target_frame, req.base_frame,
                                 ros::Time(0), stamped_transform);
        res.pose.position.x = stamped_transform.getOrigin().x();
        res.pose.position.y = stamped_transform.getOrigin().y();
        res.pose.position.z = stamped_transform.getOrigin().z();
        res.pose.orientation.x = stamped_transform.getRotation().x();
        res.pose.orientation.y = stamped_transform.getRotation().y();
        res.pose.orientation.z = stamped_transform.getRotation().z();
        res.pose.orientation.w = stamped_transform.getRotation().w();
        res.success = true;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        res.success = false;
        res.error_msg = ex.what();
    }
    return true;

}
