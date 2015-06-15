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

package de.tum.i6.cogsys.msgs;

import java.util.Vector;

import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.smerobotics.util.msg.SubscriberFactory;

import actor_msgs.ActorVec;


public class DetectedObjectsSubscriberFactory extends SubscriberFactory {

	public static final String topic = "/object_detector/objects_data";

	protected Subscriber<ActorVec> subscriber;
	
	protected Vector<ActorVec> detectedObjects = new Vector<ActorVec>(1);

	public ActorVec getDetectedObjects() {
		
		ActorVec actors = null;
		
		synchronized (detectedObjects) {

			if (detectedObjects.size()>0) {
				actors = detectedObjects.get(0);
			}
			
		}

		return actors;
		
	}

	@Override
	public void addSubscribers(ConnectedNode connectedNode) {

		if (subscriber == null) {

			detectedObjects.setSize(1);
			
			subscriber = connectedNode.newSubscriber(topic, ActorVec._TYPE);
			subscriber.addMessageListener(new MessageListener<ActorVec>() {
				@Override
				public void onNewMessage(ActorVec message) {

					detectedObjects.set(0, message);

				}
			});

		}

	}

	@Override
	public void removeSubscribers() {

		if (subscriber != null) {
			subscriber.shutdown();
			subscriber = null;
		}

	}

}
