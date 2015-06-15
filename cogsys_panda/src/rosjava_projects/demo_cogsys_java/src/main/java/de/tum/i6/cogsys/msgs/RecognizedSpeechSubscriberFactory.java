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

import speech_recognition_msgs.RecognizedSpeech;


public class RecognizedSpeechSubscriberFactory extends SubscriberFactory {

	public static final String topic = "/speech_recognition/output";

	protected Subscriber<RecognizedSpeech> subscriber;
	
	protected Vector<RecognizedSpeech> utterances = new Vector<RecognizedSpeech>();

	public boolean hasMoreRecognizedSpeech() {
		return utterances.size() > 0;
	}

	public RecognizedSpeech getNextRecognizedSpeech() {
		return utterances.remove(0);
	}
	
	@Override
	public void addSubscribers(ConnectedNode connectedNode) {

		if (subscriber == null) {

			subscriber = connectedNode.newSubscriber(topic, RecognizedSpeech._TYPE);
			subscriber.addMessageListener(new MessageListener<RecognizedSpeech>() {
				@Override
				public void onNewMessage(RecognizedSpeech message) {

					utterances.add(message);

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
