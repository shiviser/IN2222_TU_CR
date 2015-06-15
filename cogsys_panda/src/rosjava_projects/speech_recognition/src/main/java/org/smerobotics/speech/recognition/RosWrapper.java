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

package org.smerobotics.speech.recognition;

import org.ros.exception.ServiceException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.topic.Publisher;
import org.smerobotics.util.Util;

import speech_recognition_msgs.RecognizedSpeech;
import speech_recognition_srvs.AddGrammarRequest;
import speech_recognition_srvs.AddGrammarResponse;
import speech_recognition_srvs.LoadGrammarRequest;
import speech_recognition_srvs.LoadGrammarResponse;
import speech_recognition_srvs.TurnOffRequest;
import speech_recognition_srvs.TurnOffResponse;
import speech_recognition_srvs.TurnOnRequest;
import speech_recognition_srvs.TurnOnResponse;


public class RosWrapper extends AbstractNodeMain implements SpeechRecListener {

	
	public static final String PACKAGE_NAME = "rosjava_projects";
	public static final String NODE_NAME = "speech_recognition";

	public static final String SERVICE_NAME_TURN_ON = NODE_NAME + "/turn_on";
	public static final String SERVICE_NAME_TURN_OFF = NODE_NAME + "/turn_off";
	public static final String SERVICE_NAME_ADD_GRAMMAR = NODE_NAME + "/add_grammar";
	public static final String SERVICE_NAME_LOAD_GRAMMAR = NODE_NAME + "/load_grammar";

	public static final String TOPIC_NAME_REC_SPEECH_PUB = NODE_NAME + "/output";

	protected static ConnectedNode connectedNode;
	public static Publisher<speech_recognition_msgs.RecognizedSpeech> pubRecSpeech;

	protected static String localPath;
	protected static SpeechRecognizer rec;


	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(NODE_NAME);
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {

		RosWrapper.connectedNode = connectedNode;

		localPath = Util.getLocalRosjavaProjectPath(PACKAGE_NAME, NODE_NAME);

		rec = SpeechRecognizer.getSpeechRecognizer();
		rec.addSpeechRecListener(this);

		pubRecSpeech = connectedNode.newPublisher(TOPIC_NAME_REC_SPEECH_PUB,
				speech_recognition_msgs.RecognizedSpeech._TYPE);

		connectedNode.newServiceServer(RosWrapper.SERVICE_NAME_TURN_ON,
				speech_recognition_srvs.TurnOn._TYPE,
				new TurnOnServiceResponseBuilder());

		connectedNode.newServiceServer(RosWrapper.SERVICE_NAME_TURN_OFF,
				speech_recognition_srvs.TurnOff._TYPE,
				new TurnOffServiceResponseBuilder());

		connectedNode.newServiceServer(RosWrapper.SERVICE_NAME_ADD_GRAMMAR,
				speech_recognition_srvs.AddGrammar._TYPE,
				new AddGrammarServiceResponseBuilder());

		connectedNode.newServiceServer(RosWrapper.SERVICE_NAME_LOAD_GRAMMAR,
				speech_recognition_srvs.LoadGrammar._TYPE,
				new LoadGrammarServiceResponseBuilder());

	}

	@Override
	public void onShutdown(Node node) {

		if (rec != null) {
			rec.shutdown();	
		}
		
	}
	
	@Override
	public void resultCallback(String text, float score) {

		if (pubRecSpeech != null) {

			RecognizedSpeech s = pubRecSpeech.newMessage();
			s.setSpeech(text);
			s.setScore(score);
			pubRecSpeech.publish(s);
			
		}
		
	}

	static class TurnOnServiceResponseBuilder implements
	ServiceResponseBuilder<TurnOnRequest, TurnOnResponse> {

		@Override
		public void build(TurnOnRequest request, TurnOnResponse response)
				throws ServiceException {
			
			response.setSuccess(rec.turnOnMic());

		}

	}

	static class TurnOffServiceResponseBuilder implements
	ServiceResponseBuilder<TurnOffRequest, TurnOffResponse> {

		@Override
		public void build(TurnOffRequest request, TurnOffResponse response)
				throws ServiceException {
			
			rec.turnOffMic();

		}

	}
	
	static class AddGrammarServiceResponseBuilder implements
	ServiceResponseBuilder<AddGrammarRequest, AddGrammarResponse> {

		@Override
		public void build(AddGrammarRequest request, AddGrammarResponse response)
				throws ServiceException {
			
			String grammarName = rec.addGrammar(request.getJsgfGrammar());
			if (grammarName != null) {
				response.setSuccess(true);
				response.setName(grammarName);
			} else {
				response.setSuccess(false);
			}

		}

	}
	
	static class LoadGrammarServiceResponseBuilder implements
	ServiceResponseBuilder<LoadGrammarRequest, LoadGrammarResponse> {

		@Override
		public void build(LoadGrammarRequest request, LoadGrammarResponse response)
				throws ServiceException {
			
			response.setSuccess(rec.loadGrammar(request.getName()));

		}

	}
	
}
