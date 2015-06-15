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

package de.tum.i6.cogsys;

import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.smerobotics.util.srv.ServiceRequestBuilderFactory;

import de.tum.i6.cogsys.msgs.DetectedObjectsSubscriberFactory;
import de.tum.i6.cogsys.msgs.RecognizedSpeechSubscriberFactory;
import de.tum.i6.cogsys.srvs.CambotMoveJointsRequestBuilderFactory;
import de.tum.i6.cogsys.srvs.CambotMoveToCSRequestBuilderFactory;
import de.tum.i6.cogsys.srvs.CloseGripperRequestBuilderFactory;
import de.tum.i6.cogsys.srvs.GetTransformRequestBuilderFactory;
import de.tum.i6.cogsys.srvs.GripperbotMoveJointsRequestBuilderFactory;
import de.tum.i6.cogsys.srvs.GripperbotMoveToCSRequestBuilderFactory;
import de.tum.i6.cogsys.srvs.OpenGripperRequestBuilderFactory;
import de.tum.i6.cogsys.srvs.TurnOffMicRequestBuilderFactory;
import de.tum.i6.cogsys.srvs.TurnOnMicRequestBuilderFactory;

public class RosWrapper extends AbstractNodeMain {

	public static final String NODE_NAME = "demo_cogsys_java";
	
	public static MessageFactory messageFactory;
	
	protected static TurnOnMicRequestBuilderFactory clientTurnOnMic = new TurnOnMicRequestBuilderFactory();
	protected static TurnOffMicRequestBuilderFactory clientTurnOffMic = new TurnOffMicRequestBuilderFactory();
	protected static OpenGripperRequestBuilderFactory clientOpenGripper = new OpenGripperRequestBuilderFactory();
	protected static CloseGripperRequestBuilderFactory clientCloseGripper = new CloseGripperRequestBuilderFactory();
	protected static CambotMoveToCSRequestBuilderFactory clientCambotMoveToCS = new CambotMoveToCSRequestBuilderFactory();
	protected static GripperbotMoveToCSRequestBuilderFactory clientGripperbotMoveToCS = new GripperbotMoveToCSRequestBuilderFactory();
	protected static CambotMoveJointsRequestBuilderFactory clientCambotMoveJoints = new CambotMoveJointsRequestBuilderFactory();
	protected static GripperbotMoveJointsRequestBuilderFactory clientGripperbotMoveJoints = new GripperbotMoveJointsRequestBuilderFactory();
	protected static GetTransformRequestBuilderFactory clientGetTransform = new GetTransformRequestBuilderFactory();
	
	protected static RecognizedSpeechSubscriberFactory subRecognizedSpeech = new RecognizedSpeechSubscriberFactory();
	protected static DetectedObjectsSubscriberFactory subDetectedObjects = new DetectedObjectsSubscriberFactory();
	
	private static Demo dt;
	
	private static ServiceRequestBuilderFactory[] serviceClientBuilders =
			new ServiceRequestBuilderFactory[] {
		clientCambotMoveToCS, clientGripperbotMoveToCS, clientCambotMoveJoints, 
		clientGripperbotMoveJoints, clientTurnOnMic, clientTurnOffMic, 
		clientGetTransform
	};
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(NODE_NAME);
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {

		// get a message factory for service handlers to generate messages
		messageFactory = connectedNode.getTopicMessageFactory();

		// create ROS service clients
		ServiceRequestBuilderFactory.createServiceClients(connectedNode, serviceClientBuilders);

		// subscribe to speech recognition's output topic
		subRecognizedSpeech.addSubscribers(connectedNode);
		subDetectedObjects.addSubscribers(connectedNode);
		
		// create Demo which operates the robots and sensors 
		dt = new Demo();
		
		// start demo
		dt.run();
		
	}
	
	public static MessageFactory getMessageFactory() {
		return messageFactory;
	}
	
	public static RecognizedSpeechSubscriberFactory getRecognizedSpeechSubscriber() {
		return subRecognizedSpeech;
	}

	public static DetectedObjectsSubscriberFactory getDetectedObjectsSubscriber() {
		return subDetectedObjects;
	}
	
	public static TurnOnMicRequestBuilderFactory getTurnOnMicClient() {
		return clientTurnOnMic;
	}	

	public static TurnOffMicRequestBuilderFactory getTurnOffMicClient() {
		return clientTurnOffMic;
	}
	
	public static OpenGripperRequestBuilderFactory getOpenGripperClient() {
		return clientOpenGripper;
	}

	public static CloseGripperRequestBuilderFactory getCloseGripperClient() {
		return clientCloseGripper;
	}
	
	public static CambotMoveToCSRequestBuilderFactory getCambotMoveToCSClient() {
		return clientCambotMoveToCS;
	}

	public static GripperbotMoveToCSRequestBuilderFactory getGripperbotMoveToCSClient() {
		return clientGripperbotMoveToCS;
	}

	public static CambotMoveJointsRequestBuilderFactory getCambotMoveJointsClient() {
		return clientCambotMoveJoints;
	}

	public static GripperbotMoveJointsRequestBuilderFactory getGripperbotMoveJointsClient() {
		return clientGripperbotMoveJoints;
	}

	public static GetTransformRequestBuilderFactory getGetTransformClient() {
		return clientGetTransform;
	}
	
}
