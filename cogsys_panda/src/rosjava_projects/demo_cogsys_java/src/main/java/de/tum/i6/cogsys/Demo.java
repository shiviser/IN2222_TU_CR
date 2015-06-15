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

import geometry_msgs.Point;
import geometry_msgs.Pose;

import org.ros.exception.RosException;

import de.tum.i6.cogsys.msgs.RecognizedSpeechSubscriberFactory;

public class Demo implements Runnable {

	private boolean running = true;
	
	protected boolean openGripper() {

		return true;
		
		// call gripper_control's OpenGripper ROS service
//		boolean status = RosWrapper.getOpenGripperClient().call();
//
//		if (!status) {
//			System.out.println("Cannot open gripper!");
//		}
//
//		return status;

	}

	protected boolean closeGripper() {

		return true;
		
//		// call gripper_control's CloseGripper ROS service
//		boolean status = RosWrapper.getCloseGripperClient().call();
//
//		if (!status) {
//			System.out.println("Cannot close gripper!");
//		}
//
//		return status;

	}

	protected boolean moveCambotToHomePos() {

		boolean status = RosWrapper.getCambotMoveToCSClient().call("wrist", 0.15f, 0.0f, 0.45f);

		if (!status) {
			System.out.println("Cannot move cambot to home pos!");
		}

		return status;

	}

	protected boolean moveGripperbotToHomePos() {

		boolean status = RosWrapper.getGripperbotMoveToCSClient().call("wrist", 0.07f, 0, 0.35f);

		if (!status) {
			System.out.println("Cannot move gripperbot to home pos!");
		}

		return status;

	}

	protected boolean moveCambotInCS(Point camPosition) {

		float x = (float)camPosition.getX();
		float y = (float)camPosition.getY();
		float z = (float)camPosition.getZ();

		// call robot_control's MoveToCS ROS service in order to move the robot 
		boolean status = RosWrapper.getCambotMoveToCSClient().call("camera", x, y, z);

		if (!status) {
			System.out.println("Cambot cannot reach x: " + x + ", y: " + y + ",z: " + z);
		}
		return status;

	}

	protected boolean moveGripperbotInCS(Point gripperPosition) {

		float x = (float)gripperPosition.getX();
		float y = (float)gripperPosition.getY();
		float z = (float)gripperPosition.getZ();

		// call robot_control's MoveToCS ROS service in order to move the robot 
		boolean status = RosWrapper.getGripperbotMoveToCSClient().call("gripper", x, y, z);

		if (!status) {
			System.out.println("Gripperbot cannot reach x: " + x + ", y: " + y + ",z: " + z);
		}
		return status;

	}

	protected boolean turnOnMic() throws RosException {

		// call speech_recognition's TurnOn ROS service in order to enable the mic 
		boolean status = RosWrapper.getTurnOnMicClient().call();

		if (!status) {
			System.out.println("Cannot turn on mic! Is it already on?");
		}
		return status;

	}

	protected boolean turnOffMic() throws RosException {

		// call speech_recognition's TurnOff ROS service in order to disable the mic 
		RosWrapper.getTurnOffMicClient().call();
		return true;

	}

	protected String getRecognizedSpeech() {

		String s = null;

		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {}

		RecognizedSpeechSubscriberFactory r = RosWrapper.getRecognizedSpeechSubscriber();

		if (r.hasMoreRecognizedSpeech()) {
			s = r.getNextRecognizedSpeech().getSpeech();
		}

		return s;

	}

	protected Point getTransform(String baseFrame, String targetFrame) {

		Point point = null;

		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {}

		if (baseFrame != null && targetFrame != null) {
			Pose p = RosWrapper.getGetTransformClient().call(baseFrame, targetFrame);
			if (p != null) {
				point = p.getPosition();	
				if (point.getX() < 0) {
					point = null;
				} else {
					point.setZ(Math.abs(point.getZ()));	
				}
			}
		}

		return point;

	}

	@Override
	public void run() {

		try {
			// get robots into initial state
			System.out.println("Init robot state");
			moveCambotToHomePos();
			moveGripperbotToHomePos();
			openGripper();
			// turn on microphone
			System.out.println("Turn on mic");
			turnOnMic();

		} catch (RosException re) {
			re.printStackTrace();
			return;
		}

		// wait for spoken command
		String command = null;
		do {
			command = getRecognizedSpeech();
			if (command != null && !command.isEmpty()) {
				System.out.println("Recognized command: " + command);
			}
		} while (command == null || command.isEmpty()) ;

		// turn off microphone
		System.out.println("Turn off mic");
		try {
			turnOffMic();
		} catch (RosException e1) {
			e1.printStackTrace();
			return;
		}

		boolean pickRed = command.indexOf("red") >= 0;

		while(running) {

			Point pickBall = null;
			Point placeContainer = null;
			
			if (pickRed) {
				pickBall = getTransform("gripperbot_base", "RED_BOX_0");
				placeContainer = getTransform("gripperbot_base", "RED_CONTAINER_0");
			} else {
				pickBall = getTransform("gripperbot_base", "GREEN_BOX_0");
				placeContainer = getTransform("gripperbot_base", "GREEN_CONTAINER_0");
			}
			
			if (pickBall == null || placeContainer == null) {
				continue;
			}

			// approach pose for gripper, so that we don't accidentally
			// collide with the object
			Point approachPositionPick = RosWrapper.getMessageFactory().newFromType(Point._TYPE);
			approachPositionPick.setX(pickBall.getX());
			approachPositionPick.setY(pickBall.getY());
			approachPositionPick.setZ(pickBall.getZ() + 0.05); // offset in z: + 5 cm

			// approach pose for gripper, so that we don't accidentally
			// collide with the object
			Point approachPositionPlace = RosWrapper.getMessageFactory().newFromType(Point._TYPE);
			approachPositionPlace.setX(placeContainer.getX());
			approachPositionPlace.setY(placeContainer.getY());
			approachPositionPlace.setZ(placeContainer.getZ() + 0.05); // offset in z: + 5 cm

			try {

				// move robot to approach pose for picking the ball
				moveGripperbotInCS(approachPositionPick);

				// move robot to grasp pose
				moveGripperbotInCS(pickBall);

				// call gripper_control's CloseGripper ROS service
				closeGripper();

				// move robot to approach pose
				moveGripperbotInCS(approachPositionPick);

				// move robot back to home position
				moveGripperbotToHomePos();

				// move robot to approach place pose
				moveGripperbotInCS(approachPositionPlace);					

				// move robot to place pose
				moveGripperbotInCS(placeContainer);

				// call gripper_control's OpenGripper ROS service
				openGripper();				

				// move robot back to home position after executing the pick and place task
				moveGripperbotToHomePos();

			} catch (Exception e) {
				e.printStackTrace();
			} finally {
				running = false;
			}


		}

		System.exit(0);

	}

}
