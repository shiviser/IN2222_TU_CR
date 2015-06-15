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

package de.tum.i6.cogsys.srvs;

import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.smerobotics.util.srv.ServiceRequestBuilderFactory;

import speech_recognition_srvs.TurnOn;
import speech_recognition_srvs.TurnOnRequest;
import speech_recognition_srvs.TurnOnResponse;

public class TurnOnMicRequestBuilderFactory extends ServiceRequestBuilderFactory {

	public static final String SERVICE_NAME_TURN_ON = "speech_recognition/turn_on";
	private ServiceClient<TurnOnRequest, TurnOnResponse> clientTurnOnMic;

	private final Object sync = new Object();
	private volatile boolean responsePending = false;

	private boolean ok = false;
	
	@Override
	public synchronized void addServiceClients(ConnectedNode connectedNode) {

		if (clientTurnOnMic == null) {

			boolean found = false;

			while (!found) {

				try {

					clientTurnOnMic = connectedNode.newServiceClient(
							SERVICE_NAME_TURN_ON,
							TurnOn._TYPE);

					found = true;

					System.out.println("    Found service '"
							+ SERVICE_NAME_TURN_ON
							+ "'. (type: "
							+ TurnOn._TYPE + ")");

				} catch (ServiceNotFoundException e) {

					System.out.println("  Waiting for service '"
							+ SERVICE_NAME_TURN_ON
							+ "' to start up. (type: "
							+ TurnOn._TYPE + ")");

					try {
						Thread.sleep(3000);
					} catch (InterruptedException ie) {
					}

				}

			}

		}

	}

	@Override
	public synchronized void removeServiceClients() {

		if (clientTurnOnMic != null) {

			clientTurnOnMic.shutdown();
			clientTurnOnMic = null;

			System.out.println("  Shutdown service client '"
					+ SERVICE_NAME_TURN_ON
					+ "'. (type: "
					+ TurnOn._TYPE + ")");

		}

	}

	public boolean call() {

		boolean success = false;

		TurnOnRequest req = clientTurnOnMic.newMessage();

		synchronized (sync) {

			responsePending = true;

			clientTurnOnMic.call(req, new ServiceResponseListener<TurnOnResponse>() {

				@Override
				public void onSuccess(TurnOnResponse response) {

					synchronized (sync) {
						ok = response.getSuccess();
						responsePending = false;
						sync.notifyAll();
					}

				}

				@Override
				public void onFailure(RemoteException e) {

					synchronized (sync) {
						ok = false;
						responsePending = false;
						sync.notifyAll();
					}

				}

			});

			while (responsePending) {
				try {
					sync.wait();
				} catch (InterruptedException e1) {
				}
			}

			success = ok;

		}

		return success;

	}

}