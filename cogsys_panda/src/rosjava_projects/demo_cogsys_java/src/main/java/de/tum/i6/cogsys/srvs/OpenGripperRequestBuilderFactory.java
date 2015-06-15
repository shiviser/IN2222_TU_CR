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

import gripper_control_srvs.OpenGripper;
import gripper_control_srvs.OpenGripperRequest;
import gripper_control_srvs.OpenGripperResponse;

import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.smerobotics.util.srv.ServiceRequestBuilderFactory;

public class OpenGripperRequestBuilderFactory extends ServiceRequestBuilderFactory {

	public static final String SERVICE_NAME_OPEN_GRIPPER = "gripper_control/open_gripper";
	private ServiceClient<OpenGripperRequest, OpenGripperResponse> clientOpenGripper;

	private final Object sync = new Object();
	private volatile boolean responsePending = false;

	private boolean ok = false;
	
	@Override
	public synchronized void addServiceClients(ConnectedNode connectedNode) {

		if (clientOpenGripper == null) {

			boolean found = false;

			while (!found) {

				try {

					clientOpenGripper = connectedNode.newServiceClient(
							SERVICE_NAME_OPEN_GRIPPER,
							OpenGripper._TYPE);

					found = true;

					System.out.println("    Found service '"
							+ SERVICE_NAME_OPEN_GRIPPER
							+ "'. (type: "
							+ OpenGripper._TYPE + ")");

				} catch (ServiceNotFoundException e) {

					System.out.println("  Waiting for service '"
							+ SERVICE_NAME_OPEN_GRIPPER
							+ "' to start up. (type: "
							+ OpenGripper._TYPE + ")");

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

		if (clientOpenGripper != null) {

			clientOpenGripper.shutdown();
			clientOpenGripper = null;

			System.out.println("  Shutdown service client '"
					+ SERVICE_NAME_OPEN_GRIPPER
					+ "'. (type: "
					+ OpenGripper._TYPE + ")");

		}

	}

	public boolean call() {

		boolean success = false;

		OpenGripperRequest req = clientOpenGripper.newMessage();

		synchronized (sync) {

			responsePending = true;

			clientOpenGripper.call(req, new ServiceResponseListener<OpenGripperResponse>() {

				@Override
				public void onSuccess(OpenGripperResponse response) {

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
