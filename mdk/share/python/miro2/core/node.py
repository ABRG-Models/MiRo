#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, and a complete
#	definition of "The Software", see LICENSE in the MDK root
#	directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	"The Software" in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of "The
#	Software" to you; the license granted to you is not a sale.
#	This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of "The
#	Software".
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#	

import threading
import numpy as np
import time

# support
import miro2 as miro



class Node(object):

	def __init__(self, sys, node_name):

		# store
		self.pars = sys.pars
		self.kc_s = sys.kc_s
		self.kc_m = sys.kc_m
		self.input = sys.input
		self.state = sys.state
		self.output = sys.output
		self.nodes = sys.nodes

		# threads
		#self.threads = []
		#self.threads_continue = True

		# performance
		self.perf = miro.utils.PerformanceTimer()
		
		"""

	def start_thread(self, threadproc):

		try:

			# create thread
			thread = threading.Thread(target=threadproc, args=())

			# daemonize
			thread.daemon = True

			# store
			self.threads.append(thread)

			# start thread
			thread.start()

		except Exception as e:

			rospy.loginfo(e)

	def stop(self):

		self.threads_continue = False

	def main(self):

		# report
		print "entering main loop..."

		# spin on ROS until shutdown
		while not rospy.core.is_shutdown() and self.threads_continue:
			time.sleep(0.1)

		# report
		print "...exiting main loop"
		
				"""




