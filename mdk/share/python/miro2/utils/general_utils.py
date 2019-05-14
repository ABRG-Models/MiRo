#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, see LICENSE in the
#	MDK root directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	MIRO Developer Kit in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of MIRO
#	Developer Kit to you; the license granted to you is not a
#	sale. This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of MIRO
#	Developer Kit.
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.



import numpy as np
import sys
import signal
import rospy
import threading
import time



def tau2lambda( tau, fS ):
    
    # handle invalid taus
    if tau <= 0:
        return 0
    
    # get decay per sample
    nsamp = tau * fS
    decay_per_samp = 1.0 / nsamp
    
    # handle sub-sample values
    if decay_per_samp > 1.0:
        decay_per_samp = 1.0
    
    # convert to lambda
    return 1.0 - decay_per_samp

def tau2gamma( tau, fS ):
    
    return 1.0 - tau2lambda( tau, fS )

def bit(x):

	return (1 << x)

def warning(msg):

	print "**** WARNING ****", msg

def error(msg):

	print "\n", "**** ERROR ****", msg, "\n"
	raise ValueError(msg)

def keyboard_interrupt_handler(exit, frame):

	rospy.loginfo('Keyboard interrupt')
	rospy.loginfo('Shutdown node')
	sys.exit(0)

def keyboard_interrupt_init():

	signal.signal(signal.SIGINT, keyboard_interrupt_handler)

class PerformanceTimer(object):

	def __init__(self):
		
		# Stages is the number of st
		self.stage = 0
		self.stage_ticks = {}
	
		# start scheduler
		self.thread = threading.Thread(target=self.display_thread, args=())
		self.thread.daemon = True

		# not started
		self.last_t = 0

	def start(self, step_name):
	
		self.step_name = step_name
		self.stage_ticks[step_name] = 0

		# get latest time in ms
		self.last_t = time.time()

		# start
		self.thread.start()

	def step(self, step_name):

		# skip if not yet started
		if self.last_t == 0:
			return

		# Store ticks from last step
		self.stage_ticks[self.step_name] += time.time() - self.last_t

		# set up next step
		if step_name not in self.stage_ticks.keys():
			self.stage_ticks[step_name] = 0

		self.step_name = step_name

		self.last_t = time.time()

	def push(self, step_name):

		# skip if not yet started
		if self.last_t == 0:
			return

		self.old_step_name = self.step_name
		self.step(step_name)

	def pop(self):

		# skip if not yet started
		if self.last_t == 0:
			return

		self.step(self.old_step_name)

	def display_thread(self):
		
		while True:
			time.sleep(1)
			self.display()

	def display(self):

		# sum of tick values
		s = sum(self.stage_ticks.values())
			
		if not s:
			return

		print('---- usage per second ----')
		for i in self.stage_ticks.keys():
			print('\t' + i + ' ' + str(round(self.stage_ticks[i] / s, 5)))




