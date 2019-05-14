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

import numpy as np
import time
import os

import node



class NodeLower(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "lower")

		# touch state
		self.touch_body = None
		self.touch_body_com_st = [0.0, 0.0, 0.0]
		self.touch_body_com_ed = [0.0, 0.0, 0.0]
		self.touch_head = None
		self.touch_head_bak = 0

		# jerk state
		self.imu_head = None
		self.imu_body = None
		self.imu_head_bak = None
		self.imu_body_bak = None

	def process_stroke(self, i, line):

		# each "line" of sensors has a start bit, a number of bits, and a
		# layout direction (thus, all "stroke lines" must be represented
		# by consecutive bits in "touch_body")
		bit0 = line[0]
		bitn = line[1]
		pol = line[2]

		# compute centre of mass
		x = np.unpackbits(np.uint8((self.touch_body >> bit0) & (2 ** bitn - 1)))
		s = np.sum(x)
		if s > 0:
			com = np.dot(x, np.array([8, 7, 6, 5, 4, 3, 2, 1])) / np.sum(x)
		else:
			com = 0.0

		# if no com is stored for the start of a stroke
		if self.touch_body_com_st[i] == 0.0:

			# wait for one to arrive and store it
			self.touch_body_com_st[i] = com

			# no stroke at present
			return 0.0

		# otherwise, wait for release
		elif com == 0.0:

			# and then do the compare
			dcom = self.touch_body_com_ed[i] - self.touch_body_com_st[i]
			self.touch_body_com_st[i] = 0.0

			# gain with polarity
			stroke = dcom * pol

			# return that as stroke value
			return stroke

		# otherwise
		else:

			# update stored
			self.touch_body_com_ed[i] = com

			# no stroke at present
			return 0.0

	def process_touch(self):

		# process stroke
		self.state.stroke = 0.0
		for i, line in enumerate(self.pars.lower.stroke_lines):
			self.state.stroke += self.process_stroke(i, line)

		# process pet
		x = self.touch_head^self.touch_head_bak
		self.touch_head_bak = self.touch_head
		self.state.pet = bin(x).count("1") # count bits that have changed

		# user touch is based purely on number of sensors hit
		n = bin(self.touch_head).count("1") + bin(self.touch_body).count("1")
		user_touch_target = self.pars.lower.user_touch_gain * n
		if user_touch_target > 0.0:
			self.state.user_touch += self.pars.lower.user_touch_gamma_attack * (user_touch_target-self.state.user_touch)
		else:
			self.state.user_touch += self.pars.lower.user_touch_gamma_release * (user_touch_target-self.state.user_touch)
		if self.state.user_touch < self.pars.lower.user_touch_min:
			self.state.user_touch = 0.0

	def compute_jerk(self, acc0, acc1):

		x = acc0.x - acc1.x
		y = acc0.y - acc1.y
		z = acc0.z - acc1.z

		# compute modulus
		mod = np.sqrt(np.square(x) + np.square(y) + np.square(z))

		# ok
		return mod

	def compute_jerks(self):

		if not self.imu_head_bak is None:

			# compute jerk
			self.state.jerk_head = self.compute_jerk(self.imu_head, self.imu_head_bak)
			self.state.jerk_body = self.compute_jerk(self.imu_body, self.imu_body_bak)

		# store
		self.imu_head_bak = self.imu_head
		self.imu_body_bak = self.imu_body

	def tick(self):

		self.perf.step('tick')

		# process sensors_package
		msg = self.input.sensors_package

		# light sum
		self.state.light_mean = np.mean(msg.light.data)

		# imu head
		self.imu_head = msg.imu_head.linear_acceleration
		self.imu_body = msg.imu_body.linear_acceleration

		# store
		self.touch_head = msg.touch_head.data & self.pars.lower.touch_head_mask
		self.touch_body = msg.touch_body.data & self.pars.lower.touch_body_mask

		# compute
		self.compute_jerks()
		self.process_touch()

		# wait
		self.perf.step('wait')



