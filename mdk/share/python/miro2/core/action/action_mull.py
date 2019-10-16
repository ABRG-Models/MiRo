# @section COPYRIGHT
# Copyright (C) 2019 Consequential Robotics Ltd
# 
# @section AUTHOR
# Consequential Robotics http://consequentialrobotics.com
# 
# @section LICENSE
# For a full copy of the license agreement, and a complete
# definition of "The Software", see LICENSE in the MDK root
# directory.
# 
# Subject to the terms of this Agreement, Consequential
# Robotics grants to you a limited, non-exclusive, non-
# transferable license, without right to sub-license, to use
# "The Software" in accordance with this Agreement and any
# other written agreement with Consequential Robotics.
# Consequential Robotics does not transfer the title of "The
# Software" to you; the license granted to you is not a sale.
# This agreement is a binding legal agreement between
# Consequential Robotics and the purchasers or users of "The
# Software".
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
# KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
# OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# 
#
#	Action Mull
#	- Do nothing

import numpy as np
import miro2 as miro

from action_types import ActionTemplate



class ActionMull(ActionTemplate):

	def finalize(self):

		# parameters
		self.name = "mull"
		self.modulate_by_wakefulness = False # mull is default when sleepy

	def ascending(self):

		# fixed priority
		self.set_priority(self.pars.action.priority_idle)

		# except during user touch
		if self.input.user_touch:

			# if not selected
			if self.get_inhibition():
				self.set_priority(self.pars.action.priority_high)

			# if selected make uninterruptable until user_touch ends
			else:
				self.set_priority(self.pars.action.priority_uninterruptable)

	def descending(self):

		pass

	def debug_event_start(self):

		pass



