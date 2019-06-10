import numpy as np
import miro2 as miro

from action_types import ActionTemplate

# MiRo's actions all derive from the ActionTemplate object defined in action_types.py
# You can override the class methods, leave them as they are, or define new ones if desired
# However, the methods 'finalize', 'compute_priority', 'start', and 'service' must be defined for each action
# (Not *strictly* true! See action_mull.py. Why does ActionMull define different methods? Looking at node_action.py may help)
# action_retreat.py and action_flee.py are good examples of actions that only define these four methods
# action_approach.py and action_orient.py are a little more complicated
# action_halt.py is a lot more complicated

class ActionBlank(ActionTemplate):
	
	def finalize(self):
		
		# Set initial status
		self.name = "blank"

	def compute_priority(self):

		# Compute and return action priority (i.e. salience)
	
	def start(self):

		# Define start of motor program

	def service(self):

		# Define continuation of motor program
