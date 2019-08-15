#!/usr/bin/python

import rospy
import time
import sys
import os
import numpy as np
import datetime
import std_msgs
import miro2 as miro
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TwistStamped

################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class Environment:
	def __init__(self):
		self.action_space = ['UP', 'DOWN', 'LEFT', 'RIGHT']
		self.goal=[]
		self.pose=[]

	def action_space_sample(self):
		action = np.random.choice(self.action_space)
		return self.action_space.index(action)

	def step(self, action_i):
		action = self.action_space[action_i]
		# give the speed to go 0.5m and stop
		if action=='UP':
			self.action_up()
			return 0

		# turn 180 degree then give the speed to go 0.5m and stop
		elif action=='DOWN':
			self.action_down()
			return 0

		# turn -90 degree then give the speed to go 0.5m and stop
		elif action=='LEFT':
			self.action_left()
			return 0

		# turn 90 degree then give the speed to go 0.5m and stop
		else:
			self.action_right()
			return 0

		# reach the goal have 1, else -1 reward
		if self.pose==self.goal:
			reward = 1
			done = True
		else:
			reward = -1
			done = False

		return self.pose, reward, done

	def reset(self):
		return

	def action_up(self):
		start = datetime.datetime.now()

		while(True):
			self.velocity.twist.linear.x = 0.1



	def action_down(self):

	def action_left(self):

	def action_right(self):

################################################################

class QLearningTable:
	def __init__(self, actions, learning_rate=0.2, discount=0.9, e_greedy=0.9):
		self.actions = actions  # a list
		self.lr = learning_rate
		self.gamma = discount
		self.epsilon = e_greedy

		self.env = Environment()
		self.num_states = np.array([6, 6, 4])
		self.states_low = np.array([-1.5,-1.5,-3.14])
		self.states_high = np.array([1.5,1.5,3.14])
		self.scale = np.array([0.5, 0.5, 1.57])
		self.num_actions = 4
		self.Q_table = np.zeros((self.num_states[0], self.num_states[1],self.num_states[2], self.num_actions))

	def print_Tabel(self):
		print('Q',self.Q_table)
		# print('E',self.E_table)

	def discretize_state(self, pose):
		state_adj = int((pose - self.states_low) / self.scale)
		return state_adj.astype(int)

	def choose_action(self, state):
		# print(state)
	    if (np.random.random() < self.epsilon):
			return self.env.action_space_sample()
		else:
			return np.argmax(self.Q_table[state[0]][state[1]][state[2]])
		# return np.argmax((1 - self.beta) * self.Q_table[state[0]][state[1]] + self.beta * self.E_table[state][0][state[1]])

	def learn(self, s, a, r, s_):
		self.check_state_exist(s_)
		q_predict = self.q_table.loc[s, a]
		if s_ != 'terminal':
			q_target = r + self.gamma * self.q_table.loc[s_, :].max()  # next state is not terminal
		else:
			q_target = r  # next state is terminal
		self.q_table.loc[s, a] += self.lr * (q_target - q_predict)  # update

	def check_state_exist(self, state):
		if state not in self.q_table.index:
			# append new state to q table
			self.q_table = self.q_table.append(
				pd.Series(
					[0] * len(self.actions),
					index=self.q_table.columns,
					name=state,
				)
			)






################################################################

class client_Qlearning:

	def callback_package(self, msg):

		x = msg.sonar.range

		print "sonar", x


	def callback_pose(self, msg):

		self.pose = msg
		print "received pose", self.pose

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# sleep
			time.sleep(0.01)

	def __init__(self):

		# config
		self.pose = []
		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subscribe
		self.sub_package = rospy.Subscriber(topic_base + "sensors/package", miro.msg.sensors_package, self.callback_package)
		print ("subscribe", topic_base + "sensors/package")
		self.sub_pose = rospy.Subscriber(topic_base + "sensors/body_pose", Pose2D, self.callback_pose)
		print ("subscribe", topic_base + "sensors/body_pose")



if __name__ == "__main__":

	rospy.init_node("client_Qlearning", anonymous=True)
	main = client_Qlearning()
	main.loop()




