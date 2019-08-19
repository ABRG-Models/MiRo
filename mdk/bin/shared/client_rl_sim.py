#!/usr/bin/python

import rospy
import time
import sys
import os
import numpy as np
import datetime
import math

import std_msgs
import miro2 as miro
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import TwistStamped

################################################################

def error(msg):

	print(msg)
	sys.exit(0)


################################################################

class QLearningTable:
	def __init__(self, learning_rate=0.2, discount=0.9, e_greedy=0.9):
		self.lr = learning_rate
		self.gamma = discount
		self.epsilon = e_greedy

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
		state_adj = (pose - self.states_low) / self.scale
		return state_adj.astype(int)

	def choose_action(self, state):
		return np.argmax(self.Q_table[state[0]][state[1]][state[2]])
		# return np.argmax((1 - self.beta) * self.Q_table[state[0]][state[1]] + self.beta * self.E_table[state][0][state[1]])

	def learn(self, state, action, reward, new_state):
		self.Q_table[state[0]][state[1]][state[2]][action] += self.lr * (
					reward + self.gamma * np.max(self.Q_table[new_state[0]][new_state[1]][new_state[2]]) -
					self.Q_table[state[0]][state[1]][state[2]][action])



################################################################

class client_Qlearning:

	def callback_package(self, msg):

		self.sonar = msg.sonar.range
		# print "sonar", self.sonar
		if self.sonar < 0.2:
			self.velocity.twist.linear.x = 0.0
			self.velocity.twist.angular.z = 0.0
			self.pub_cmd_vel.publish(self.velocity)
		# 	print(self.sonar)
		# 	self.action_down()


	def callback_pose(self, msg):

		self.pose = np.array([msg.x, msg.y, msg.theta])
		# print "received pose", self.pose

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			f = lambda x, min_x: max(min_x, min(1.0, 1.0 - np.log10((x + 1) / 25.0)))

			for i in range(self.episode):
				print "i", i

				done = False
				state = self.pose
				# Discretize state
				state_adj = self.Q.discretize_state(state)

				step=0

				while done!=True:
					if (np.random.random() < self.Q.epsilon):
						action = self.action_space_sample()
					else:
						action = self.Q.choose_action(state_adj)

					if self.sonar < 0.13:
						action = 1

					# Get next state and reward
					state2_adj, reward, done = self.step(action)

					if self.sonar < 0.13:
						reward = -5
						done = True

					if step > 100:
						done = True

					# Allow for terminal states
					if done:
						self.Q.Q_table[state_adj[0], state_adj[1],state_adj[2], action] = reward
					# Adjust Q value for current state
					else:
						self.Q.learn(state_adj, action, reward, state2_adj)

					state_adj = state2_adj
					step +=1
					print "steps", step

				self.Q.epsilon = f(i,0.1)
				#print "steps", step

			# sleep
			time.sleep(0.01)

	def __init__(self):

		# config
		self.pose = np.array([0, 0, 0])
		self.sonar = 0.58

		self.velocity = TwistStamped()
		self.action_space = ['UP', 'DOWN', 'LEFT', 'RIGHT']
		self.goal = np.array([5,5,3])
		self.episode = 100
		self.Q = QLearningTable()

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subscribe
		self.pub_cmd_vel = rospy.Publisher(topic_base + "control/cmd_vel", TwistStamped, queue_size=0)

		# subscribe
		self.sub_package = rospy.Subscriber(topic_base + "sensors/package", miro.msg.sensors_package, self.callback_package)
		print ("subscribe", topic_base + "sensors/package")
		self.sub_pose = rospy.Subscriber(topic_base + "sensors/body_pose", Pose2D, self.callback_pose)
		print ("subscribe", topic_base + "sensors/body_pose")

	def action_space_sample(self):
		action = np.random.choice(self.action_space)
		return self.action_space.index(action)

	def step(self, action_i):
		action = self.action_space[action_i]
		print "action", action
		# if self.sonar < 0.2:
		# 	print(self.sonar)
		# 	action = 'DOWN'
		# give the speed to go 0.5m and stop
		if action == 'UP':
			self.action_up()

		# turn 180 degree then give the speed to go 0.5m and stop
		elif action == 'DOWN':
			self.action_down()

		# turn -90 degree then give the speed to go 0.5m and stop
		elif action == 'LEFT':
			self.action_left()

		# turn 90 degree then give the speed to go 0.5m and stop
		else:
			self.action_right()

		new_state = self.Q.discretize_state(self.pose)
		# reach the goal have 1, else -1 reward
		if new_state[0] == self.goal[0] and new_state[1] == self.goal[1] and new_state[2] == self.goal[2]:
			reward = 1
			done = True
		else:
			reward = -1
			done = False

		return new_state, reward, done

	def reset(self):
		return 0

	def action_up(self):
		start = datetime.datetime.now()

		while (True):
			end = datetime.datetime.now()

			if (end - start).seconds > 4:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)
				break
			# time.sleep(1)
			else:
				self.velocity.twist.linear.x = 0.1
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)

	def action_down(self):
		start = datetime.datetime.now()

		while (True):
			end = datetime.datetime.now()
			if (end - start).seconds > 1:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)
				break
			else:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = 1.57
				self.pub_cmd_vel.publish(self.velocity)

		start = datetime.datetime.now()

		while (True):
			end = datetime.datetime.now()

			if (end - start).seconds > 4:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)
				break
			# time.sleep(1)
			else:
				self.velocity.twist.linear.x = 0.1
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)

	def action_left(self):
		start = datetime.datetime.now()

		while (True):
			end = datetime.datetime.now()
			if (end - start).seconds > 0:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)
				break
			else:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = 1.57
				self.pub_cmd_vel.publish(self.velocity)

		start = datetime.datetime.now()

		while (True):
			end = datetime.datetime.now()

			if (end - start).seconds > 4:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)
				break
			# time.sleep(1)
			else:
				self.velocity.twist.linear.x = 0.1
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)

	def action_right(self):
		start = datetime.datetime.now()

		while (True):
			end = datetime.datetime.now()
			if (end - start).seconds > 0:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)
				break
			else:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = -1.57
				self.pub_cmd_vel.publish(self.velocity)

		start = datetime.datetime.now()

		while (True):
			end = datetime.datetime.now()

			if (end - start).seconds > 4:
				self.velocity.twist.linear.x = 0.0
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)
				break
			# time.sleep(1)
			else:
				self.velocity.twist.linear.x = 0.1
				self.velocity.twist.angular.z = 0.0
				self.pub_cmd_vel.publish(self.velocity)

if __name__ == "__main__":

	rospy.init_node("client_Qlearning", anonymous=True)
	main = client_Qlearning()
	main.loop()



