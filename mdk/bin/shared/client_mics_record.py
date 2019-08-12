#!/usr/bin/python

import rospy
import time
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

import std_msgs
#import scipy.io.wavfile as wavfile
import wave
################################################################

def error(msg):

	print(msg)
	sys.exit(0)

################################################################

class client_mics:

	def callback_mics(self, msg):

		# mag = (np.mean(np.abs(np.array(msg.data))) / 32768.0 * 100.0).astype('int')
		# print "received", len(msg.data), "samples with mean mag", mag, "%"

		# save microphone data to rolling buffer for later processing
		# reshape into 4 x 500 array
		data = np.asarray(msg.data)
		data = np.transpose(data.reshape((self.no_of_mics, 500)))
		data = np.flipud(data)

	# array is [ L, R, H, B] mics
	# add to top of buffer
                self.input_mics = np.vstack((data,self.input_mics[:119500,:]))
		mics = np.flipud(self.input_mics[:,0])
		print "left ear data", self.input_mics.shape
	# save wav file
#		wavfile.write("tmp/output.wav",44100,data)
#		np.savetxt('/home/miro/Documents/chennan/output', data)
		self.saveWave(mics)

	def saveWave(self,data):
		data = np.asarray(data,dtype=np.int16)
		print data
		f=wave.open(r"/home/miro/Documents/chennan/output3.wav","wb")
		f.setnchannels(1)
		f.setsampwidth(2)
		f.setframerate(20000)
	#	plt.plot(data)
	#	plt.show()
		f.writeframes(data)
		f.close()

	def loop(self):

		# loop
		while not rospy.core.is_shutdown():

			# sleep
			time.sleep(1)

	def __init__(self):

		# config

		self.no_of_mics = 4
		self.seconds = 5
                self.input_mics = np.zeros((self.seconds*40000, self.no_of_mics))

		# robot name
		topic_base = "/" + os.getenv("MIRO_ROBOT_NAME") + "/"

		# subscribe
		topic = topic_base + "sensors/mics"
		print ("subscribe", topic)
		self.sub_mics = rospy.Subscriber(topic, std_msgs.msg.Int16MultiArray, self.callback_mics)

if __name__ == "__main__":

	rospy.init_node("client_mics", anonymous=True)
	main = client_mics()
	main.loop()




