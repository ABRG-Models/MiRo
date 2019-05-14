#!/usr/bin/python

from detect_audio_engine import *

import pickle
import cProfile

class ParsDA:

	def __init__(self):

		self.inter_ear_distance = 0.104 # metres
		self.raw_magnitude_thresh = 0.01 # normalized; audio event processing skipped unless over thresh
		self.assumed_sound_source_height = 1.0 # metres
		self.assumed_sound_source_range = 1.5 # metres

		speed_of_sound = 343.0 # m/s
		self.inter_ear_lag = self.inter_ear_distance / speed_of_sound * 20000

class Pars:

	def __init__(self):

		self.x = 1
		self.detect_audio = ParsDA()

pars = Pars()

obj = DetectAudioEngine(pars, None)

file = open(r'mics_data', 'rb')
datas = pickle.load(file)
file.close()

file = open(r'mics_X', 'rb')
X_expected = pickle.load(file)
file.close()

def PerfTest():

	for j in range(100):
		if (j % 10) == 0:
			print "rep", j
		X = []
		for i in range(len(datas)):
			data = datas[i]
			msg = obj.process_data(data)
			if not msg is None:
				x = [msg.azim, msg.level]
				X.append(x)

	return X

cProfile.run('X = PerfTest()')

if True:
	if X != X_expected:
		print "bad results"
else:
	file = open(r'mics_X', 'wb')
	pickle.dump(X, file)
	file.close()


	"""
	100 * 100:
	real	0m4.793s
	user	0m4.688s
	sys		0m0.296s
	"""


