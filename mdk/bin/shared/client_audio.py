#!/usr/bin/python
#
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

import rospy
from std_msgs.msg import UInt16MultiArray, Int16MultiArray

import time
import sys
import os
import numpy as np
import wave, struct

# amount to keep the buffer stuffed - larger numbers mean
# less prone to dropout, but higher latency when we stop
# streaming. with a read-out rate of 8k, 4000 samples will
# buffer for half of a second, for instance.
BUFFER_STUFF_SAMPLES = 4000

# messages larger than this will be dropped by the receiver,
# however, so - whilst we can stuff the buffer more than this -
# we can only send this many samples in any single message.
MAX_STREAM_MSG_SIZE = (4096 - 48)

# using a margin avoids sending many small messages - instead
# we will send a smaller number of larger messages, at the cost
# of being less precise in respecting our buffer stuffing target.
BUFFER_MARGIN = 1000
BUFFER_MAX = BUFFER_STUFF_SAMPLES + BUFFER_MARGIN
BUFFER_MIN = BUFFER_STUFF_SAMPLES - BUFFER_MARGIN

# how long to record before playing back in seconds?
RECORD_TIME = 2

# microphone sample rate (also available at miro2.constants)
MIC_SAMPLE_RATE = 20000

# sample count
SAMPLE_COUNT = RECORD_TIME * MIC_SAMPLE_RATE



################################################################

def error(msg):
	print(msg)
	sys.exit(0)

################################################################

# if no argument provided
if len(sys.argv) != 2:

	# show usage
	print "pass one of the following arguments:"
	print "\trecord: record stereo audio (ear mics) and store to /tmp/client_audio.wav"
	print "\trecord4: record all four mics (centre and tail mics as well as ear mics)"
	print "\techo: record audio then stream back to platform immediately"

	# done
	exit()
	
	
################################################################

class client:

	def callback_stream(self, msg):

		self.buffer_space = msg.data[0]
		self.buffer_total = msg.data[1]
		self.buffer_stuff = self.buffer_total - self.buffer_space
		
	def callback_mics(self, msg):
	
		# if recording
		if not self.micbuf is None:

			# append mic data to store
			data = np.array(msg.data, 'int16')
			x = np.reshape(data, (-1, 500))
			self.micbuf = np.concatenate((self.micbuf, x.T))
			
			# report
			sys.stdout.write(".")
			sys.stdout.flush()
		
			# finished recording?
			if self.micbuf.shape[0] >= SAMPLE_COUNT:
			
				# end recording
				self.outbuf = self.micbuf
				self.micbuf = None
				print " OK!"
		
	def loop(self):

		# loop
		while not rospy.core.is_shutdown():
		
			# if recording finished
			if not self.outbuf is None:
				break

			# state
			time.sleep(0.02)
			
		# if echo
		if self.mode == "echo":
		
			# downsample for playback
			outbuf = np.zeros((int(SAMPLE_COUNT / 2.5), 0))
			for c in range(4):
				i = np.arange(0, SAMPLE_COUNT, 2.5)
				j = np.arange(0, SAMPLE_COUNT)
				x = np.interp(i, j, self.outbuf[:, c])
				outbuf = np.concatenate((outbuf, x[:, np.newaxis]), axis=1)
				
			# channel names
			chan = ["LEFT", "RIGHT", "CENTRE", "TAIL"]
		
			# loop
			while not rospy.core.is_shutdown():
		
				# stuff output buffer
				if self.buffer_stuff < BUFFER_MIN:
		
					# report
					if self.playsamp == 0:
	
						# report
						print "echo recording from " + chan[self.playchan] + " microphone..."
			
					# desired amount to send
					n_samp = BUFFER_MAX - self.buffer_stuff
			
					# limit by receiver buffer space
					n_samp = np.minimum(n_samp, self.buffer_space)

					# limit by amount available to send
					n_samp = np.minimum(n_samp, outbuf.shape[0] - self.playsamp)

					# limit by maximum individual message size
					n_samp = np.minimum(n_samp, MAX_STREAM_MSG_SIZE)
			
					# prepare data
					spkrdata = outbuf[self.playsamp:(self.playsamp+n_samp), self.playchan]
					self.playsamp += n_samp
			
					# send data
					msg = Int16MultiArray()
					msg.data = spkrdata
					self.pub_stream.publish(msg)
			
					# update buffer_stuff so that we don't send
					# again until we get some genuine feedback
					self.buffer_stuff = BUFFER_MIN
			
					# finished?
					if self.playsamp == outbuf.shape[0]:
			
						# move to next channel
						self.playsamp = 0
						self.playchan += 1
				
						# finished?
						if self.playchan == 4:
				
							# clear output
							print "(playback complete)"
							break

				# state
				time.sleep(0.02)
						
		# if record
		if self.mode == "record" or self.mode == "record4":
		
			# write output file
			outfilename = '/tmp/client_audio.wav'
			file = wave.open(outfilename, 'wb')
			file.setsampwidth(2)
			file.setframerate(MIC_SAMPLE_RATE)
	
			# write data
			if self.mode == "record4":
				print "writing all four channels to file..."
				file.setnchannels(4)
				x = np.reshape(self.outbuf[:, :], (-1))
				for s in x:
					file.writeframes(struct.pack('<h', s))
			else:
				print "writing two channels to file (LEFT and RIGHT)..."
				file.setnchannels(2)
				x = np.reshape(self.outbuf[:, [0, 1]], (-1))
				for s in x:
					file.writeframes(struct.pack('<h', s))
			
			# close file
			file.close()
			print "wrote output file at", outfilename

	def __init__(self, mode):

		# state
		self.micbuf = np.zeros((0, 4), 'uint16')
		self.outbuf = None
		self.buffer_stuff = 0
		self.mode = mode
		self.playchan = 0
		self.playsamp = 0
		
		# check mode
		if not (mode == "echo" or mode == "record" or mode == "record4"):
			error("argument not recognised")

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = topic_base_name + "/control/stream"
		print ("publish", topic)
		self.pub_stream = rospy.Publisher(topic, Int16MultiArray, queue_size=0)

		# subscribe
		topic = topic_base_name + "/sensors/stream"
		print ("subscribe", topic)
		self.sub_stream = rospy.Subscriber(topic, UInt16MultiArray, self.callback_stream, queue_size=1, tcp_nodelay=True)

		# subscribe
		topic = topic_base_name + "/sensors/mics"
		print ("subscribe", topic)
		self.sub_mics = rospy.Subscriber(topic, Int16MultiArray, self.callback_mics, queue_size=5, tcp_nodelay=True)
		
		# report
		print "recording from 4 microphones for", RECORD_TIME, "seconds..."

if __name__ == "__main__":

	rospy.init_node("client_audio", anonymous=True)
	main = client(sys.argv[1])
	main.loop()




