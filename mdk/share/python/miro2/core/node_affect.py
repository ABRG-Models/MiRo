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
import datetime

import node
import miro2 as miro
import signals



class RTC(object):

	def __init__( self, pars ):

		# store pars
		self.pars = pars

		"""
		# initialize to 12:00 -> noon
		self.midnight = time.time() - 12*60*60

		# Setup tables
		self.setup_arousal_table()
		self.setup_skew_table()

		# Misc constants
		self.skew_lim = 1 << 30
		self.skew_gain = 524288
		self.skew_max = 1 << 28
		self.skew_accum = 0
		self.skew_secs = 0
		"""

	def get_time_of_day( self ):

		# get normalised time (0.0 to 1.0 during day)
		now = datetime.datetime.now()
		t = (now.hour * 60.0 + now.minute) / 1440.0
		return t

		"""
	def get_time( self ):
		'''
			Returns time relative to our 'midnight'
		'''
		oneday = 86400 # no of seconds in 1 day
		currtime = time.time()
		# In midnight hs not updated to present day
		while currtime - self.midnight > oneday:
			self.midnight += 86400

		currtime -= self.midnight

		hrs = int(currtime / 3600)
		mins = int( ( currtime - hrs*3600) / 60 )
		secs = int( currtime - hrs*3600 - mins*60 )

		return hrs, mins, secs

	def ascending( self, light_sum ):
		'''
			Gets arousal based on RTC
		'''
		# get curr time
		hrs, mins, secs = self.get_time()

		if self.pars.flags.AFFECT_ADJUST_RTC:

			t_combined = hrs*60 + mins

			# Get base arousal between -127 and +127
			self.arousal = self.table_arousal[t_combined]
			#scale to be between -1 and +1
			self.arousal *= (1.0/127.0)

			# Get skew between -127 and +127
			skew = self.table_skew[t_combined]
			light_level = light_sum * self.skew_gain
			dskew = skew * light_level

			# Clip
			dskew = np.clip( dskew, -self.skew_max, self.skew_max )

			# Accumulate
			self.skew_accum += dskew


			if self.skew_accum > self.skew_lim:
				#accept skew

				#if miro.P2_RTC_ZERO_SKEW:
				#	self.skew_accum = 0
				#else:
				self.skew_accum -= self.skew_lim

				# apply skew
				self.skew_secs += 1

			if self.skew_accum < -self.skew_lim:
				#if miro.P2_RTC_ZERO_SKEW:
				#	self.skew_accum = 0
				#else:
				self.skew_accum += self.skew_lim

				self.skew_secs -= 1

			# Then we do physical skew updates only when clear of rollovers,
			# so there is no risk of rollover of anything while we are making changes

			if secs < 55 and mins > 0 and mins < 59:

				# implement skew
				if self.skew_secs >= 60:
					self.skew_secs -= 60
					self.set_time( hrs, mins+1, secs )

				if self.skew_secs <= -60:
					self.skew_secs += 60
					self.set_time( hrs, mins-1, secs )

			# send upstream? not sure where to

	def setup_arousal_table( self ):

		self.table_arousal = [
				-124, -124, -124, -124, -124, -124, -124, -124,
				-124, -125, -125, -125, -125, -125, -125, -125,
				-125, -125, -125, -125, -125, -125, -125, -125,
				-125, -125, -125, -125, -125, -125, -125, -125,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -127, -127,
				-127, -127, -127, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -126, -126, -126, -126, -126, -126, -126,
				-126, -125, -125, -125, -125, -125, -125, -125,
				-125, -125, -125, -125, -125, -125, -125, -125,
				-125, -125, -125, -125, -125, -125, -125, -125,
				-124, -124, -124, -124, -124, -124, -124, -124,
				-124, -124, -124, -124, -124, -124, -124, -124,
				-123, -123, -123, -123, -123, -123, -123, -123,
				-123, -123, -123, -122, -122, -122, -122, -122,
				-122, -122, -122, -122, -121, -121, -121, -121,
				-121, -121, -121, -120, -120, -120, -120, -120,
				-120, -120, -119, -119, -119, -119, -119, -118,
				-118, -118, -118, -118, -117, -117, -117, -117,
				-117, -116, -116, -116, -116, -115, -115, -115,
				-114, -114, -114, -114, -113, -113, -113, -112,
				-112, -112, -111, -111, -110, -110, -110, -109,
				-109, -108, -108, -108, -107, -107, -106, -106,
				-105, -105, -104, -104, -103, -103, -102, -102,
				-101, -100, -100, -99, -99, -98, -97, -97,
				-96, -95, -95, -94, -93, -92, -92, -91,
				-90, -89, -88, -87, -87, -86, -85, -84,
				-83, -82, -81, -80, -79, -78, -77, -76,
				-75, -74, -73, -72, -71, -70, -68, -67,
				-66, -65, -64, -62, -61, -60, -58, -57,
				-56, -55, -53, -52, -50, -49, -48, -46,
				-45, -43, -42, -40, -39, -37, -36, -34,
				-33, -31, -29, -28, -26, -25, -23, -22,
				-20, -18, -17, -15, -13, -12, -10, -8,
				-7, -5, -3, -2, 0, 2, 3, 5,
				7, 8, 10, 12, 13, 15, 17, 18,
				20, 22, 23, 25, 26, 28, 29, 31,
				33, 34, 36, 37, 39, 40, 42, 43,
				45, 46, 48, 49, 50, 52, 53, 55,
				56, 57, 58, 60, 61, 62, 64, 65,
				66, 67, 68, 70, 71, 72, 73, 74,
				75, 76, 77, 78, 79, 80, 81, 82,
				83, 84, 85, 86, 87, 87, 88, 89,
				90, 91, 92, 92, 93, 94, 95, 95,
				96, 97, 97, 98, 99, 99, 100, 100,
				101, 102, 102, 103, 103, 104, 104, 105,
				105, 106, 106, 107, 107, 108, 108, 108,
				109, 109, 110, 110, 110, 111, 111, 112,
				112, 112, 113, 113, 113, 114, 114, 114,
				114, 115, 115, 115, 116, 116, 116, 116,
				117, 117, 117, 117, 117, 118, 118, 118,
				118, 118, 119, 119, 119, 119, 119, 120,
				120, 120, 120, 120, 120, 120, 121, 121,
				121, 121, 121, 121, 121, 122, 122, 122,
				122, 122, 122, 122, 122, 122, 123, 123,
				123, 123, 123, 123, 123, 123, 123, 123,
				123, 124, 124, 124, 124, 124, 124, 124,
				124, 124, 124, 124, 124, 124, 124, 124,
				124, 125, 125, 125, 125, 125, 125, 125,
				125, 125, 125, 125, 125, 125, 125, 125,
				125, 125, 125, 125, 125, 125, 125, 125,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 127, 127, 127,
				127, 127, 127, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 126, 126, 126, 126, 126, 126, 126,
				126, 125, 125, 125, 125, 125, 125, 125,
				125, 125, 125, 125, 125, 125, 125, 125,
				125, 125, 125, 125, 125, 125, 125, 125,
				124, 124, 124, 124, 124, 124, 124, 124,
				124, 124, 124, 124, 124, 124, 124, 124,
				123, 123, 123, 123, 123, 123, 123, 123,
				123, 123, 123, 122, 122, 122, 122, 122,
				122, 122, 122, 122, 121, 121, 121, 121,
				121, 121, 121, 120, 120, 120, 120, 120,
				120, 120, 119, 119, 119, 119, 119, 118,
				118, 118, 118, 118, 117, 117, 117, 117,
				117, 116, 116, 116, 116, 115, 115, 115,
				114, 114, 114, 114, 113, 113, 113, 112,
				112, 112, 111, 111, 110, 110, 110, 109,
				109, 108, 108, 108, 107, 107, 106, 106,
				105, 105, 104, 104, 103, 103, 102, 102,
				101, 100, 100, 99, 99, 98, 97, 97,
				96, 95, 95, 94, 93, 92, 92, 91,
				90, 89, 88, 87, 87, 86, 85, 84,
				83, 82, 81, 80, 79, 78, 77, 76,
				75, 74, 73, 72, 71, 70, 68, 67,
				66, 65, 64, 62, 61, 60, 58, 57,
				56, 55, 53, 52, 50, 49, 48, 46,
				45, 43, 42, 40, 39, 37, 36, 34,
				33, 31, 29, 28, 26, 25, 23, 22,
				20, 18, 17, 15, 13, 12, 10, 8,
				7, 5, 3, 2, 0, -2, -3, -5,
				-7, -8, -10, -12, -13, -15, -17, -18,
				-20, -22, -23, -25, -26, -28, -29, -31,
				-33, -34, -36, -37, -39, -40, -42, -43,
				-45, -46, -48, -49, -50, -52, -53, -55,
				-56, -57, -58, -60, -61, -62, -64, -65,
				-66, -67, -68, -70, -71, -72, -73, -74,
				-75, -76, -77, -78, -79, -80, -81, -82,
				-83, -84, -85, -86, -87, -87, -88, -89,
				-90, -91, -92, -92, -93, -94, -95, -95,
				-96, -97, -97, -98, -99, -99, -100, -100,
				-101, -102, -102, -103, -103, -104, -104, -105,
				-105, -106, -106, -107, -107, -108, -108, -108,
				-109, -109, -110, -110, -110, -111, -111, -112,
				-112, -112, -113, -113, -113, -114, -114, -114,
				-114, -115, -115, -115, -116, -116, -116, -116,
				-117, -117, -117, -117, -117, -118, -118, -118,
				-118, -118, -119, -119, -119, -119, -119, -120,
				-120, -120, -120, -120, -120, -120, -121, -121,
				-121, -121, -121, -121, -121, -122, -122, -122,
				-122, -122, -122, -122, -122, -122, -123, -123,
				-123, -123, -123, -123, -123, -123, -123, -123,
				-123, -124, -124, -124, -124, -124, -124, -124
			]

	def setup_skew_table( self ):

		self.table_skew = [
				-50, -50, -50, -49, -49, -49, -49, -48,
				-48, -48, -47, -47, -47, -46, -46, -46,
				-46, -45, -45, -45, -44, -44, -44, -44,
				-43, -43, -43, -42, -42, -42, -42, -41,
				-41, -41, -41, -40, -40, -40, -39, -39,
				-39, -39, -38, -38, -38, -38, -37, -37,
				-37, -36, -36, -36, -36, -35, -35, -35,
				-35, -34, -34, -34, -33, -33, -33, -33,
				-32, -32, -32, -32, -31, -31, -31, -30,
				-30, -30, -30, -29, -29, -29, -29, -28,
				-28, -28, -27, -27, -27, -27, -26, -26,
				-26, -26, -25, -25, -25, -24, -24, -24,
				-24, -23, -23, -23, -23, -22, -22, -22,
				-21, -21, -21, -21, -20, -20, -20, -19,
				-19, -19, -19, -18, -18, -18, -18, -17,
				-17, -17, -16, -16, -16, -16, -15, -15,
				-15, -14, -14, -14, -14, -13, -13, -13,
				-12, -12, -12, -12, -11, -11, -11, -11,
				-10, -10, -10, -9, -9, -9, -9, -8,
				-8, -8, -7, -7, -7, -7, -6, -6,
				-6, -5, -5, -5, -5, -4, -4, -4,
				-3, -3, -3, -3, -2, -2, -2, -1,
				-1, -1, -1, 0, 0, 0, 1, 1,
				1, 1, 2, 2, 2, 3, 3, 3,
				3, 4, 4, 4, 5, 5, 5, 5,
				6, 6, 6, 7, 7, 7, 7, 8,
				8, 8, 9, 9, 9, 9, 10, 10,
				10, 11, 11, 11, 11, 12, 12, 12,
				12, 13, 13, 13, 14, 14, 14, 14,
				15, 15, 15, 16, 16, 16, 16, 17,
				17, 17, 18, 18, 18, 18, 19, 19,
				19, 19, 20, 20, 20, 21, 21, 21,
				21, 22, 22, 22, 23, 23, 23, 23,
				24, 24, 24, 24, 25, 25, 25, 26,
				26, 26, 26, 27, 27, 27, 27, 28,
				28, 28, 29, 29, 29, 29, 30, 30,
				30, 30, 31, 31, 31, 32, 32, 32,
				32, 33, 33, 33, 33, 34, 34, 34,
				35, 35, 35, 35, 36, 36, 36, 36,
				37, 37, 37, 38, 38, 38, 38, 39,
				39, 39, 39, 40, 40, 40, 41, 41,
				41, 41, 42, 42, 42, 42, 43, 43,
				43, 44, 44, 44, 44, 45, 45, 45,
				46, 46, 46, 46, 47, 47, 47, 48,
				48, 48, 49, 49, 49, 49, 50, 50,
				50, 51, 51, 51, 52, 52, 52, 53,
				53, 53, 54, 54, 54, 55, 55, 55,
				56, 56, 56, 57, 57, 57, 58, 58,
				58, 59, 59, 60, 60, 60, 61, 61,
				62, 62, 62, 63, 63, 64, 64, 64,
				65, 65, 66, 66, 67, 67, 68, 68,
				69, 69, 70, 70, 71, 71, 72, 72,
				73, 73, 74, 74, 75, 75, 76, 77,
				77, 78, 78, 79, 80, 80, 81, 81,
				82, 83, 83, 84, 85, 85, 86, 87,
				87, 88, 89, 90, 90, 91, 92, 92,
				93, 94, 95, 95, 96, 97, 98, 99,
				99, 100, 101, 102, 103, 103, 104, 105,
				106, 106, 107, 108, 109, 110, 110, 111,
				112, 113, 113, 114, 115, 116, 116, 117,
				118, 118, 119, 120, 120, 121, 121, 122,
				123, 123, 124, 124, 124, 125, 125, 126,
				126, 126, 126, 127, 127, 127, 127, 127,
				127, 127, 127, 127, 127, 126, 126, 126,
				126, 125, 125, 124, 124, 123, 122, 122,
				121, 120, 120, 119, 118, 117, 116, 115,
				114, 113, 111, 110, 109, 108, 106, 105,
				104, 102, 101, 99, 98, 96, 95, 93,
				92, 90, 89, 87, 85, 84, 82, 80,
				79, 77, 75, 74, 72, 70, 69, 67,
				65, 64, 62, 61, 59, 57, 56, 54,
				53, 51, 50, 48, 47, 46, 44, 43,
				41, 40, 39, 38, 36, 35, 34, 33,
				32, 31, 30, 29, 28, 27, 26, 25,
				24, 23, 22, 21, 20, 20, 19, 18,
				18, 17, 16, 16, 15, 14, 14, 13,
				13, 12, 12, 11, 11, 10, 10, 9,
				9, 9, 8, 8, 8, 7, 7, 7,
				6, 6, 6, 6, 5, 5, 5, 5,
				4, 4, 4, 4, 4, 4, 3, 3,
				3, 3, 3, 3, 3, 2, 2, 2,
				2, 2, 2, 2, 2, 2, 2, 2,
				1, 1, 1, 1, 1, 1, 1, 1,
				1, 1, 1, 1, 1, 1, 1, 1,
				1, 1, 1, 1, 1, 1, 1, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, -1, -1, -1, -1, -1, -1,
				-1, -1, -1, -1, -1, -1, -1, -1,
				-1, -1, -1, -1, -1, -1, -1, -1,
				-1, -2, -2, -2, -2, -2, -2, -2,
				-2, -2, -2, -2, -3, -3, -3, -3,
				-3, -3, -3, -4, -4, -4, -4, -4,
				-4, -5, -5, -5, -5, -6, -6, -6,
				-6, -7, -7, -7, -8, -8, -8, -9,
				-9, -9, -10, -10, -11, -11, -12, -12,
				-13, -13, -14, -14, -15, -16, -16, -17,
				-18, -18, -19, -20, -20, -21, -22, -23,
				-24, -25, -26, -27, -28, -29, -30, -31,
				-32, -33, -34, -35, -36, -38, -39, -40,
				-41, -43, -44, -46, -47, -48, -50, -51,
				-53, -54, -56, -57, -59, -61, -62, -64,
				-65, -67, -69, -70, -72, -74, -75, -77,
				-79, -80, -82, -84, -85, -87, -89, -90,
				-92, -93, -95, -96, -98, -99, -101, -102,
				-104, -105, -106, -108, -109, -110, -111, -113,
				-114, -115, -116, -117, -118, -119, -120, -120,
				-121, -122, -122, -123, -124, -124, -125, -125,
				-126, -126, -126, -126, -127, -127, -127, -127,
				-127, -127, -127, -127, -127, -127, -126, -126,
				-126, -126, -125, -125, -124, -124, -124, -123,
				-123, -122, -121, -121, -120, -120, -119, -118,
				-118, -117, -116, -116, -115, -114, -113, -113,
				-112, -111, -110, -110, -109, -108, -107, -106,
				-106, -105, -104, -103, -103, -102, -101, -100,
				-99, -99, -98, -97, -96, -95, -95, -94,
				-93, -92, -92, -91, -90, -90, -89, -88,
				-87, -87, -86, -85, -85, -84, -83, -83,
				-82, -81, -81, -80, -80, -79, -78, -78,
				-77, -77, -76, -75, -75, -74, -74, -73,
				-73, -72, -72, -71, -71, -70, -70, -69,
				-69, -68, -68, -67, -67, -66, -66, -65,
				-65, -64, -64, -64, -63, -63, -62, -62,
				-62, -61, -61, -60, -60, -60, -59, -59,
				-58, -58, -58, -57, -57, -57, -56, -56,
				-56, -55, -55, -55, -54, -54, -54, -53,
				-53, -53, -52, -52, -52, -51, -51, -51
			]
		"""



class NodeAffect(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "affect")

		# state
		self.sleep = signals.SleepState()
		self.mood = signals.AffectState()
		self.emotion = signals.AffectState()
		self.rtc = RTC(self.pars)

		# internal drive states
		self.d_valence = 0
		self.d_arousal = 0

		# state
		self.audio_level = 0
		self.audio_level_accum = 0
		self.jerk_head_smooth = np.array([0.0, 0.0, 0.0])
		self.jerk_head_smooth_i = 0

	def drive_arousal(self, d):

		# accumulate drive
		self.d_arousal += d

	def drive_valence(self, d):

		# accumulate drive
		self.d_valence += d

	def affect_from_adjust(self):

		# receive
		adjust = self.input.animal_adjust
		if not adjust is None:
			self.input.animal_adjust = None

			# handle adjust
			self.mood.adjust(adjust.mood)
			self.emotion.adjust(adjust.emotion)
			self.sleep.adjust(adjust.sleep)

	def affect_from_sleep_blocked(self):

		# detect high wakefulness and high sleep pressure
		q = self.sleep.wakefulness * self.sleep.pressure

		# map through offset
		q -= self.pars.affect.valence_sleep_blocked_thresh

		# clip at zero
		q = np.clip(q, 0.0, 1.0)

		# apply drive
		self.drive_valence(q * self.pars.affect.valence_sleep_blocked_gain)

	def affect_from_touch(self):

		# stroke can be +ve or -ve to valence (depending on direction)
		if self.state.stroke:

			delta = self.state.stroke

			self.drive_valence(delta * self.pars.affect.valence_stroke_gain)

			if self.mood.valence > self.pars.affect.arousal_stroke_thresh:
				self.drive_arousal(delta * self.pars.affect.arousal_stroke_gain)

			if self.mood.valence < (1.0 - self.pars.affect.arousal_stroke_thresh):
				self.drive_arousal(delta * -self.pars.affect.arousal_stroke_gain)

		# head touch (petting) will elicit +ve valence and +ve arousal
		if self.state.pet:

			self.drive_valence(self.state.pet * self.pars.affect.valence_pet_gain)
			self.drive_arousal(self.state.pet * self.pars.affect.arousal_pet_gain)

	def affect_from_accel(self):

		# smooth
		self.jerk_head_smooth[self.jerk_head_smooth_i] = self.state.jerk_head
		jerk = np.mean(self.jerk_head_smooth)
		self.jerk_head_smooth_i += 1
		if self.jerk_head_smooth_i == len(self.jerk_head_smooth):
			self.jerk_head_smooth_i = 0

		# default threshold
		thresh = self.pars.affect.valence_jerk_thresh

		# only if not being touched and not moving and not moving
		# cosmetic joints in head, since all of these will cause jerk
		if self.state.user_touch > 0 or self.state.in_motion > 0 or self.state.in_cos_head > 0:

			# use elevated threshold
			thresh = self.pars.affect.valence_jerk_thresh_2

		#print "{0:.3f}".format(jerk), "{0:.3f}".format(thresh), self.state.user_touch, self.state.in_motion, self.state.in_cos_head

		# map through offset, gain, saturation
		b = jerk - thresh
		c = b * self.pars.affect.valence_jerk_slope

		# clip
		d = np.clip(c, 0.0, 1.0)

		#if d > 0:
		#	print "affect_from_accel", d, jerk

		# apply to valence
		self.drive_valence(d * self.pars.affect.valence_jerk_gain)

	def affect_from_sound(self):

		# get audio level
		q = self.state.audio_events_for_50Hz
		for event in q:
			self.audio_level = max(event.level, self.audio_level)

		# run dynamics of audio_level accum
		self.audio_level_accum *= self.pars.affect.lambda_audio_level_accum

		# loud sounds will startle MIRO
		if self.audio_level > self.audio_level_accum:
			self.audio_level_accum = self.audio_level

		# clear stored input level
		self.audio_level = 0

		# EDIT: actually, it's fine, there's not much noise from either I think
		#
		# only if not being touched and not moving, since both will cause noise
		#if (self.state.user_touch > 0 or self.state.in_motion > 0):
		#	self.audio_level_accum = 0.0
		#	return

		#print "{0:.2f}".format(self.audio_level_accum)

		# apply to arousal
		d_arousal = self.audio_level_accum - self.pars.affect.arousal_audio_level_min
		d_arousal *= self.pars.affect.arousal_audio_level_rng_recip
		d_arousal = np.clip(d_arousal, 0.0, 1.0)
		self.drive_arousal(d_arousal * self.pars.affect.arousal_audio_gain)

		# apply to valence
		d_valence = self.audio_level_accum - self.pars.affect.valence_audio_level_min
		d_valence *= self.pars.affect.valence_audio_level_rng_recip
		d_valence = np.clip(d_valence, 0.0, 1.0)
		self.drive_valence(d_valence * self.pars.affect.valence_audio_gain)

	def affect_from_action_target(self):

		# positive action target values raise valence
		d_valence = self.state.action_target_valence
		if not d_valence is None:
			self.state.action_target_valence = None
			d_valence *= self.pars.affect.valence_action_target_gain
			if not self.pars.dev.DEBUG_HALT: # do not pollute info
				if np.abs(d_valence) > 0.001:
					print "valence boost from action target", d_valence
			self.drive_valence(d_valence)

		# handle also effects on arousal
		d_arousal = self.state.action_target_arousal
		if not d_arousal is None:
			self.state.action_target_arousal = None
			d_arousal *= self.pars.affect.arousal_action_target_gain
			if not self.pars.dev.DEBUG_HALT: # do not pollute info
				if np.abs(d_arousal) > 0.001:
					print "arousal boost from action target", d_arousal
			self.drive_arousal(d_arousal)

	def affect_from_random(self):

		# apply a random drive
		# not yet implemented
		pass

	def state_extrinsic_drive(self):

		if self.pars.flags.AFFECT_FROM_SLEEP_BLOCKED:
			self.affect_from_sleep_blocked()

		if self.pars.flags.AFFECT_FROM_TOUCH:
			self.affect_from_touch()

		if self.pars.flags.AFFECT_FROM_ACCEL:
			self.affect_from_accel()

		if self.pars.flags.AFFECT_FROM_SOUND:
			self.affect_from_sound()

		if self.pars.flags.AFFECT_FROM_ACTION_TARGET:
			self.affect_from_action_target()

		if self.pars.dev.RANDOMIZE_VALENCE:
			self.affect_from_random()

		# apply drives over a period of time so that emotional
		# changes are not too sudden - which can be jarring
		# when expressed (e.g. through lights)
		gamma = self.pars.affect.gamma_respond_drive
		x = self.d_valence * gamma
		self.emotion.valence += x
		self.d_valence -= x
		x = self.d_arousal * gamma
		self.emotion.arousal += x
		self.d_arousal -= x

	def state_valence_dynamics(self):

		# valence is computed relatively independently from arousal/sleep
		# (though there may be some cross-over in particular mapping, such
		# as used in extrinsic() above)
		if self.pars.flags.AFFECT_VALENCE_DYNAMICS:

			# mood valence approaches emotion valence
			self.mood.valence += self.pars.affect.gamma_mood * (self.emotion.valence - self.mood.valence)

			# emotion valence approaches mood valence
			self.emotion.valence += self.pars.affect.gamma_emotion * (self.mood.valence - self.emotion.valence)

			# mood valence tends to return to neutral
			gamma = self.pars.affect.gamma_neutral_valence_asleep
			if self.pars.dev.FAST_SLEEP_DYNAMICS:
				gamma *= self.pars.affect.fast_sleep_dyn_gain
			gamma_awake = self.pars.affect.gamma_neutral_valence_awake
			gamma += self.sleep.wakefulness * (gamma_awake - gamma)
			self.mood.valence += gamma * (0.5 - self.mood.valence)

		# clip
		self.mood.valence = np.clip(self.mood.valence, 0.0, 1.0)
		self.emotion.valence = np.clip(self.emotion.valence, 0.0, 1.0)

		# happy flag
		if not self.pars.flags.AFFECT_ENABLE_UNHAPPY:
			self.mood.valence = np.clip(self.mood.valence, 0.5, 1.0)
			self.emotion.valence = np.clip(self.emotion.valence, 0.5, 1.0)

	def state_arousal_dynamics(self):

		if self.pars.flags.AFFECT_AROUSAL_DYNAMICS:

			# compute arousal baseline target under current conditions

			# nominal arousal target
			arousal_target = self.pars.affect.arousal_nominal

			# adjustment from circadian rhythm
			if self.pars.flags.AFFECT_FROM_CLOCK:

				# cosine map
				arousal_rtc = -np.cos(self.time_of_day * 2.0 * np.pi)
				arousal_target += arousal_rtc * self.pars.affect.arousal_gain_rtc

			# store target from circadian rhythm
			self.pars.affect.arousal_target_circadian = arousal_target

			# adjust from wakefulness
			if self.pars.flags.AFFECT_FROM_WAKEFULNESS:
				arousal_target *= self.sleep.wakefulness

			# adjustment from light
			if self.pars.flags.AFFECT_FROM_LIGHT:
				d_arousal = self.state.light_mean
				d_arousal -= self.pars.affect.arousal_light_mean_offset
				d_arousal *= self.pars.affect.arousal_light_mean_gain
				arousal_target += d_arousal

			# clip to valid range
			arousal_target = np.clip(arousal_target, 0.0, 1.0)

			# arousal mood is then pulled towards a weighted combination of the baseline target
			# whilst arousal emotion is pulled towards the mood state, as for valence

			# mood arousal approaches emotion arousal
			arousal_target_imm = (arousal_target + self.pars.affect.arousal_target_emotion_weighting * \
									self.emotion.arousal)	* \
									self.pars.affect.arousal_target_emotion_weighting_recip

			self.mood.arousal += self.pars.affect.gamma_mood * (arousal_target_imm -
									self.mood.arousal)

			# emotion arousal approaches mood arousal
			self.emotion.arousal += self.pars.affect.gamma_emotion * (self.mood.arousal -
									   self.emotion.arousal)

		# clip
		self.mood_arousal = np.clip(self.mood.arousal, 0.0, 1.0)
		self.emotion.arousal = np.clip(self.emotion.arousal, 0.0, 1.0)

	def state_sleep_dynamics(self):
		'''
			- Rising sleep pressure will drive down wakefulness
			- Whilst wakefulness is low, sleep pressure decreases
			- Together, the two states form a relaxation oscillator
		'''

		# lock in current state for computing dynamics
		p = self.sleep.pressure - 0.5
		w = self.sleep.wakefulness - 0.5

		# compute dp/dw
		w2 = w * w
		w3 = w2 * w
		dp = self.pars.affect.sleep_dyn_p * w \
				+ self.pars.affect.sleep_dyn_q * w2
		dw = self.pars.affect.sleep_dyn_a * p \
				+ self.pars.affect.sleep_dyn_b * w \
				+ self.pars.affect.sleep_dyn_c * w3

		# accelerate
		if self.pars.dev.FAST_SLEEP_DYNAMICS:
			dp *= self.pars.affect.fast_sleep_dyn_gain
			dw *= self.pars.affect.fast_sleep_dyn_gain

		# integrate
		self.sleep.pressure += dp
		self.sleep.wakefulness += dw

		# constrain
		self.sleep.pressure = np.clip(self.sleep.pressure, 0.0, 1.0)
		self.sleep.wakefulness = np.clip(self.sleep.wakefulness, 0.0, 1.0)

		# keep awake
		if not self.pars.flags.AFFECT_ENABLE_SLEEP:

			# override sleep dynamics
			self.sleep.pressure = 0.0
			self.sleep.wakefulness = 1.0

	def tick(self):

		# tick clock
		self.time_of_day = self.rtc.get_time_of_day()

		# external influence on states
		self.affect_from_adjust()

		# enable
		if self.pars.flags.AFFECT_ENABLE:
			self.state_extrinsic_drive()
			self.state_valence_dynamics()
			self.state_arousal_dynamics()
			self.state_sleep_dynamics()

		# output
		self.state.emotion = self.emotion
		self.state.wakefulness = self.sleep.wakefulness

		# load affect output message
		msg = self.output.animal_state
		msg.sleep.pressure = self.sleep.pressure
		msg.sleep.wakefulness = self.sleep.wakefulness
		msg.mood.valence = self.mood.valence
		msg.mood.arousal = self.mood.arousal
		msg.emotion.valence = self.emotion.valence
		msg.emotion.arousal = self.emotion.arousal
		msg.time_of_day = self.time_of_day
