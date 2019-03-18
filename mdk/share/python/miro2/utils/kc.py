
import numpy as np
import math
import copy
import ctypes
import os



###################################### CONSTANTS & STATE #######################################

# internal value used by the kc
KC_ANGLE_UNCONSTRAINED = 1000.0 * np.pi

# identifier of the frame that is the parent to
# the zeroth frame in the kc stack.
KC_FRAME_PARENT = -1

# flag values
KC_PUSH_FLAG_IMPULSE = 1
KC_PUSH_FLAG_VELOCITY = 2
KC_PUSH_FLAG_NO_PARENT_TRANSLATION = 4
KC_PUSH_FLAG_NO_PARENT_ROTATION = 8
KC_PUSH_FLAG_NO_INTERNAL_MOVEMENT = 16

# default values
KC_TICK_SEC=0.02
KC_PUSH_FLAGS_DEFAULT=KC_PUSH_FLAG_VELOCITY
KC_PUSH_LINK_DEFAULT=0

# function to initialise default values
def kc_init(tick, flags, link):
	global KC_TICK_SEC, KC_PUSH_FLAGS_DEFAULT, KC_PUSH_LINK_DEFAULT
	KC_TICK_SEC=tick
	KC_PUSH_FLAGS_DEFAULT=flags
	KC_PUSH_LINK_DEFAULT=link



###################################### HELPER FCNS #######################################

def kc_rotate( point_in, axis, angle ):

	point_out = np.array([0.0, 0.0, 0.0])

	c = np.cos( angle )
	s = np.sin( angle )

	if 'x' in axis:
		point_out[1] = c*point_in[1] - s*point_in[2]
		point_out[2] = s*point_in[1] + c*point_in[2]
		point_out[0] = point_in[0]

	elif 'y' in axis:
		point_out[2] = c*point_in[2] - s*point_in[0]
		point_out[0] = s*point_in[2] + c*point_in[0]
		point_out[1] = point_in[1]

	elif 'z' in axis:
		point_out[0] = c*point_in[0] - s*point_in[1]
		point_out[1] = s*point_in[0] + c*point_in[1]
		point_out[2] = point_in[2]

	else:
		raise ValueError( 'invalid axis' )

	return point_out

def kc_rotate_fwd( lhs, link ):
	return kc_rotate( lhs, link.axis, link.angle )

def kc_rotate_rev( lhs, link ):
	return kc_rotate( lhs, link.axis, -link.angle )



###################################### OBJECT #######################################

class KinematicPush:

	def __init__(self):

		self.flags = KC_PUSH_FLAGS_DEFAULT
		self.link = KC_PUSH_LINK_DEFAULT
		self.pos = np.array([0.0, 0.0, 0.0])
		self.vec = np.array([0.0, 0.0, 0.0])

	def resolve(self):

		# if neither, return zero
		if not ((self.flags & KC_PUSH_FLAG_VELOCITY) or (self.flags & KC_PUSH_FLAG_IMPULSE)):

			push = KinematicPush()
			return push

		# if is_velocity, convert to position
		if self.flags & KC_PUSH_FLAG_VELOCITY:

			push = KinematicPush()
			push.flags = self.flags & ~(KC_PUSH_FLAG_VELOCITY) | KC_PUSH_FLAG_IMPULSE
			push.link = self.link
			push.pos = copy.copy(self.pos)
			push.vec = self.vec * KC_TICK_SEC
			return push

		else:

			# otherwise, make a copy and return that, so
			# we don't change our input
			return copy.copy(self)



###################################### OBJECT #######################################

class KinematicChain:

	def __init__( self, link_desc_array, holonomic=None ):

		# params
		self.holonomic = False
		if not holonomic is None:
			self.holonomic = holonomic

		# zero pose change
		if self.holonomic:
			self.poseChange = np.array([0.0, 0.0, 0.0])
		else:
			self.poseChange = np.array([0.0, 0.0])

		# create kc
		self.dll_ctype = ctypes.c_double
		self.dll_ntype = np.float64
		path = os.getenv("MIRO_DIR_BIN")
		self.dll = ctypes.cdll.LoadLibrary(path + "/libmiro_kc.so")
		uncon = np.array([KC_ANGLE_UNCONSTRAINED]).astype(self.dll_ntype)
		self.dll.kc_init(0, uncon.ctypes.data_as(ctypes.POINTER(self.dll_ctype)))
		holonomic = 0
		if self.holonomic:
			holonomic = 1
		self.kc_handle = self.dll.kc_create("john", holonomic)

		# create links
		for link_desc in link_desc_array:

			# extract init data
			name = link_desc[0]
			trans = link_desc[1]
			axis = link_desc[2]
			angle_ini = link_desc[3]
			pars = link_desc[4]

			# add link
			trans = link_desc[1].astype(self.dll_ntype)
			angle_ini = np.array(link_desc[3]).astype(self.dll_ntype)
			pars = np.array(link_desc[4]).astype(self.dll_ntype)
			self.dll.kc_addLink(self.kc_handle,
				name,
				trans.ctypes.data_as(ctypes.POINTER(self.dll_ctype)),
				axis,
				angle_ini.ctypes.data_as(ctypes.POINTER(self.dll_ctype)),
				pars.ctypes.data_as(ctypes.POINTER(self.dll_ctype))
				)

	def zeroPose(self):

		# zero pose
		if self.dll.kc_zeroPose(self.kc_handle) == -1:
			raise ValueError("error in libmiro_kc")

	def getPose(self):

		# get pose
		pose = np.array([0.0, 0.0, 0.0]).astype(self.dll_ntype)
		if self.dll.kc_getPose(self.kc_handle, pose.ctypes.data_as(ctypes.POINTER(self.dll_ctype))) == -1:
			raise ValueError("error in libmiro_kc")
		return (pose[0:2], pose[2])

	def accumPoseChange(self, dpose):

		# just add it up
		self.poseChange += dpose

	def getPoseChange(self):

		# get accumulated pose change
		poseChange = copy.copy(self.poseChange)

		# zero pose change
		if self.holonomic:
			self.poseChange = np.array([0.0, 0.0, 0.0])
		else:
			self.poseChange = np.array([0.0, 0.0])

		# ok
		return poseChange

	def setPose(self, pose):

		# set pose
		pose = np.array(pose).astype(self.dll_ntype)
		if self.dll.kc_setPose(self.kc_handle, pose.ctypes.data_as(ctypes.POINTER(self.dll_ctype))) == -1:
			raise ValueError("error in libmiro_kc")

	def setConfig( self, config ):

		# set config
		config = np.array(config).astype(self.dll_ntype)
		if self.dll.kc_setConfig(self.kc_handle, 0, len(config), config.ctypes.data_as(ctypes.POINTER(self.dll_ctype))) == -1:
			raise ValueError("error in libmiro_kc")

	def setConfigIfInactive(self, config):

		# set config if inactive
		config = np.array(config).astype(self.dll_ntype)
		result = self.dll.kc_setConfig(self.kc_handle, 1, len(config), config.ctypes.data_as(ctypes.POINTER(self.dll_ctype)))
		if result == -1:
			raise ValueError("error in libmiro_kc")
		if result == 1:
			return True
		return False

	def isActive(self):

		# return active state
		result = self.dll.kc_getActive(self.kc_handle)
		if result == -1:
			raise ValueError("error in libmiro_kc")
		return result > 0

	def getConfig(self):

		# get config
		config = np.array([0.0] * 8).astype(self.dll_ntype)
		result = self.dll.kc_getConfig(self.kc_handle, config.ctypes.data_as(ctypes.POINTER(self.dll_ctype)))
		if result == -1:
			raise ValueError("error in libmiro_kc")
		return config[0:result].tolist()


	def getState( self ):

		# combine pose and config into one list
		return [self.getPose(), self.getConfig()]

	def changeFrameAbs( self, inFrame, outFrame, pos ):

		pos = pos.astype(self.dll_ntype)
		result = self.dll.kc_changeFrameAbs(self.kc_handle, inFrame, outFrame, pos.ctypes.data_as(ctypes.POINTER(self.dll_ctype)))
		if result == -1:
			raise ValueError("error in libmiro_kc")
		return pos


	def changeFrameRel( self, inFrame, outFrame, pos ):

		pos = pos.astype(self.dll_ntype)
		result = self.dll.kc_changeFrameRel(self.kc_handle, inFrame, outFrame, pos.ctypes.data_as(ctypes.POINTER(self.dll_ctype)))
		if result == -1:
			raise ValueError("error in libmiro_kc")
		return pos


	def push(self, push):

		# pushes in link KC_FRAME_PARENT are null
		if push.link == KC_FRAME_PARENT:
			return

		# resolve push to an impulse
		push = push.resolve()

		# if push is empty, no effect
		if np.linalg.norm(push.vec) == 0.0:
			return

		# apply push
		pushpos = push.pos.astype(self.dll_ntype)
		pushvec = push.vec.astype(self.dll_ntype)
		dpose = np.array([0.0, 0.0, 0.0]).astype(self.dll_ntype)
		result = self.dll.kc_push(self.kc_handle,
				push.link,
				push.flags,
				pushpos.ctypes.data_as(ctypes.POINTER(self.dll_ctype)),
				pushvec.ctypes.data_as(ctypes.POINTER(self.dll_ctype)),
				dpose.ctypes.data_as(ctypes.POINTER(self.dll_ctype))
				)
		if result == -1:
			raise ValueError("error in libmiro_kc")
		if self.holonomic:
			self.accumPoseChange(dpose)
		else:
			self.accumPoseChange(dpose[0:2])




