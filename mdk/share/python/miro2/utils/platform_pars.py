
import miro2 as miro
import numpy as np



class TimingPars(object):

	def __init__(self):

		self.tick_hz = miro.constants.PLATFORM_TICK_HZ
		self.tick_sec = miro.constants.PLATFORM_TICK_SEC



class GeomPars(object):

	def __init__(self):

		self.reachable_z_min = miro.constants.REACHABLE_Z_MIN
		self.reachable_z_max = miro.constants.REACHABLE_Z_MAX



class CameraPars(object):

	def __init__(self):

		self.elevation = [miro.constants.CAM_ELEVATION, miro.constants.CAM_ELEVATION]
		self.azimuth = [miro.constants.CAM_DIVERGENCE, -miro.constants.CAM_DIVERGENCE]
		self.location = [
			[miro.constants.LOC_EYE_X, miro.constants.LOC_EYE_Y, miro.constants.LOC_EYE_Z],
			[miro.constants.LOC_EYE_X, -miro.constants.LOC_EYE_Y, miro.constants.LOC_EYE_Z]
			]
		self.hori_half_fov = miro.constants.CAM_HORI_HALF_FOV
		self.tan_hori_half_fov = np.tan(self.hori_half_fov)
		self.norm_focal_length = 0.5 / self.tan_hori_half_fov
		self.pixel_aspect_ratio = miro.constants.CAM_PIXEL_ASPECT_RATIO
		self.distortion_model_k1 = miro.constants.CAM_DISTORTION_MODEL_K1
		self.distortion_model_k2 = miro.constants.CAM_DISTORTION_MODEL_K2
		self.distortion_model_k3 = miro.constants.CAM_DISTORTION_MODEL_K3
		self.distortion_model_k4 = miro.constants.CAM_DISTORTION_MODEL_K4



class PlatformPars (object):

	def __init__(self):

		self.timing = TimingPars()
		self.geom = GeomPars()
		self.camera = CameraPars()




