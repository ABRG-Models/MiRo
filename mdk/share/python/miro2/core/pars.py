
import numpy as np
import miro2 as miro
import rospy
import os
import imp



class ROS(object):

	def __init__(self):

		self.log_level = rospy.DEBUG
		self.robot_name = "miro"

		# use MIRO_ROBOT_NAME
		MIRO_ROBOT_NAME = os.getenv('MIRO_ROBOT_NAME')
		if (not MIRO_ROBOT_NAME is None) and (len(MIRO_ROBOT_NAME) > 0):
			self.robot_name = MIRO_ROBOT_NAME

	def get_log_level_string(self):

		if self.log_level == rospy.DEBUG:
			return "verb"
		if self.log_level == rospy.INFO:
			return "info"
		if self.log_level == rospy.WARN:
			return "warn"
		if self.log_level == rospy.ERROR:
			return "silent"
		if self.log_level == rospy.FATAL:
			return "silent"

	def get_cmd_opt_string(self):

		opt = " "
		opt += " log=" + self.get_log_level_string()
		opt += " robot=" + self.robot_name
		return opt



class Platform(object):

	def __init__(self):

		# time for movement to settle after commands stop changing
		self.t_mov_settle = 0.75

		# time for head IMU to stop reporting jerk after eye blink command
		self.t_blink_settle = 0.5

		# time for IMU to stop reporting jerk after any cosmetic command change in that section
		# see dev/system_monitor/record_cos_and_jerk
		self.t_head_cos_settle = 0.8
		self.t_body_cos_settle = 0.8

		# time for our own vocalisations to fall out of the microphone buffers
		self.t_vocalising_settle = 0.1

	def finalize(self, pars):

		self.n_mov_settle = int(self.t_mov_settle * pars.timing.tick_hz)
		self.n_blink_settle = int(self.t_blink_settle * pars.timing.tick_hz)
		self.n_head_cos_settle = int(self.t_head_cos_settle * pars.timing.tick_hz)
		self.n_body_cos_settle = int(self.t_body_cos_settle * pars.timing.tick_hz)
		self.n_vocalising_settle = int(self.t_vocalising_settle * pars.timing.tick_hz)



class Express(object):

	def __init__(self):

		# lights
		self.ORANGE_red = 128
		self.ORANGE_grn = 64
		self.ORANGE_blu = 0
		self.led_phase_range = np.pi * 0.5
		self.led_phase_separation = np.pi * 0.5

		# eyelids
		self.eyelids_droop_on_touch = 0.5
		self.blink_mean_interval = 500
		self.blink_refractory_period = 100
		self.blink_period = 8
		self.double_blink_period = 16
		self.double_blink_prob = 0.2

		# tail
		self.tail_wag_max_amp = 0.5



class Body(object):

	def __init__(self):

		self.body_pose_filt_gain_dr = 0.8
		self.body_pose_filt_gain_dtheta = 0.8
		self.wheels_sleep_suppress_gain = 1.0
		self.lift_sleep_droop_gain = 1.0
		self.body_pose_filt_L = 10
		self.sonar_min_range = 0.10
		self.sonar_max_range = 0.25
		self.max_accel_mmpsps = 4000.0
		self.max_decel_mmpsps = 4000.0



class Selection(object):

	def __init__(self):

		self.selection_hysteresis = 0.05
		self.selection_noise_mag = 0.01



class Lower(object):

	def __init__(self):

		self.user_touch_tau_attack = 0.25
		self.user_touch_tau_release = 0.5
		self.user_touch_min = 0.01
		self.user_touch_gain = 0.2

		# touch sensor masks mask out sensor elements that are unreliable
		self.touch_head_mask = 0xFFFF
		self.touch_body_mask = 0xFFF0

		# lists of touch elements involved in each stroke line
		# format is [<first bit>, <number of bits>, <stroke polarity>]
		#
		# NB: central line is disabled because MIRO strokes itself there!
		#	[0, 4, -1.0],
		self.stroke_lines = [
			[4, 5, 1.0],
			[9, 5, 1.0]
		]

	def finalize(self, pars):

		fS = pars.timing.tick_hz
		self.user_touch_gamma_attack = miro.utils.tau2gamma( self.user_touch_tau_attack, fS )
		self.user_touch_gamma_release = miro.utils.tau2gamma( self.user_touch_tau_release, fS )



class Decode(object):

	def __init__(self):

		# image height at which we process within demo
		self.image_height = 100 # 90, 100, 120, 150

	def finalize(self, pars):

		self.image_width = self.image_height * 16 / 9



class DetectAudio(object):

	def __init__(self):

		self.inter_ear_distance = 0.104 # metres
		self.raw_magnitude_thresh = 0.01 # normalized; audio event processing skipped unless over thresh
		self.assumed_sound_source_height = 1.0 # metres
		self.assumed_sound_source_range = 1.5 # metres

	def finalize(self, pars):

		speed_of_sound = 343.0 # m/s
		self.inter_ear_lag = self.inter_ear_distance / speed_of_sound * miro.constants.MIC_SAMPLE_RATE



class DetectFace(object):

	def __init__(self):

		self.resources_dir = "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades"

	def finalize(self, pars):

		pass



class Action(object):

	def __init__(self):

		self.fixation_region_width = 0.5
		self.range_estimate_min = 0.1
		self.range_estimate_max = 2.0
		self.size_large = 0.1

		self.halt_stall_input_filt = 0.333
		self.halt_stall_output_filt = 0.15
		self.halt_stall_acc_filt = 0.1
		self.halt_stall_thresh = 0.25
		self.halt_stall_eff_gain = 0.5
		self.halt_stall_acc_gain = 15.0
		self.halt_num_steps = 20

		self.orient_base_prio = 0.5
		self.orient_speed_sec_per_rad = 2.0
		self.orient_gaze_target_radius = 1.0
		self.orient_min_steps = 25
		self.orient_max_steps = 100
		self.orient_appetitive_commitment = 0.5

		self.approach_base_prio = 0.2
		self.approach_size_gain = 0.5
		self.approach_arousal_gain = 1.0
		self.approach_valence_gain = 0.2
		self.approach_fixation_gain = 1.5
		self.approach_speed_mps = 0.2
		self.approach_min_steps = 50
		self.approach_max_steps = 300
		self.approach_appetitive_commitment = 0.5

		self.flee_base_prio = -0.2
		self.flee_fixation_gain = 0.0
		self.flee_arousal_gain = 0.25
		self.flee_valence_gain = 0.5
		self.flee_size_gain = 0.75
		self.flee_speed_mps = 0.3
		self.flee_min_steps = 50
		self.flee_max_steps = 300

		self.avert_base_prio = 0.0

		self.retreat_distance_m = 0.6
		self.retreat_speed_mps = 0.2
		self.retreat_rand_gain = 0.3

		self.cliff_thresh = 6
		self.cliff_margin = 1

		self.priority_idle = 0.1
		self.priority_uninterruptable = 1.0
		self.priority_high = 0.75
		self.priority_medium = 0.5
		self.priority_low = 0.25

		# known sizes of particular objects are used to estimate range
		self.face_size_m = 0.3
		self.ball_size_m = 0.2

		# these entries correspond to the entries in priority_peak.source_conf
		# each is the value to MIRO attached to that class of target; at
		# time of writing, that's 1.0 for his ball (my toy!) and 0.5 for a
		# person's face.
		self.priority_peak_source_value = np.array([0.5, 1.0])

	def finalize(self, pars):

		# derived
		self.fixation_width_recip = 1.0 / self.fixation_region_width
		self.size_large_recip = 1.0 / self.size_large
		self.approach_speed_spm = 1.0 / self.approach_speed_mps
		self.flee_speed_spm = 1.0 / self.flee_speed_mps



class Spatial(object):

	def __init__(self):

		self.audio_event_level_gain = 10
		self.audio_event_assumed_range = 2000
		self.audio_event_assumed_height = 600
		self.moving_afferent_filter_gain = 0.9
		self.moving_afferent_filter_thresh = 1.0
		self.t_aec_settle = 1.0
		self.t_aud_history = 50 * 1000
		self.audio_event_azim_size_rad = 0.1
		self.audio_event_elev_size_rad = 0.3
		self.audio_event_gain = 5.0
		self.audio_event_gain_making_noise = 1.0
		self.face_gain = 1.0
		self.pri_decay_lambda = 0.7
		self.pri_peak_height_thresh = 0.75
		self.pri_filt_width = 2
		self.degrees_hindsight = 30
		self.wide_stream_n = 256
		self.ball_gain = 0.75
		self.face_gain = 1.0
		self.association_angle = 0.0872 # 5 degrees is 0.0872 rad

	def finalize(self, pars):

		self.audio_event_azim_size_recip = 1.0 / self.audio_event_azim_size_rad
		self.audio_event_elev_size_recip = 1.0 / self.audio_event_elev_size_rad



class Affect(object):

	def __init__(self):

		self.valence_stroke_gain = 0.05
		self.valence_pet_gain = 0.025
		self.valence_audio_level_min = 0.2
		self.valence_audio_level_max = 1.0
		self.tau_audio_level_accum = 0.1
		self.valence_audio_gain = -0.05
		self.valence_jerk_thresh = 1.0
		self.valence_jerk_thresh_2 = 4.0
		self.valence_jerk_slope = 0.5
		self.valence_jerk_gain = -0.2
		self.valence_orient_fixation_gain = 0.1
		self.valence_sleep_blocked_thresh = 0.75
		self.valence_sleep_blocked_gain = -0.001
		self.valence_action_target_gain = 0.5
		self.arousal_stroke_thresh = 0.75
		self.arousal_stroke_gain = 0.05
		self.arousal_pet_gain = 0.025
		self.arousal_audio_level_min = 0.2
		self.arousal_audio_level_max = 1.0
		self.arousal_audio_gain = 0.025
		self.arousal_light_mean_offset = 0.2
		self.arousal_light_mean_gain = 0.1
		self.arousal_nominal = 0.5
		self.arousal_gain_rtc = 0.125
		self.arousal_target_emotion_weighting = 2.0
		self.tau_respond_drive = 1.0
		self.tau_neutral_valence_asleep = 120.0
		self.tau_neutral_valence_awake = 600.0
		self.tau_mood = 4.0
		self.tau_emotion = 4.0
		self.sleep_dyn_p = 1.0 * 0.0002
		self.sleep_dyn_q = -1.0 * 0.0002
		self.sleep_dyn_a = -1.0 * 0.002
		self.sleep_dyn_b = 2.0 * 0.002
		self.sleep_dyn_c = -8.0 * 0.002
		self.fast_sleep_dyn_gain = 25.0

	def finalize(self, pars):

		self.arousal_target_emotion_weighting_recip = 1.0 / ( 1.0 +
								self.arousal_target_emotion_weighting )
		self.valence_audio_level_rng_recip = 1.0 / (
							self.valence_audio_level_max -
							self.valence_audio_level_min )
		self.arousal_audio_level_rng_recip = 1.0 / (
							self.arousal_audio_level_max -
							self.arousal_audio_level_min )

		fS = pars.timing.tick_hz
		self.lambda_audio_level_accum = miro.utils.tau2lambda( self.tau_audio_level_accum, fS )
		self.gamma_neutral_valence_asleep = miro.utils.tau2gamma(self.tau_neutral_valence_asleep, fS)
		self.gamma_neutral_valence_awake = miro.utils.tau2gamma(self.tau_neutral_valence_awake, fS)
		self.gamma_mood = miro.utils.tau2gamma(self.tau_mood, fS)
		self.gamma_emotion = miro.utils.tau2gamma(self.tau_emotion, fS)
		self.gamma_respond_drive = miro.utils.tau2gamma(self.tau_respond_drive, fS)



class Flags( object ):

	def __init__( self ):

		# most defaults are 1 (enabled)
		self.AFFECT_ENABLE				= 1
		self.AFFECT_ADJUST_RTC			= 1
		self.AFFECT_VALENCE_DYNAMICS	= 1
		self.AFFECT_AROUSAL_DYNAMICS	= 1
		self.AFFECT_ENABLE_SLEEP		= 1
		self.AFFECT_ENABLE_UNHAPPY		= 1
		self.AFFECT_FROM_CLOCK			= 1
		self.AFFECT_FROM_TOUCH			= 1
		self.AFFECT_FROM_LIGHT			= 1
		self.AFFECT_FROM_SOUND			= 1
		self.AFFECT_FROM_WAKEFULNESS	= 1
		self.AFFECT_FROM_ACCEL			= 1
		self.AFFECT_FROM_SLEEP_BLOCKED	= 1
		self.AFFECT_FROM_ACTION_TARGET	= 1

		self.EXPRESS_ENABLE				= 1
		self.EXPRESS_THROUGH_LIGHT		= 1
		self.EXPRESS_THROUGH_TAIL		= 1
		self.EXPRESS_THROUGH_EARS		= 1
		self.EXPRESS_THROUGH_EYELIDS	= 1
		self.EXPRESS_THROUGH_VOICE		= 1
		self.EXPRESS_THROUGH_NECK		= 1
		self.EXPRESS_THROUGH_WHEELS		= 1

		self.ACTION_ENABLE				= 1
		self.ACTION_ENABLE_INPUT		= 1
		self.ACTION_MODULATE_BY_SONAR	= 1
		self.ACTION_MODULATE_BY_CLIFF	= 1
		self.ACTION_ENABLE_SONAR_STOP	= 1
		self.ACTION_ENABLE_MOVE_AWAY	= 1
		self.ACTION_HALT_ON_STALL		= 1

		self.SALIENCE_FROM_MOTION		= 1
		self.SALIENCE_FROM_SOUND		= 1
		self.SALIENCE_FROM_BALL			= 1
		self.SALIENCE_FROM_FACES		= 1

		self.BODY_ENABLE_CLIFF_REFLEX	= 1
		self.BODY_ENABLE_TRANSLATION	= 1
		self.BODY_ENABLE_ROTATION		= 1
		self.BODY_ENABLE_NECK_MOVEMENT	= 1

		# developer flags default to 0 (disabled)
		self.DEV_RANDOMIZE_VALENCE		= 0
		self.DEV_FAST_SLEEP_DYNAMICS	= 0
		self.DEV_DEBUG_HALT				= 0
		self.DEV_DETECT_FACE			= 0
		self.DEV_DETECT_BALL			= 0
		self.DEV_ORIENT_ONLY			= 0
		self.DEV_RUN_TEST_ACTION		= 0
		self.DEV_RECONFIG_CAMERA_QUICK	= 0
		self.DEV_DEBUG_ACTION_PARAMS	= 1
		self.DEV_DEBUG_ORIENT_START		= 0
		self.DEV_SEND_DEBUG_TOPICS		= 0

		# developer flags accessible from MIROapp (for now)
		self.DEV_DEBUG_SONAR			= 0
		self.DEV_DEBUG_DETECTION		= 0



class CorePars (object):

	def __init__(self):

		# import platform pars
		platform_pars = miro.utils.platform_pars.PlatformPars()
		self.timing = platform_pars.timing
		self.geom = platform_pars.geom
		self.camera = platform_pars.camera

		# augment with core pars
		self.ros = ROS()
		self.platform = Platform()
		self.lower = Lower()
		self.decode = Decode()
		self.express = Express()
		self.body = Body()
		self.selection = Selection()
		self.action = Action()
		self.spatial = Spatial()
		self.affect = Affect()
		self.detect_audio = DetectAudio()
		self.detect_face = DetectFace()
		self.flags = Flags()

		# finalize
		self.finalize()

	def action_demo_flags(self, flags):

		# report
		print "demo_flags: \"" + flags + "\""

		# handle flags
		if "v" in flags: print "v disable vocalisation"
		if "t" in flags: print "t disable translation"
		if "r" in flags: print "r disable rotation"
		if "n" in flags: print "n disable neck movement"
		if "s" in flags: print "s disable sleep"
		if "c" in flags: print "c disable cliff reflex"
		if "u" in flags: print "u disable shun cliffs"
		if "d" in flags: print "d disable affect from sound"
		if "M" in flags: print "M disable salience from motion"
		if "S" in flags: print "S disable salience from sound"
		if "F" in flags: print "F disable salience from faces"
		if "B" in flags: print "B disable salience from ball"
		if "T" in flags: print "T disable express through tail"
		if "h" in flags: print "h disable negative valence"
		if "a" in flags: print "a disable move away"
		if "H" in flags: print "H disable halt on stall"
		if "m" in flags: print "m disable sonar modulation"
		if "P" in flags: print "P debug sonar modulation"
		if "Q" in flags: print "Q debug detection"

		# not accessible from MIROapp
		if "R" in flags: print "R debug halt"
		if "I" in flags: print "I disable action input"

		# report
		print "end demo_flags"

		# do config (normally on)
		self.flags.EXPRESS_THROUGH_VOICE = 0 if "v" in flags else 1
		self.flags.BODY_ENABLE_TRANSLATION = 0 if "t" in flags else 1
		self.flags.BODY_ENABLE_ROTATION = 0 if "r" in flags else 1
		self.flags.BODY_ENABLE_NECK_MOVEMENT = 0 if "n" in flags else 1
		self.flags.AFFECT_ENABLE_SLEEP = 0 if "s" in flags else 1
		self.flags.BODY_ENABLE_CLIFF_REFLEX = 0 if "c" in flags else 1
		self.flags.ACTION_MODULATE_BY_CLIFF = 0 if "u" in flags else 1
		self.flags.AFFECT_FROM_SOUND = 0 if "d" in flags else 1
		self.flags.SALIENCE_FROM_MOTION = 0 if "M" in flags else 1
		self.flags.SALIENCE_FROM_SOUND = 0 if "S" in flags else 1
		self.flags.SALIENCE_FROM_FACES = 0 if "F" in flags else 1
		self.flags.SALIENCE_FROM_BALL = 0 if "B" in flags else 1
		self.flags.EXPRESS_THROUGH_TAIL = 0 if "T" in flags else 1
		self.flags.AFFECT_ENABLE_UNHAPPY = 0 if "h" in flags else 1
		self.flags.ACTION_ENABLE_MOVE_AWAY = 0 if "a" in flags else 1
		self.flags.ACTION_HALT_ON_STALL = 0 if "H" in flags else 1
		self.flags.ACTION_MODULATE_BY_SONAR = 0 if "m" in flags else 1
		self.flags.ACTION_ENABLE_SONAR_STOP = 0 if "m" in flags else 1
		self.flags.ACTION_ENABLE_INPUT = 0 if "I" in flags else 1

		# do config (normally off)
		self.flags.DEV_DEBUG_SONAR = 1 if "P" in flags else 0
		self.flags.DEV_DEBUG_DETECTION = 1 if "Q" in flags else 0
		self.flags.DEV_DEBUG_HALT = 1 if "R" in flags else 0

	def finalize(self):

		# changes to parameters are stored in demo_parameters.py
		# which allows the user to change anything in this object
		import_path = os.getenv("MIRO_DIR_CONFIG")
		module_name = "demo_parameters"
		q = imp.find_module(module_name, [import_path,])
		try:
			module = imp.load_module(module_name, q[0], q[1], q[2])
			q[0].close()
			module.update_parameters(self)

			# for developer convenience, we also allow demo_flags
			# in this file, but we don't advertise this to the user
			try:
				self.action_demo_flags(module.demo_flags)
			except:
				pass
		finally:
			if q[0]:
				q[0].close()

		# some parameters need to be accessible through MIROapp
		# and may also affect the bridge; these are stored in
		# platform_parameters (this includes "demo_flags" which
		# at time of writing does not affect the bridge, but may
		# do in future if we add more flags).
		params_filename = import_path + "/platform_parameters"
		if not os.path.isfile(params_filename):
			miro.utils.warning("params not found")

		else:
			with open(params_filename) as f:
				file = f.readlines()

			for line in file:

				try:

					line = line.strip()

					if len(line) == 0:
						continue

					if line[0] == "#":
						continue

					eq = line.find("=")
					if eq == -1:
						continue

					key = line[0:eq]
					val = line[eq+1:]

					if key == "cliff_thresh":
						if val:
							self.action.cliff_thresh = int(val)
					if key == "cliff_margin":
						if val:
							self.action.cliff_margin = int(val)
					if key == "demo_flags":
						self.action_demo_flags(val)

				except:

					print "exception handling line:", line

		# you must call this function after you change any parameters
		self.platform.finalize(self)
		self.lower.finalize(self)
		self.decode.finalize(self)
		self.action.finalize(self)
		self.affect.finalize(self)
		self.detect_audio.finalize(self)
		self.detect_face.finalize(self)
		self.spatial.finalize(self)



