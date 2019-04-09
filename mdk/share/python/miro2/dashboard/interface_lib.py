import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk,GObject,Gdk,GLib,GdkPixbuf

import sys
import rospy

# Outdated MiRo-B imports
# from std_msgs.msg import String
# from sensor_msgs.msg import Image, CompressedImage
# from geometry_msgs.msg import Twist

# import miro_msgs
# from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, \
#     core_state, core_control, core_config, bridge_config, bridge_stream

# MiRo-E ROS interfaces
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, UInt32, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage
import miro2 as miro


# from miro2.core.node_action import NodeAction



import numpy as np
import time
import threading
import cv2
import math

# # Outdated MiRo-B imports
# from miro_constants import miro

import Image
###############################################################


INTERPTYPE = GdkPixbuf.InterpType.BILINEAR

###############################################################


def hex2(x):
    return "{0:#04x}".format(x)

def error(msg):
    print(msg)
    sys.exit(0)


def usage():
    print """
Usage:
    miro_ros_client.py robot=<robot_name>

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.
    """
    sys.exit(0)

# We use a quartic barrel distortion model. Pass in the radius
# of a point in the affine projection to find the distortion
# coefficient.
def camera_model_distort_coeff(rad2):
	rad4 = rad2 * rad2
	return 1.0 \
		+ MIRO_CAM_DISTORTION_MODEL_K1 * rad2 \
		+ MIRO_CAM_DISTORTION_MODEL_K2 * rad4

# Apply distortion (map from affine projection space to image
# space).
def camera_model_u2d(u):
    rad = u[0] * u[0] + u[1] * u[1]
    z = camera_model_distort_coeff(rad)
    result = [0] * 2
    result[0] = u[0] * z;
    result[1] = u[1] * z;
    return result

# Map from d (true) to p (pixel).
#
# NB: This is strictly a transform into an image, and the
# image can be any size, so it's unsurprising that we have to
# indicate the image size as a parameter. For efficiency, we
# assume the image is either of size RAW or DEC.
def camera_model_d2p(d, rows, cols):
    # scale by pixel aspect ratio to correct y coordinate
    p = d
    p[1] = p[1] * MIRO_CAM_PIXEL_ASPECT_RATIO
    # scale by image width (yes, for y coordinate too - that's our standard)
    p[0] = p[0] * cols
    p[1] = p[1] * cols
    # transform by image centre (which is non-zero in P-space)
    p[0] = p[0] + 0.5 * cols - 0.5
    p[1] = p[1] + 0.5 * rows - 0.5
    # ok
    return p

def show_overlay(im):
    # The test pattern is a grid at a fixed range.
    patt_step = 20
    patt_lim = 100
    patt_range = 100
    # extract width and height
    cols = im.shape[1]
    rows = im.shape[0]
    # compute half FOV as an object half width at the given range
    hori_half_width = math.tan(MIRO_CAM_HORI_HALF_FOV) * patt_range
    # pre-divide
    hori_half_width_recip = 0.5 / hori_half_width
    # pre-stretch the pinhole projection so that after the barrel
    # distortion, the image is of the expected (correct) scale
    barrel_correc = 1.0 / camera_model_distort_coeff(0.25)
    # for each test pattern point
    u = [0] * 2
    for i in range(-patt_lim, patt_lim+1, patt_step):
        for j in range(-patt_lim, patt_lim+1, patt_step):
            # lay out pattern in undistorted image space
            u[0] = float(i) * hori_half_width_recip
            u[1] = float(j) * hori_half_width_recip
            # apply barrel distortion
            d = camera_model_u2d(u)
            # apply barrel correction
            d[0] = d[0] * barrel_correc
            d[1] = d[1] * barrel_correc
            # apply image acquisition
            p = camera_model_d2p(d, rows, cols)
            # round (deliberately round down because we're going to
            # make a pattern at each pixel that is 2x2)
            xi_ = int(p[0])
            yi_ = int(p[1])
            # loop
            for k in [0, 1]:
                for l in [0, 1]:
                    xi = xi_ + k
                    yi = yi_ + l
                    # write into image
                    if xi >= 0 and xi < cols and yi >= 0 and yi < rows:
                        q = xi * 3 + yi * cols * 3
                        im.data[q] = np.uint8(255);
                        im.data[q+1] = np.uint8(255);
                        im.data[q+2] = np.uint8(255);


MIRO_CAM_DISTORTION_MODEL_K1 = -0.75
MIRO_CAM_DISTORTION_MODEL_K2 = 0.25
MIRO_CAM_PIXEL_ASPECT_RATIO = 1.0
# MIRO_CAM_HORI_HALF_FOV = miro.__DEG2RAD(45)
MIRO_CAM_HORI_HALF_FOV = miro.constants.__DEG2RAD(45)
################################################################

class fifo:

    def __init__(self, uncompressed):
        self.N = 1  # raise if we get "overflow in fifo", or to improve fps measurement
        self.buf = [None] * self.N
        self.r = 0
        self.w = 0
        self.tN = 30
        self.ti = 0
        self.td = 10
        self.tt = [None] * self.tN
        self.hz = None
        self.uncompressed = uncompressed
        self.lock = threading.Lock()

    def push(self, frm):
        self.lock.acquire()
        try:
            t_frm = time.time()
            if self.buf[self.w] is None:
                if self.uncompressed:
                    pb = GdkPixbuf.Pixbuf.new_from_data(frm.data,
                        GdkPixbuf.Colorspace.RGB, False, 8,
                        frm.width, frm.height, frm.step)
                else:
                    im = cv2.imdecode(np.fromstring(frm.data, np.uint8),
                                cv2.IMREAD_COLOR)
                    w = im.shape[1]
                    h = im.shape[0]
                    N = h * w
                    for i in range(0, N):
                        tmp = im.data[i*3+0]
                        im.data[i*3+0] = im.data[i*3+2]
                        im.data[i*3+2] = tmp
                    pb = im
                    #pb = GdkPixbuf.Pixbuf.new_from_data(im.data, GdkPixbuf.Colorspace.RGB, False, 8, w, h, w*3)

                self.buf[self.w] = pb
            else:
                #print("**** frame dropped ****")
                pass
            self.tt[self.ti] = t_frm
            ti_bak = self.ti - self.td
            if ti_bak < 0:
                ti_bak = ti_bak + self.tN
            if not self.tt[ti_bak] is None:
                dt = t_frm - self.tt[ti_bak]
                self.hz = self.td / dt
                if self.hz > (self.td + 3) and self.td < (self.tN - 1):
                    self.td = self.td + 1
                    #print "> ", self.td
                if self.hz < (self.td - 3) and self.td > 5:
                    self.td = self.td - 1
                    #print "< ", self.td
            self.ti = (self.ti + 1) % self.tN
        finally:
            self.lock.release()

    def pop(self):
        obj = self.buf[self.r]
        if not obj is None:
            self.buf[self.r] = None
            self.r = (self.r + 1) % self.N
        return obj

    def latest(self):
        ret = None
        while True:
            obj = self.pop()
            if obj is None:
                break
            ret = obj
        return ret

    def freq(self):
        hz = self.hz
        if not hz is None:
            self.hz = None
        return hz

################################################################
class miro_ros_client:
    def config_send(self):
        # c = core_config()
        # c.P2B_W_signals = c.P2B_W_signals | miro.MIRO_P2B_W_BRANCH_ENABLE
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ENABLE
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ADJUST_RTC
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_VALENCE_DYNAMICS
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_AROUSAL_DYNAMICS
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ENABLE_SLEEP
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_CLOCK
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_WAKEFULNESS
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_TOUCH
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_LIGHT
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_SOUND
        # c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_ACCEL
        # #c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_SLEEP_BLOCKED
        # #c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_RANDOMIZE_VALENCE
        # #c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FAST_SLEEP_DYNAMICS
        # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_ENABLE
        # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_LIGHT
        # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_TAIL
        # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_EYELIDS
        # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_EARS
        # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_VOCAL
        # c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_BODY
        # #c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_PING
        # #c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_NO_PIRATE_NOISES
        # #c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_DO_PIRATE_NOISES
        # c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_ENABLE
        # #c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_DEBUG
        # #c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_FORCE_MULL
        # #c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_RANDOMIZE_ORIENT
        # #c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_DISABLE_HALT
        # c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_MODULATE_BY_SONAR
        # c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_ENABLE
        # #c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_RESET_KC_INTEGRATORS
        # #c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH
        # #c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH_MOTION
        # #c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH_TRANSLATION
        # c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH_INTO_SONAR
        # #c.P2L_W_signals = c.P2L_W_signals | miro.MIRO_P2L_W_ENABLE_POS_CONTROL
        # c.P2L_W_signals = c.P2L_W_signals | miro.MIRO_P2L_W_ENABLE_CLIFF_REFLEX
        # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_ENABLE
        # #c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_IGNORE_AUDIO
        # #c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_IGNORE_VIDEO
        # c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SEND_PRIORITY
        # #c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SEND_OTHER
        # #c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_NO_REAFF_COMPROMISE
        # #c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_NO_SUPPRESS
        # #c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SHOW_COMPROMISE
        # #c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SHOW_TEST_PATTERN
        # #c.P1_W_signals = c.P1_W_signals | miro.MIRO_P1_W_TEST_ALARM
        # #c.P1_W_signals = c.P1_W_signals | miro.MIRO_P1_W_NO_I2C_BUSY_ALARM

        # c.msg_flags = c.FLAG_UPDATE_SIGNALS
        # self.pub_core_config.publish(c)
        # print("config sent")
        pass
        return


        # # bridge/config
        # q = bridge_config()
        # q.flags = miro.MIRO_BRIDGE_UPDATE
        # self.pub_bridge_config.publish(q)
        # print("bridge/config sent")
        # # '''
        # # # bridge/stream
        # # q = bridge_stream()
        # # q.sound_index_P3 = 0 # Plays music
        # # self.pub_bridge_stream.publish(q)
        # # print("bridge/stream sent")
        # #
        # # # platform/config
        # # self.send_platform_config = False
        # # q = platform_config()
        # # q.reset = 0
        # # q.frame_size = 320
        # # max_frame_rate = 8
        # # q.frame_rate = max_frame_rate * 0.5
        # # self.pub_platform_config.publish(q)
        # # print("platform/config sent")
        # #
        # # # publish
        # # q = platform_control()
        # # q.msg_flags = platform_control.FLAG_SYNC_PLATFORM | platform_control.FLAG_SYNC_CORE
        # #
        # # # publish
        # # q = core_control()
        # # q.msg_flags = core_control.FLAG_SYNC_PLATFORM | core_control.FLAG_SYNC_CORE
        # # '''

    def callback_caml(self, frm):
        self.caml_fifo.push(frm)
        #self.image_caml = self.caml_fifo.latest()

        if self.do_config:
            self.config_send()
            self.do_config = False

    def callback_camr(self, frm):
        self.camr_fifo.push(frm)
        #self.image_camr = self.camr_fifo.latest()

    # priority frames
    def callback_pril(self, frm):
        self.pril_fifo.push(frm)
        #self.image_pril = self.pril_fifo.latest()

    def callback_prir(self, frm):
        self.prir_fifo.push(frm)
        #self.image_prir = self.prir_fifo.latest()

    def callback_priw(self, frm):
        self.priw_fifo.push(frm)
        #self.image_priw = self.priw_fifo.latest()

    # '''
    # def callback_rgbl(self, frm):
    #     self.rgbl_fifo.push(frm)
    #
    # def callback_rgbr(self, frm):
    #     self.rgbr_fifo.push(frm)
    # '''

    def callback_platform_sensors(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_sensors = object


    def callback_platform_state(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_state = object

        # # send back a core control message so that flags get respected
        # q = core_control()
        # # q.msg_flags = core_control.FLAG_SYNC_PLATFORM | core_control.FLAG_SYNC_CORE
        # self.pub_core_control.publish(q)

    def callback_platform_mics(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_mics = object

    def callback_core_time(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.core_time = object

    def callback_core_affect(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.core_affect = object


    def __init__(self):

        self.do_config = True

        self.image_caml = None
        self.image_camr = None
        self.image_pril = None
        self.image_prir = None
        self.image_priw = None


        self.rtc_hrs = None
        self.rtc_mins = None
        self.rtc_secs = None
        self.rtc_skew = None

        # report
        print("initialising...")
        print(sys.version)

        # no arguments gives usage
        if len(sys.argv) == 1:
            usage()

        # options
        self.drive_pattern = ""
        self.opt = lambda: 0
        self.opt.robot_is_phys = None
        self.opt.uncompressed = False

        # handle args
        for arg in sys.argv[1:]:
            f = arg.find('=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f + 1:]
            if key == "robot":
                self.opt.robot_name = val
            elif key == "rob":
                self.opt.robot_is_phys = True
            elif key == "sim":
                self.opt.robot_is_phys = False
            elif key == "uncompressed":
                self.opt.uncompressed = True
            else:
                error("argument not recognised \"" + arg + "\"")

        # check we got at least one
        if len(self.opt.robot_name) == 0:
            error("argument \"robot\" must be specified")

        # set inactive
        self.active = False

        # topic root
        # topic_root = "/miro/" + self.opt.robot_name

        # MiRo-E sim appears to just use /miro, may need changing for phys. robot
        topic_root = "/miro"
        print "Topic root:", topic_root

        # publish
        # Come back to this later
        # self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)
        # self.pub_core_control = rospy.Publisher(topic_root + "/core/control", core_control, queue_size=0)
        # self.pub_core_config = rospy.Publisher(topic_root + "/core/config", core_config, queue_size=0)
        # self.pub_bridge_config = rospy.Publisher(topic_root + "/bridge/config", bridge_config, queue_size=0)
        # self.pub_bridge_stream = rospy.Publisher(topic_root + "/bridge/stream", bridge_stream, queue_size=0)
        # self.pub_platform_config = rospy.Publisher(topic_root + "/platform/config", platform_config, queue_size=0)
        #
        # # subscribe
        # self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_platform_sensors)
        self.sub_sensors = rospy.Subscriber(topic_root + "/sensors/package", miro.msg.sensors_package, self.callback_platform_sensors)

        # self.sub_state = rospy.Subscriber(topic_root + "/platform/state", platform_state, self.callback_platform_state)
        # self.sub_mics = rospy.Subscriber(topic_root + "/platform/mics", platform_mics, self.callback_platform_mics)
        self.sub_mics = rospy.Subscriber(topic_root + "/sensors/mics", Int16MultiArray, self.callback_platform_mics)

        # Appears to be no single 'core state' replacement, will have to make several updates
        # self.sub_core_state = rospy.Subscriber(topic_root + "/core/state", core_state, self.callback_core_state)
        self.sub_core_affect = rospy.Subscriber(topic_root + "/core/affect", miro.msg.affect_state, self.callback_core_affect)
        # self.sub_core_time = rospy.Subscriber(topic_root + "/core/affect/time", UInt16, self.callback_core_time)

        # set active
        self.active = True

        # default data
        self.platform_sensors = None
        # self.platform_state = None
        # self.platform_mics = None
        self.caml_fifo = fifo(self.opt.uncompressed)
        self.camr_fifo = fifo(self.opt.uncompressed)
        self.pril_fifo = fifo(self.opt.uncompressed)
        self.prir_fifo = fifo(self.opt.uncompressed)
        self.priw_fifo = fifo(self.opt.uncompressed)
        # self.rgbl_fifo = fifo(self.opt.uncompressed)
        # self.rgbr_fifo = fifo(self.opt.uncompressed)
        # self.core_state = None
        # self.mood = None
        self.core_affect = None
        # self.core_time = 0

        if self.opt.uncompressed:
            # self.sub_caml = rospy.Subscriber(topic_root + "/platform/caml", Image, self.callback_caml)
            # self.sub_camr = rospy.Subscriber(topic_root + "/platform/camr", Image, self.callback_camr)
            self.sub_caml = rospy.Subscriber(topic_root + "/sensors/caml", Image, self.callback_caml)
            self.sub_camr = rospy.Subscriber(topic_root + "/sensors/camr", Image, self.callback_camr)

            # self.sub_pril = rospy.Subscriber(topic_root + "/core/pril", Image, self.callback_pril)
            # self.sub_prir = rospy.Subscriber(topic_root + "/core/prir", Image, self.callback_prir)
            # self.sub_priw = rospy.Subscriber(topic_root + "/core/priw", Image, self.callback_priw)
            self.sub_pril = rospy.Subscriber(topic_root + "/sensors/pril", Image, self.callback_pril)
            self.sub_prir = rospy.Subscriber(topic_root + "/sensors/prir", Image, self.callback_prir)
            self.sub_priw = rospy.Subscriber(topic_root + "/sensors/priw", Image, self.callback_priw)

            # '''
            # self.sub_rgbl = rospy.Subscriber(topic_root + "/core/rgbl", Image, self.callback_rgbl)
            # self.sub_rgbr = rospy.Subscriber(topic_root + "/core/rgbr", Image, self.callback_rgbr)
            # '''
        else:
            # self.sub_caml = rospy.Subscriber(topic_root + "/platform/caml/compressed", CompressedImage, self.callback_caml)
            # self.sub_camr = rospy.Subscriber(topic_root + "/platform/camr/compressed", CompressedImage, self.callback_camr)
            self.sub_caml = rospy.Subscriber(topic_root + "/sensors/caml/compressed", CompressedImage, self.callback_caml)
            self.sub_camr = rospy.Subscriber(topic_root + "/sensors/camr/compressed", CompressedImage, self.callback_camr)

            # self.sub_pril = rospy.Subscriber(topic_root + "/core/pril/compressed", CompressedImage, self.callback_pril)
            # self.sub_prir = rospy.Subscriber(topic_root + "/core/prir/compressed", CompressedImage, self.callback_prir)
            # self.sub_priw = rospy.Subscriber(topic_root + "/core/priw/compressed", CompressedImage, self.callback_priw)
            self.sub_pril = rospy.Subscriber(topic_root + "/sensors/pril/compressed", CompressedImage, self.callback_pril)
            self.sub_prir = rospy.Subscriber(topic_root + "/sensors/prir/compressed", CompressedImage, self.callback_prir)
            self.sub_priw = rospy.Subscriber(topic_root + "/sensors/priw/compressed", CompressedImage, self.callback_priw)

            # '''
            # self.sub_rgbl = rospy.Subscriber(topic_root + "/core/rgbl/compressed", CompressedImage, self.callback_rgbl)
            # self.sub_rgbr = rospy.Subscriber(topic_root + "/core/rgbr/compressed", CompressedImage, self.callback_rgbr)
            # '''

    #==================================================
    # '''def update_data(self):
    #     # sensors
    #     q = self.platform_sensors
    #     self.platform_sensors = None
    #     if hasattr(q, 'battery_state'):
    #         self.vbat = q.battery_state.voltage
    #     else:
    #         self.vbat = q.battery_voltage
    #     self.vtemp = q.temperature.temperature
    #     self.eyelid_closure = q.eyelid_closure
    #     self.sonar = q.sonar_range.range
    #     self.accel_head = q.accel_head.linear_acceleration
    #     self.accel_body = q.accel_body.linear_acceleration.x
    #     self.odomx = q.odometry.twist.twist.linear.x
    #     self.odomz = q.odometry.twist.twist.angular.z
    #
    #     self.text_joints = q.joint_state.position
    #     self.text_joints_effort = q.joint_state.effort
    #     self.light = q.light
    #     self.touch_head = q.touch_head
    #     self.touch_body = q.touch_body
    #     self.cliff = q.cliff
    #     self.dip_state = hex2(q.dip_state_phys)
    #
    #     # cameras
    #     #self.image_caml = self.caml_fifo.latest()
    #     #self.image_camr = self.camr_fifo.latest()
    #
    #     # core_state
    #     q = self.core_state
    #     self.core_state = None
    #     self.selection = q.selection
    #     self.emotion = q.emotion
    #     self.mood = q.mood
    #     self.sleep = q.sleep
    #     self.priority = q.priority
    #     self.disinhibition = q.disinhibition
    #
    #     # platform_statertc
    #     q = self.platform_state
    #     self.platform_state = None
    #     if q is not None:
    #         self.rtc_hrs = q.rtc_hrs
    #         #self.rtc_mins = q.rtc_mins
    #         #self.rtc_secs = q.rtc_secs
    #         self.rtc_skew = q.rtc_skew
    # '''