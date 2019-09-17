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
#
#	Action search
#	action for MIRO to search master's face


import numpy as np
import tf
import rospy
import copy
import miro2 as miro

from action_types import ActionTemplate



class ActionSearch(ActionTemplate):

    def finalize(self):

        # parameters
        self.name = "search"
        self.retreatable = True

    def compute_priority(self):

        # extract variables
        valence = self.input.emotion.valence
        arousal = self.input.emotion.arousal
        height = self.input.priority_peak.height
        size_norm = self.input.priority_peak.size_norm
        fixation = self.input.fixation

        # extract pars
        base_priority = self.pars.action.approach_base_prio
        size_gain = self.pars.action.approach_size_gain
        arousal_gain = self.pars.action.approach_arousal_gain
        valence_gain = self.pars.action.approach_valence_gain
        fixation_gain = self.pars.action.approach_fixation_gain

        # compute priority
        tend_some = np.tanh(arousal_gain * (arousal - 0.5) + fixation_gain * (fixation - 0.5))
        tend_appe = np.tanh(valence_gain * (valence - 0.5) - size_gain * (size_norm - 0.5))
        priority = height * (base_priority + tend_some + tend_appe)

        # modulate by cliff and sonar
        priority *= self.input.conf_surf
        priority *= self.input.conf_space

        # modulate for dev
        if self.pars.flags.DEV_ORIENT_ONLY:
            priority = 0.0

        # ok set the max priority, then we can only do search action and test it ~lucy
        return self.pars.action.priority_uninterruptable  #priority

    def event_start(self):

        # self.appetitive_response(self.pars.action.approach_appetitive_commitment)
        self.debug_event_start()

    def start(self):

        # compute start point for fovea in WORLD
        self.fovea_i_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)

        # compute end point for fovea in WORLD
        fovea_f_WORLD = self.kc.changeFrameAbs(
                miro.constants.LINK_HEAD,
                miro.constants.LINK_WORLD,
                miro.utils.kc_interf.kc_view_to_HEAD(
                    self.input.priority_peak.azim,
                    self.input.priority_peak.elev,
                    self.input.priority_peak.range
                    )
                )

        # limit end point to be reachable
        fovea_f_WORLD[2] = np.clip(fovea_f_WORLD[2],
                            self.pars.geom.reachable_z_min,
                            self.pars.geom.reachable_z_max
                            )

        # compute total movement fovea will make in world
        self.dfovea_WORLD = fovea_f_WORLD - self.fovea_i_WORLD

        # compute pattern time
        total_dist = np.linalg.norm(self.dfovea_WORLD)
        secs_ideal = total_dist * self.pars.action.approach_speed_spm
        steps_ideal = int(secs_ideal * self.pars.timing.tick_hz)
        steps_constrained = np.clip(steps_ideal,
                    self.pars.action.approach_min_steps,
                    self.pars.action.approach_max_steps
                    )

        # start pattern
        self.clock.start(steps_constrained)

        # debug
        if self.debug:
            print "fovea_i_WORLD", self.fovea_i_WORLD
            print "fovea_f_WORLD", fovea_f_WORLD
            print "pattern time", total_dist, secs_ideal, steps_ideal, steps_constrained

    def service(self):

        print "doing Action Search"
        print  'searchActionLoop',self.input.priority_peak.azim
        # read clock
        x = self.clock.cosine_profile()
        self.clock.advance(True)

        # compute an interim target along a straight trajectory
        fovea_x_WORLD = x * self.dfovea_WORLD + self.fovea_i_WORLD

        # transform interim target into HEAD for actioning
        fovea_x_HEAD = self.kc.changeFrameAbs(miro.constants.LINK_WORLD, miro.constants.LINK_HEAD, fovea_x_WORLD)

        # apply push
        self.apply_push_fovea(fovea_x_HEAD - self.fovea_HEAD)

        # debug fovea movement through WORLD
        #fovea_WORLD = self.kc.changeFrameAbs(miro.constants.LINK_HEAD, miro.constants.LINK_WORLD, self.fovea_HEAD)
        #print "fovea_WORLD", fovea_WORLD, self.clock.toString()