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
# the "animal_state" is the state of the animal aspects of the model
# which includes "affect" values for emotion (quickly changing) and
# mood (slowly changing) and a "sleep" value (also slowly changing),
# and the animal's estimate of time. this state may also include, in
# future, physical states such as temperature.

#	DOCLINK ANIMAL STATE FLAGS
#
#	Some flags are included here because parts of the implementation
#	are in separate nodes that read this topic in order to determine
#	how they should behave, and their behaviour is affected by flags.
#
#	The values of these flags are defined in miro2.constants.
uint32 flags

# affective states
affect emotion
affect mood

# sleep state
sleep sleep

# normalised time of day (0.0 -> 1.0)
float32 time_of_day

# normalised ambient sound level (0.0 -> 1.0)
float32 sound_level
