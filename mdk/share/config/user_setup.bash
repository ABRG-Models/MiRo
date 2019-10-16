#!/bin/bash
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

#-	DO NOT MODIFY THIS FILE
#-
#-	this is a template configuration file and should not be modified.
#-	it is copied to the user directory during installation and this
#-	message is removed as part of that process. the installed copy of
#-	this file can be modified.
#-
#-	DO NOT MODIFY THIS FILE

#	This is the user configuration script for the MDK, and is shared
#	between the robot and off-board installations.
#
# *	On the robot, values here are modified by MIROapp; usually, you
#	should not need to modify them manually. If you do, see the
#	documentation page "Configure Robot".
#
# * If you are configuration an off-board installation, see the
#	documentation page "Configure MDK".



################  ROBOT & OFF-BOARD  ################

# The settings in this section are used both on-board and off-board;
# see the separate documentation pages listed above, but their behaviour
# is much the same in both cases.

# MIRO_NETWORK_MODE
#
# Select one of the available network modes, "dynamic", "static", or
# "loopback". The robot's network address is set either from the
# network address dynamically assigned to the network adapter (e.g.
# by DHCP), from MIRO_STATIC_IP, or to the loopback address.
export MIRO_NETWORK_MODE=dynamic

# MIRO_STATIC_IP
#
# Static network address for this robot. If MIRO_NETWORK_MODE is set
# to "static", MIRO_STATIC_IP is used as the robot's network address;
# otherwise, this setting is not used.
export MIRO_STATIC_IP=

# ROS_MASTER_IP
#
# This value locates the ROS master on the network. In addition, if it is
# set to "" (empty), a ROS master is started automatically at boot time.
export ROS_MASTER_IP=

# MIRO_ROBOT_NAME
#
# This setting is used as the base name for ROS topics (e.g. "/miro/...").
export MIRO_ROBOT_NAME=miro

# MIRO_ROS_RELEASE
#
# This setting indicates which ROS release to use in case more than
# one version is installed on the system.
export MIRO_ROS_RELEASE=kinetic



################  ROBOT ONLY  ################

# The settings in this section are used only on-board, and are not
# relevant to an off-board (workstation) installation of the MDK.

# MIRO_BRIDGE_FLAGS
#
# This setting is the flag set passed to the bridge when it is started
# by the multitool. See "miro_bridge help" for details of flags.
export MIRO_BRIDGE_FLAGS=l!

# MIRO_SPEAK_IP
#
# This setting, if equal to "1", causes the network address to be read
# out at boot time.
export MIRO_SPEAK_IP=1

# MIRO_SYSTEM_UPDATE_SCHEDULE
#
# This setting can be "never" (default if blank or missing), "weekly",
# "hourly", or "always". msm update_system will update the operating
# system on that basis, actioned in on_system_ready.
export MIRO_SYSTEM_UPDATE_SCHEDULE=

# MIRO_DYNAMIC_IP_WAIT
#
# Dynamic address recovery period in seconds. If MIRO_NETWORK_MODE is
# set to "dynamic", the robot's network address is recovered from the
# network adapter after it has connected. If the adapter has still not
# connected after MIRO_DYNAMIC_IP_WAIT seconds, the loopback address is
# used instead, allowing on-board software to run if the network is
# not available.
#
# NB: A value of "inf" will wait indefinitely for the connection.
export MIRO_DYNAMIC_IP_WAIT=60

# MIRO_DYNAMIC_IP_MATCH
#
# Dynamic address recovery will continue until an address matching
# this regular expression is found. This prevents accidentally
# picking up a temporarily-assigned IP address. If this is not
# necessary, use the default given below which matches any
# valid IP address.
#
# (default) MIRO_DYNAMIC_IP_MATCH=^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$
export MIRO_DYNAMIC_IP_MATCH='^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$'



################  FUNCTIONS  ################

# miro_get_dynamic_address
#
# If the user has configured the network to their own preference and
# the automatic address recovery function provided is not working, the
# user can override the provided function here by uncommenting below
# and authoring their own version that works with their network.
#miro_get_dynamic_address()
#{
#	# this function just has to return this value
#	MIRO_DYNAMIC_IP=
#
#	# for example, to recover address of named adapter "wlan0" in
#	# an off-board installation, use these lines:
#	ADAPTER_NAME=wlan0
#	MIRO_DYNAMIC_IP=`/sbin/ifconfig | grep $ADAPTER_NAME -A 2 | grep inet | grep -v inet6 | grep -v 127.0.0.1 | head -n 1 | sed 's/[^0-9]*//' | sed 's/\ .*//'`
#}




