#!/bin/bash
#	@section COPYRIGHT
#	Copyright (C) 2019 Consequential Robotics Ltd
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, see LICENSE in the
#	MDK root directory.
#	
#	Subject to the terms of this Agreement, Consequential
#	Robotics grants to you a limited, non-exclusive, non-
#	transferable license, without right to sub-license, to use
#	MIRO Developer Kit in accordance with this Agreement and any
#	other written agreement with Consequential Robotics.
#	Consequential Robotics does not transfer the title of MIRO
#	Developer Kit to you; the license granted to you is not a
#	sale. This agreement is a binding legal agreement between
#	Consequential Robotics and the purchasers or users of MIRO
#	Developer Kit.
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
#	KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#	WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#	PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
#	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#	OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# caller can skip slow parts if desired
if [[ "$MIRO_SETUP_QUICK" == "" ]];
then

	# report
	if [ "$MIRO_SETUP_SILENT" == "" ];
	then
		echo -e "________________________________________________________________\n"
		echo -e "Sourcing mdk/setup.bash...\n"
	fi

fi

# error function
miro_config_error()
{
	echo
	echo -e "**** CONFIGURATION ERROR:"
	echo -e "     $1"
	echo
}

# warn function
miro_warn()
{
	echo -e "\t**** WARNING **** $1"
}

# MDK info function
miro_info()
{
	echo
	echo -e "[ MIRO ]"
	env | grep MIRO_ | sed 's/\(MIRO_WIFI_PASS=\).*/\1********/' | sort | sed s!$HOME!~!g
	echo -e "\n[ ROS ]"
	env | grep ROS | grep -v UNKNOWN | sed s!$HOME!~!g
	env | grep GAZEBO | grep -v UNKNOWN | sed s!$HOME!~!g
	env | grep PYTHON | grep -v UNKNOWN | sed s!$HOME!~!g
	echo
}

# auto network address recovery function
miro_get_dynamic_address()
{
	# recover address (this works on the system as configured at the factory)
	MIRO_DYNAMIC_IP=`/sbin/ifconfig | grep inet | grep -v inet6 | grep -v 127.0.0.1 | head -n 1 | sed s/addr:// | sed s/.*inet\ // | sed s/\ .*//`
}

# network address resolution function
miro_resolve_network_address()
{
	# static
	[[ "$MIRO_NETWORK_MODE" == "static" ]] && { MIRO_LOCAL_IP=$MIRO_STATIC_IP; return; }

	# dynamic
	[[ "$MIRO_NETWORK_MODE" == "dynamic" ]] && { miro_get_dynamic_address; MIRO_LOCAL_IP=$MIRO_DYNAMIC_IP; return; }

	# loopback
	[[ "$MIRO_NETWORK_MODE" == "loopback" ]] && { MIRO_LOCAL_IP=127.0.0.1; return; }
}

# MIRO edition
export MIRO_EDITION=2

# MIRO token
export MIRO_TOKEN=miro$MIRO_EDITION

# identify system
UNAME=$(uname -m)
if [ "$UNAME" = "x86_64" ]; then
	export MIRO_SYSTEM=deb64
elif [ "$UNAME" = "i386" ]; then
	export MIRO_SYSTEM=deb32
elif [ "$UNAME" = "i686" ]; then
	export MIRO_SYSTEM=deb32
elif [ "$UNAME" = "armv7l" ]; then
	export MIRO_SYSTEM=arm32
fi

# get state dir - use XDG_RUNTIME_DIR if available since
# it's guaranteed tmpfs thus efficient for data exchange
DIR_STATE_PARENT=$XDG_RUNTIME_DIR
[[ "$DIR_STATE_PARENT" == "" ]] && DIR_STATE_PARENT=/tmp
export MIRO_DIR_STATE=$DIR_STATE_PARENT/$MIRO_TOKEN/state

# get user dirs
export MIRO_DIR_USER=~/.$MIRO_TOKEN
export MIRO_DIR_CONFIG=$MIRO_DIR_USER/config
export MIRO_DIR_TRASH=$MIRO_DIR_USER/trash

# get tmp dirs
export MIRO_DIR_TMP=/tmp/$MIRO_TOKEN
export MIRO_DIR_LOG=$MIRO_DIR_TMP/log
export MIRO_DIR_DUMP=$MIRO_DIR_TMP/dump

# get mdk dirs
export MIRO_DIR_MDK="$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd )"
export MIRO_DIR_SHARE=$MIRO_DIR_MDK/share
export MIRO_DIR_BIN=$MIRO_DIR_MDK/bin/$MIRO_SYSTEM
export MIRO_DIR_ONBOARD=$MIRO_DIR_MDK/bin/onboard

# get multitool
export MIRO_MULTITOOL=$MIRO_DIR_ONBOARD/multitool.sh

# check dirs exist
[ -d "$MIRO_DIR_BIN" ] || { miro_config_error "failed to find MDK bin directory for this system"; return; }

# ensure dirs exist
mkdir -p "$MIRO_DIR_CONFIG" "$MIRO_DIR_TRASH" "$MIRO_DIR_LOG" "$MIRO_DIR_DUMP" "$MIRO_DIR_STATE"

# caller can skip slow parts if desired
if [[ "$MIRO_SETUP_QUICK" == "" ]];
then

	# locate user setup file if there is one, else select default
	export MIRO_USER_SETUP=$MIRO_DIR_CONFIG/user_setup.bash
	if [ ! -e "$MIRO_USER_SETUP" ];
	then
		MIRO_USER_SETUP=$MIRO_DIR_SHARE/config/user_setup.bash
	fi

	# load that file
	source $MIRO_USER_SETUP || { miro_config_error "failed to source user_setup.bash"; }

	# resolve network address
	miro_resolve_network_address

	# set ROS network variables
	[[ "$ROS_MASTER_IP" == "" ]] && ROS_MASTER_IP=localhost
	export ROS_IP=$MIRO_LOCAL_IP
	export ROS_MASTER_URI=http://$ROS_MASTER_IP:11311

	# source ROS setup
	source /opt/ros/$MIRO_ROS_RELEASE/setup.bash || { miro_config_error "failed to source ROS setup.bash"; }
	
	# source Gazebo setup, if installed
	if [[ -f "/usr/share/gazebo/setup.sh" ]];
	then
		source /usr/share/gazebo/setup.sh
	fi

	# source catkin workspace setup
	CATKIN_INSTALL=install
	CATKIN_SETUP=$MIRO_DIR_MDK/catkin_ws/$CATKIN_INSTALL/setup.bash
	if [ -f "$CATKIN_SETUP" ];
	then
		source $CATKIN_SETUP || { miro_config_error "failed to source catkin setup.bash"; }
	else
		miro_config_error "catkin_ws has not yet been built on this machine - did you install_mdk.sh?"
	fi
	
	# add path to python share
	TMP=$MIRO_DIR_MDK/share/python
	[[ ":$PYTHONPATH:" != *":$TMP:"* ]] && PYTHONPATH=$TMP:$PYTHONPATH

	# make packages available to third-party software
	TMP=$MIRO_DIR_MDK/sim
	[[ ":$GAZEBO_RESOURCE_PATH:" != *":$TMP:"* ]] && GAZEBO_RESOURCE_PATH=$TMP:$GAZEBO_RESOURCE_PATH
	TMP=$MIRO_DIR_MDK/sim/models
	[[ ":$GAZEBO_MODEL_PATH:" != *":$TMP:"* ]] && GAZEBO_MODEL_PATH=$TMP:$GAZEBO_MODEL_PATH
	TMP=$MIRO_DIR_BIN
	[[ ":$GAZEBO_PLUGIN_PATH:" != *":$TMP:"* ]] && GAZEBO_PLUGIN_PATH=$TMP:$GAZEBO_PLUGIN_PATH

	# read MDK release
	export MIRO_MDK_RELEASE=$(cat $MIRO_DIR_MDK/RELEASE)
	[ "$MIRO_MDK_RELEASE" != "" ] || { miro_config_error "failed to read MDK release file"; return; }

	# report
	if [ "$MIRO_SETUP_SILENT" == "" ];
	then
		echo MIRO edition: $MIRO_EDITION
		echo MDK path: $MIRO_DIR_MDK
		echo MDK release: $MIRO_MDK_RELEASE
		echo User setup: $MIRO_USER_SETUP
		echo Network address: $MIRO_LOCAL_IP
		echo
		echo -e "Type \"miro_info\" to see your environment"
		echo -e "________________________________________________________________\n"
	fi

fi




