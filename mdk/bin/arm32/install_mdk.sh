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

#	This script will set up the local system to use the MDK
#	in the current directory

#	To run the installer, use this command:
#
#		./install_mdk.sh

# import common
DIR_SELF="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
source $DIR_SELF/../onboard/common.sh || exit 1

# install file function
install_file()
{
	SRC=$1
	DST=$2

	echo -e "Install \"$SRC\"..."
	if [ -e "$DST" ];
	then
		already
	else
		cat $SRC | grep -v "#-" > $DST
	fi
	echo -e "\tOK\n"
}



################################################################
## INSTALL

# report
operation_start "INSTALL MDK"

# put link at ~/mdk
echo -e "Add symbolic link to MDK at ~/mdk..."
DST=~/mdk
if [[ -h "$DST" ]];
then
        TGT=`realpath $DST`
        CUR=`pwd -P`
        [[ "$CUR" == "$TGT/bin/$MIRO_SYSTEM" ]] || quit "A link exists at ~/mdk already; does not point at this MDK"
        already
else
        ln -s $MIRO_DIR_MDK $DST
fi
echo -e "\tOK\n"

# add setup.bash to bashrc
echo -e "Add setup.bash to .bashrc..."
DST=~/.bashrc
LINK=`cat $DST | grep "mdk/setup.bash"`
if [[ "$LINK" == "" ]];
then
	echo -e "\n# MDK\nsource ~/mdk/setup.bash\n\n" >> $DST
else
	already
fi
echo -e "\tOK\n"

# install user_setup.bash
SRC=$MIRO_DIR_MDK/share/config/user_setup.bash
DST=$MIRO_DIR_CONFIG/user_setup.bash
install_file "$SRC" "$DST"

# install platform_parameters
SRC=$MIRO_DIR_MDK/share/config/platform_parameters
DST=$MIRO_DIR_CONFIG/platform_parameters
install_file "$SRC" "$DST"

# install simulation_parameters
SRC=$MIRO_DIR_MDK/share/config/simulation_parameters
DST=$MIRO_DIR_CONFIG/simulation_parameters
install_file "$SRC" "$DST"

# install demo_parameters.py
SRC=$MIRO_DIR_MDK/share/config/demo_parameters.py
DST=$MIRO_DIR_CONFIG/demo_parameters.py
install_file "$SRC" "$DST"

# make catkin_ws
cd $MIRO_DIR_MDK/catkin_ws
[[ -d "install" ]] || catkin_make install



################################################################
## FINISH

operations_finished



