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



################################################################
## COMMON FUNCTIONS

# preserve command line
export COMMAND_LINE="$0 $@"

# ensure root
ensure_root()
{
	if [[ ($EUID -ne 0) ]]; then
		echo this operation requires root access, restarting using sudo...
		sudo -E $COMMAND_LINE
		exit $?
	fi
}

# start
operation_start()
{
	echo -e "________________________________________________________________\n"
	echo -e "[ $1 ]\n"
}

# end
operation_finished()
{
	echo -e "\nOperation completed successfully.\n"
}

# end
operations_finished()
{
	echo -e "________________________________________________________________\n"
	echo -e "All operations completed successfully.\n"
	exit 0
}

# already
already()
{
	echo -e "\t(operation already performed, not repeating)"
}

# warn
warn()
{
	echo -en "\n"
	if [ "$1" != "" ]; then echo -e "\t[WARNING] $1"; fi
	echo -e "\t**** WARNING ****\n"
}

# quit
quit()
{
	if [ "$1" != "" ]; then echo -e "\t[ERROR] $1"; fi
	echo -e "\t**** Operation FAILED ****\n"
	exit 1
}



################################################################
## CONFIGURE ENVIRONMENT

# ensure MIRO_SYSTEM
if [[ "$MIRO_SYSTEM" == "" ]];
then

	# report
	echo "seeking mdk/setup.bash..."

	# can find it from script location
	MIRO_DIR_MDK=$DIR_SELF/../..

	# try and find setup.bash
	if [ -e "$MIRO_DIR_MDK/setup.bash" ];
	then
		# source it now
		source $MIRO_DIR_MDK/setup.bash
	else
		# failed automatic find
		echo could not find it, please source mdk/setup.bash manually before running this script
		exit 1
	fi

fi




