#	{{##file_header##}}



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




