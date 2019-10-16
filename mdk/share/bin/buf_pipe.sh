#!/bin/bash

# this script provides a buffered pipe facility. linux
# fifos are blocking on the write side if there is no
# reader, and besides do not buffer data. this "buffered
# pipe" can receive input at any time, which is added to
# the buffer, and it returns immediately. any time it is
# read, the current buffer contents are returned and the
# buffer is cleared. at the back end, this is implemented
# using a write file and a read file, and the comparison
# between them. in principle these files could get very
# large, so it is only suitable for use with brief logs.

# next argument is direction
DIRECTION=$1
shift
[[ "$DIRECTION" == "write" || "$DIRECTION" == "read" ]] || {
	echo -e "direction must be 'write' or 'read'";
	exit 1;
}

# next argument is filename (identity of log)
[[ "$1" == "" ]] && {
	echo -e "filename must be provided";
	exit 1;
}

# filename can be absolute or relative to MIRO_DIR_STATE
if [[ "${1:0:1}" == "/" ]];
then
	FILENAME=$1
else
	FILENAME=$MIRO_DIR_STATE/$1
fi
shift

# defaults
CLEAR_ON_RESTART=1

# handle other arguments
for arg in $@
do

	[[ "$arg" == "--no-clear" ]] && { CLEAR_ON_RESTART=0; continue; }
	echo -e "argument unrecognised $arg"
	exit 1

done

# files
FILE_WRITE=$FILENAME.write
FILE_READ=$FILENAME.read



################################################################
## write

if [[ "$DIRECTION" == "write" ]];
then

	# brief report when writer starts
	echo "$0 start $DIRECTION"
	echo "$FILENAME"

	# clear write and read files on restart of writer
	if [[ "$CLEAR_ON_RESTART" == "1" ]];
	then
		echo "clearing files..."
		cp /dev/null $FILE_WRITE
		cp /dev/null $FILE_READ
	fi

	# loop until stdin disconnects, writing input to FILE_WRITE
	while read line
	do
		echo "$line" >> $FILE_WRITE
	done < /dev/stdin

fi



################################################################
## read

if [[ "$DIRECTION" == "read" ]];
then

	# if exist
	if [[ -f "$FILE_WRITE" ]];
	then

		# compute what's new in write that isn't yet in read, reading only one line max
		ADD=`cat $FILE_READ | comm -1 -3 --nocheck-order - $FILE_WRITE | sed 's/^/---SOL---/' | head -n 1`

		# if anything has been added
		if [[ "$ADD" != "" ]];
		then

			# add it now to read file
			#
			# remove SOL token which was only added to force head to
			# pass empty lines correctly
			echo -e "$ADD" | sed 's/---SOL---//' >> $FILE_READ

			# and print it out for the caller
			echo -e "$ADD" | sed 's/---SOL---//'

		fi

	fi

fi




