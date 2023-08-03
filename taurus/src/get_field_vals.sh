#!/bin/bash
#
# Script for using the data formatting array in a table file to match the text of 
# desired fields with its column position in a data file, and then print this
# data to stdout
#
# The table file must have lines of the form (without the leading '#', 
# which is necessary for commenting in this script):
#
# data_log_column_spec_t experiment_spec[] = 
# {
#  {"%13.8lf ", &my_gps.longitude, BASE_DOUBLE, REPLAY_USE}, //###112
#  {"%13.8lf ", &my_gps.latitude, BASE_DOUBLE, REPLAY_USE},  //###113
#  {"%6.3lf ", &my_gps.speed, BASE_DOUBLE, REPLAY_USE},      //###114
# };
#
# In other words, there must be six fields per line, ending in the "//###number"
# sequence.  The three pound signs are used to find members of this array,
# and the number afterwards is the column number in the data file. It is up to
# the programmer and user to make sure the table file has the correct format!!
#


NUMPARMS=$#
if [[ $NUMPARMS < 3 ]] 
then
	echo "Usage: $0 <table file> <data file> <words to match>"
	echo "Example: $0 /home/cacc3/veh/cacc3_tables.c /big/data/xe1123025.dat GPS date"
	exit 1
fi

if [ ! -f $1 ]
then
	echo Table file $1 does not exist
	exit 1
fi

if [ ! -f $2 ]
then
	echo Data file $2 does not exist
	exit 1
fi

PARMSTR="cat $1 | grep '###' "
declare -a PARMARR
PARMARR=($3 $4 $5 $6 $7 $8 $9)
INDEX=0
for((NUMPARMS=$#-2; $NUMPARMS>0; NUMPARMS--))
do
PARMSTR+="| grep -i ${PARMARR[$INDEX]} "
INDEX=$(($INDEX+1))
done
echo $PARMSTR >blah.txt
source blah.txt | awk '{print $3,$(NF)}' | sed '{s/\/\/###//g}' >blah2.txt

if [ `stat -c %s blah2.txt` == 0 ]
then
	echo "No matching field for input string"
	exit 1
fi

PARMSTR="cat $2 | awk '{print "
for x in `cat blah2.txt|awk '{print $2}'`
do
	PARMSTR+=\$"$x,"
done
echo $PARMSTR >blah.txt

PARMSTR=`echo $PARMSTR | sed '{s/,$//g}'`

PARMSTR+="}'"
echo $PARMSTR >blah3.txt
source blah3.txt
echo
echo "Matching fields and their column numbers are:"
cat blah2.txt | sed '{s/&//g}'
