#!/bin/sh

LOGFILE="/home/jspring/bin/sms_sensors_present.log"
TMPFILE="/home/jspring/bin/tmpfile"
INFILE="/home/jspring/bin/infile"

if [[ `ps -e | grep smsparse | grep -v grep` != '' ]]
then
	echo `date`: smsparse already running $0 exiting >>$LOGFILE
	exit 0
fi

/home/path/sens/sms/src/lnx/smsparse -d48 >$INFILE &
sleep 20
killall smsparse

cat $INFILE  | grep CASE_SENSOR_CTL1 >$TMPFILE

if [[ `stat -c %s $TMPFILE` -ne 0 ]]
then
	ERR=0
	for x in `cat $TMPFILE | awk '{print $8}'`
	do
		if [[ $x -ne "0xf" ]]
		then
			ERR=1
			echo `date`: sensors present byte is $x >>$LOGFILE
		fi
	done
	
	if [[ $ERR -eq 0 ]]
	then
		echo `date`: sensors OK >>$LOGFILE
	fi
else
	echo `date`: No data in $TMPFILE or $TMPFILE nonexistant >>$LOGFILE
	if [[ `ping -c10 172.16.1.200` -eq '' ]]
	then
		echo `date`: 172.16.1.200 not responding to ping >>$LOGFILE
		exit 0
	else
		echo `date`: 172.16.1.200 is responding to ping >>$LOGFILE
	fi
fi

cat $INFILE  | grep objctl >$TMPFILE

linectr=0
if [[ `stat -c %s $TMPFILE` -ne 0 ]]
then
	ERR=0
	for x in `cat $TMPFILE | awk '{print $7}'`
	do
		linectr=$(($linectr+1))

		# The first three t_scan reads give false high scan rates
		if [[ $linectr -gt 3 ]]
		then
			x=0x$x
			if [[ $x -gt 100 ]]
			then
				ERR=1
				echo `date`: t_scan is $(($x)) msec >>$LOGFILE
			fi
		fi
	done
	
	if [[ $ERR -eq 0 ]]
	then
		echo `date`: t_scan OK >>$LOGFILE
	fi
fi
