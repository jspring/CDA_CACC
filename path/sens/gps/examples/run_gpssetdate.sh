#!/bin/sh

DAYLIGHT_SAVING_TIME=$1
CTR=10
while [[ $CTR -gt 0 ]]
do
	echo $DAYLIGHT_SAVING_TIME | grep "DST"
	if [[ $? -eq 0 ]]
	then 
		/home/path/sens/gps/examples/qnx/gpssetdate -v -DST </dev/ser1 &
	else
		/home/path/sens/gps/examples/qnx/gpssetdate -v </dev/ser1 &
	fi
	sleep 1
	slay gpssetdate
	CTR=$(($CTR-1))
	YEAR=`date +%y`
	if [[ ($YEAR -lt 16) || ($YEAR -gt 20) ]]
	then
		CTR=10
	fi
done
