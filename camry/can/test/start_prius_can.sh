#!/bin/bash

ACCEL="$1"

if [[ ! -n $ACCEL ]]
then 
	ACCEL=0
fi
killall prius_can
killall -s KILL komodo_read_can_write_db
killall -s KILL komodo_read_db_write_can
killall clt_vars
killall db_slv
/home/path/db/db_clean.sh

/bin/ps -ef | grep db_slv | grep -v grep

if [ $? != 0 ]
then
	/home/path/db/lnx/db_slv &
fi

sleep 1
/home/das3/veh/prius/can/src/lnx/clt_vars &

sleep 1
cd /home/path/sens/can2usb/komodo/files/komodo-api-linux-x86_64-v1.00/c
echo "Starting komodo_read_can_write_db \(add -v if verbose output needed\)..."
lnx/komodo_read_can_write_db &
sleep 1
if [[ `ps -ef | grep -v grep | grep komodo_read_can_write_db` == '' ]]
then
	echo $0: komodo_db_read_can_write_db startup error. Exiting.
	echo
	echo
	echo "Make sure the Komodo is plugged into a USB port"
	echo
	echo
	exit 1
fi
sleep 1
echo Starting prius_can...
/home/das3/veh/prius/can/src/lnx/prius_can -v &
if [ $? != 0 ]
then
	echo
	echo
	echo "Make sure BOTH the Komodo and the Panda emulator are plugged into USB ports"
	echo
	echo
fi
