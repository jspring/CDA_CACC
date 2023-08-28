#!/bin/bash

ACCEL="$1"

if [[ ! -n $ACCEL ]]
then 
	ACCEL=0
fi
killall python2
killall accord_can
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
/home/das3/veh/accord/can/src/lnx/clt_vars &

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
	echo "Make sure BOTH the Komodo and the Panda emulator are plugged into USB ports"
	echo
	echo
	exit 1
fi
sleep 1
echo Starting accord_can...
/home/das3/veh/accord/can/src/lnx/accord_can -v &
if [ $? != 0 ]
then
	echo
	echo
	echo "Make sure BOTH the Komodo and the Panda emulator are plugged into USB ports"
	echo
	echo
fi
sleep 1
cd /windows/Linux/Argonne_CACC/Vehicle_emulator
python2 ./emulator.py >emulator.log 
/home/das3/veh/accord/can/test/stop_accord_can.sh
