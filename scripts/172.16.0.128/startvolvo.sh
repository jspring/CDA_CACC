#!/bin/sh
#
# Script to run device drivers and longitudinal control code

NOCOMM=$1
TEST_DIR=/home/qnxuser/path_can_bin
CAN_CLIENT_DIR=/home/qnxuser/path_can_bin
CONFIG_FILE=realtime.ini
CAN_DRIVER_DIR=/home/qnxuser/path_can_bin

	slay  -f -Q -s TERM trk_wr
	slay  -f -Q -s TERM rdj1939
	slay  -f -Q -s TERM veh_snd
	slay  -f -Q -s TERM veh_rcv
	slay  -f -Q -s TERM gpsdb
	slay  -f -Q -s TERM jbussend
	slay  -f -Q -s TERM trk_cr
	slay  -f -Q -s TERM db_slv

	DATESTR=`date +%m%d%Y_%H%M%S`
	echo $DATESTR >/big/data/last_start_timestamp.txt

	IPADDR=`ifconfig | grep 172.16.0 | gawk '{print $2}'`

	echo $IPADDR | grep 127 
	if [[ $? -eq 0 ]]
	then
	        ITRIADDR=172.16.1.127
	        TRUCK=Gold
	fi
	$TEST_DIR/db_slv -Q -S `hostname` &
	sleep 1
	$TEST_DIR/trk_cr -t 1000 2>long_input_$DATESTR.dbg &
	sleep 1
	$CAN_CLIENT_DIR/rdj1939 -c -f 1 >/big/data/rdj1939_can1_$DATESTR.txt &
	sleep 1
	# To start jbussend with debugging ON, add -d flag
	$TEST_DIR/jbussend -d -c -e 1 -b 1 >/big/data/jbussend_$DATESTR.dbg &
	sleep 1
	$TEST_DIR/trk_wr -t 100 1>/big/data/trk_wr_$DATESTR.txt 2>/big/data/trk_wr.err &

	echo $NOCOMM | grep "no_communication"
	if [[ $? -ne 0 ]]
	then
		sleep 1
		$TEST_DIR/veh_snd -v -i 100 -A $IPADDR -a $ITRIADDR -u 1516 >/big/data/veh_snd_$DATESTR.dbg &
		sleep 1
		$TEST_DIR/veh_rcv -v -A $IPADDR -a $ITRIADDR -u 15000 -o 3 >/big/data/veh_rcv_$DATESTR.dbg &
		sleep 1
	fi
	sleep 1
	GPSDBNUM=202
	su qnxuser ssh rsu@$ITRIADDR gpspipe -r | $TEST_DIR/gpsdb -n $GPSDBNUM -d1 >/big/data/gpsdb_$DATESTR.dbg
