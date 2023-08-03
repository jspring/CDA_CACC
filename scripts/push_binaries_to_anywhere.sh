#!/bin/bash

USER=$1
IPADDR=$2
TOP_REPO_DIR=$3
DEST_DIR=$4
PORT=$5

if [  q$USER == 'q' ] || [  w$IPADDR == 'w' ] ||  [ z$TOP_REPO_DIR == 'z' ] 
then
        echo "Usage $0 <username on destination pc> <IP address of destination host pc> <top directory of repository on host pc, e.g. \"/windows/Linux/Volvo_rolys\"> <Destination directory>"
	exit 1
fi
if [ $? -ne 0 ] 
then
	echo /home/qnxuser/volvo/bin does not exist
	exit 1
fi
if [[  q$PORT -eq 'z' ]] 
then
	PORT=22
fi

scp -P $PORT -p $TOP_REPO_DIR/path/db/qnx/db_slv $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/avcs/qnx/trk_cr $USER@$IPADDR:$DEST_DIR
#scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/test/realtime.ini $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/can/jbus/src/qnx/rdj1939 $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/can/client/steinhoff/qnx/can_rx $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/can/client/steinhoff/qnx/can_tx $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/avcs/qnx/jbussend $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/vehcomm/qnx/veh_snd $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/vehcomm/qnx/veh_rcv $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/vehcomm/qnx/dvi_rcv $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/vehcomm/qnx/dvi_snd $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/path/sens/gps/examples/qnx/gpsdb $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/path/sens/gps/examples/qnx/gpssetdate $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/avcs/qnx/trk_wr $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/avcs/qnx/wrfiles_volvo $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/avcs/qnx/replay_truck $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/path/sens/lidar/leddar/pixell/qnx/read_pixell $USER@$IPADDR:$DEST_DIR/read_pixell_leddar
scp -P $PORT -p $TOP_REPO_DIR/path/sens/lidar/leddar/pixell/qnx/clt_vars_pixell $USER@$IPADDR:$DEST_DIR/clt_vars_pixell_leddar
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/avcs/qnx/read_pixell $USER@$IPADDR:$DEST_DIR/read_pixell_avcs
#scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/avcs/qnx/clt_vars_pixell $USER@$IPADDR:$DEST_DIR/clt_vars_pixell__avcs
scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/common/long_out $USER@$IPADDR:$DEST_DIR
scp -P $PORT -p $TOP_REPO_DIR/path/sens/lidar/leddar/sight/src/qnx/leddar_SIGHT $USER@$IPADDR:$DEST_DIR
#scp -P $PORT -p $TOP_REPO_DIR/truckcontrol/src/xyl/qnx/long_trk $USER@$IPADDR:$DEST_DIR
