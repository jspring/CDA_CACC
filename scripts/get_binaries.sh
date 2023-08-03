#!/bin/sh

DATETIME=`date +%m%d%Y_%H%M%S`
echo "Must execute as qnxuser"
cd /home/qnxuser/path_can_bin
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/accord/can/src/qnx/clt_vars_accord clt_vars_accord.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/accord/can/src/qnx/accord_can accord_can.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/vehcomm/qnx/veh_snd veh_snd.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/vehcomm/qnx/veh_rcv veh_rcv.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/vehcomm/realtime.ini .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/taurus/can/src/qnx/clt_vars_taurus clt_vars_taurus.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/taurus/can/src/qnx/taurus_can taurus_can.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/prius/can/src/qnx/clt_vars_prius clt_vars_prius.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/prius/can/src/qnx/prius_can prius_can.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/path/sens/gps/examples/qnx/gpssetdate .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/steinhoff/qnx/can_tx can_tx.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/steinhoff/qnx/can_rx can_rx.after_$DATETIME
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/path/db/qnx/db_slv db_slv.after_$DATETIME
