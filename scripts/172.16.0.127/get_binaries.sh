#!/bin/sh

DATETIME=`date +%m%d%Y_%H%M%S`
echo "Must execute as qnxuser"
cd /home/qnxuser/path_can_bin
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/accord/can/src/qnx/clt_vars_accord .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/accord/can/src/qnx/accord_can .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/vehcomm/qnx/veh_snd .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/vehcomm/qnx/veh_rcv .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/vehcomm/realtime.ini .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/taurus/can/src/qnx/clt_vars_taurus .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/taurus/can/src/qnx/taurus_can .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/prius/can/src/qnx/clt_vars_prius .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/prius/can/src/qnx/prius_can .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/path/sens/gps/examples/qnx/gpssetdate .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/steinhoff/qnx/can_tx .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/steinhoff/qnx/can_rx .
scp jspring@172.16.0.150:/windows/Linux/Argonne_CACC/path/db/qnx/db_slv .
