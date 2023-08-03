#!/bin/sh
slay -f can_tx
slay -f argonne_cacc
slay -f accord_can
slay -f can_rx
slay -f clt_vars_accord
slay -f db_slv
cd /home/qnxuser/path_can_bin
DATETIME=`date +%m%d%Y_%H%M%S`
./db_slv -Q -S `hostname` &
sleep 1	
./clt_vars_taurus &
sleep 1	
./can_rx -p 1 -cv >can1_rx_out_$DATETIME.txt &
