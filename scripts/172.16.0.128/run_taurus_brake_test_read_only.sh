#!/bin/sh
slay -f can_tx
slay -f argonne_cacc
slay -f taurus_can
slay -f can_rx
slay -f clt_vars_taurus
slay -f db_slv
DATETIME=`date +%m%d%Y_%H%M%S` 

cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1	
./clt_vars_taurus &
sleep 1	
./can_rx -p 1 -cv >can1_rx_out_$DATETIME.txt 2>&1 &
sleep 1
./can_rx -p 2 -v >can2_rx_out_$DATETIME.txt 2>&1 &
sleep 10
slay -f can_tx
slay -f argonne_cacc
slay -f taurus_can
slay -f can_rx
slay -f clt_vars_taurus
slay -f db_slv
