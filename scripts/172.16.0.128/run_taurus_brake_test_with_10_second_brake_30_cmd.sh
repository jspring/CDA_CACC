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
sleep 1
./can_tx -i 0x99 -s 2 -p 1 -t 0.02 -v -c -n 4000 >can_tx_out_brake_$DATETIME.txt &

sleep 1
./can_tx -i 0x98 -s 1 -p 2 -t 0.02 -v -c -n 5000 >can_tx_out_accel_$DATETIME.txt &
sleep 1
./taurus_can -v -b -10 >taurus_can_out_$DATETIME.txt &
sleep 1
slay -f taurus_can
./taurus_can -v -b -10 >taurus_can_out_$DATETIME.txt &
sleep 1
slay -f taurus_can
slay -f can_tx
slay -f taurus_can
slay -f can_rx
slay -f clt_vars_taurus
slay -f db_slv
