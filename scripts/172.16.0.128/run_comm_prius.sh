#!/bin/sh
slay -f can_tx
slay -f argonne_cacc
slay -f prius_can
slay -f can_rx
slay -f clt_vars_prius
slay -f db_slv
DATETIME=`date +%m%d%Y_%H%M%S` 

cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1	
./clt_vars_prius &
sleep 1	
./can_rx -p 1 -cv >can1_rx_out_$DATETIME.txt 2>&1 &
sleep 1
./veh_snd -A 172.16.0.127 -a 172.16.1.127 -u 1516 -v >veh_snd_out_$DATETIME.txt 2>&1 &
sleep 1	
./veh_rcv -A 172.16.0.127 -a 172.16.1.127 -u 15000 -v -o 3 >veh_rcv_out_$DATETIME.txt 2>&1 &
tail -f veh_rcv_out_$DATETIME.txt
