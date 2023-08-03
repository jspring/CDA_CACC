#!/bin/sh
slay -f can_tx
slay -f argonne_cacc
slay -f accord_can
slay -f can_rx
slay -f clt_vars_accord
slay -f db_slv
cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1	
./clt_vars_accord &
sleep 1	
./can_rx -p 4 -vd >can4_rx_out.txt &
sleep 1
./can_rx -p 1 -cv >can1_rx_out.txt &
sleep 1
./can_rx -p 3 -v >can3_rx_out.txt &
#./can_rx -p 3 -v &
sleep 1
./can_tx -i 0x99 -s 6 -p 1 -t 0.02 -v -n 4000 >can_tx_out_brake.txt 2>/dev/null &
sleep 1	
./can_tx -i 0x98 -s 1 -p 2 -t 0.01 -v -n 5000 >can_tx_out_accel.txt 2>/dev/null &
#sleep 1
./accord_can -w -a 30 >accord_can_out.txt &
sleep 60
slay accord_can
#./accord_can -v -b 30 >>accord_can_out.txt &
#sleep 3
#slay accord_can
#./accord_can -v >accord_can_out.txt 
#./accord_can  -w
