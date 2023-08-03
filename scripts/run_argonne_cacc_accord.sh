#!/bin/sh
slay -f can_tx
slay -f argonne_cacc
slay -f accord_can
slay -f can_rx
slay -f clt_vars_accord
slay -f db_slv
DATETIME=`date +%m%d%Y_%H%M%S` 

cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1	
./clt_vars_accord &
sleep 1	
./can_rx -p 1 -c >can1_rx_out_$DATETIME.txt 2>&1 &
sleep 1
./can_rx -p 2 -cv >can2_rx_out_$DATETIME.txt 2>&1 &
sleep 1
./can_rx -p 3 -v >can3_rx_out_$DATETIME.txt 2>&1 &
sleep 1
./can_rx -p 4  >can4_rx_out_$DATETIME.txt 2>&1 &
sleep 1
./can_tx -i 0x99 -s 6 -p 1 -t 0.02 -v -n 4000 >can_tx_out_brake_$DATETIME.txt 2>&1 &
sleep 1	
./can_tx -i 0x98 -s 1 -p 2 -t 0.02 -v -n 5000 >can_tx_out_accel_$DATETIME.txt 2>&1 &
sleep 1
./veh_rcv -A 172.16.0.120 -a 172.16.1.120 -u 15000 -v -o 3 -t Blue >veh_rcv_out_$DATETIME.txt 2>&1 &
sleep 1	
./veh_snd -A 172.16.0.120 -a 172.16.1.120 -u 1516 -t Blue -i 100 >veh_snd_out_$DATETIME.txt 2>&1 &
sleep 1	
./accord_can >accord_can_out_$DATETIME.txt 2>&1 &
sleep 1
./argonne_cacc >/home/qnxuser/data/argonne_cacc_$DATETIME.txt 2>&1 &
tail -f /home/qnxuser/data/argonne_cacc_$DATETIME.txt


