#!/bin/sh

DESC=_$1
DATETIME=`date +%m%d%Y_%H%M%S`
slay -f can_tx
slay -f argonne_cacc
slay -f taurus_can
slay -f can_rx
slay -f clt_vars_taurus
slay -f db_slv
cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1	
./clt_vars_taurus &
sleep 1	
./can_rx -p 4 -cv >can2_rx_out_$DATETIME$DESC.txt &
sleep 1
./can_tx -i 0x99 -s 2 -p 1 -t 0.02 -c -n 4000 >can_tx_out_brake_$DATETIME$DESC.txt &
sleep 1	
./can_tx -i 0x98 -s 1 -p 2 -t 0.02 -c -n 5000 >can_tx_out_accel_$DATETIME$DESC.txt &
sleep 1
#./taurus_can -b -2 >taurus_can_out.txt &
#sleep 1
#slay taurus_can
echo "Starting taurus_can"
./taurus_can >taurus_can_out_$DATETIME$DESC.txt &
sleep 1
echo "Starting argonne_cacc"
./argonne_cacc -i 1 -e 2 -s 8 -c 0 >argonne_cacc_$DATETIME$DESC.txt &
tail -f argonne_cacc_$DATETIME$DESC.txt
