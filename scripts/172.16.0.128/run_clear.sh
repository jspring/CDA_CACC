#!/bin/sh

DESC=_$1
DATETIME=`date +%m%d%Y_%H%M%S`
cd /home/qnxuser/path_can_bin
slay -f can_tx
../bin/slayscript.sh leddar
slay -f argonne_cacc
slay -f taurus_can
slay -f can_rx
slay -f clt_vars_taurus
slay -f db_slv
slay -f hmi_server
./db_slv -Q -S `hostname` &
sleep 1	
./hmi_server -e 2 & 
sleep 1
./clt_vars_taurus &
sleep 1	
./can_rx -p 4 -cv >can2_rx_out_$DATETIME$DESC.txt &
sleep 1
./can_tx -i 0x99 -s 2 -p 1 -t 0.02 -c -n 4000 >can_tx_out_brake_$DATETIME$DESC.txt &
sleep 1	
./can_tx -i 0x98 -s 1 -p 2 -t 0.02 -c -n 5000 >can_tx_out_accel_$DATETIME$DESC.txt &
sleep 1
./veh_snd -A 172.16.0.128 -a 172.16.1.128 -u 1516 -i 50 -t taur >veh_snd_$DATETIME$DESC.txt 2>&1 &
sleep 1
./veh_rcv -A 172.16.0.128 -a 172.16.1.128 -u 15000 -o 3 >veh_rcv_out_$DATETIME.txt 2>&1 & 
sleep 1
echo "Starting taurus_can"
sleep 1
#../bin/run_leddar.sh &
#./leddar -p /dev/ser1 -d > leddar.out & 
#sleep 1
./taurus_can  >taurus_can_out_$DATETIME$DESC.txt &
sleep 1
#echo "Starting argonne_cacc"
#./argonne_cacc -m -e 2 -i 1 -c 0 -s 12 -f cacc_setup_taurus.cfg >argonne_cacc_$DATETIME$DESC.txt &
#tail -f argonne_cacc_$DATETIME$DESC.txt
