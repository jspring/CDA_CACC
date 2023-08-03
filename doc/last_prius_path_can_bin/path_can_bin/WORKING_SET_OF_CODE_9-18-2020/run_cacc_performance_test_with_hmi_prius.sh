#!/bin/sh

IPADDR=`ifconfig en0 | grep "inet " |gawk '{print $2}'` 
DSRCADDR=`echo $IPADDR | sed '{s/172.16.0/172.16.1/g}'` 

slay -f argonne_cacc
slay -f prius_can
slay -f gpsrcv
slay -f veh_snd
slay -f veh_rcv
slay -f can_tx
slay -f can_rx
slay -f clt_vars_prius
slay -f db_slv
slay -f hmi_server

DATETIME=`date +%m%d%Y_%H%M%S` 

cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1	
./hmi_server -e 1 &
sleep 1
./clt_vars_prius &
sleep 1	
#./can_rx -p 1 -cv >can1_rx_out_$DATETIME.txt &
./can_rx -p 1 -c >can1_rx_out_$DATETIME.txt &
sleep 1	
./can_rx -p 2 >obd2_rx_out_$DATETIME.txt &
sleep 1
./can_tx -i 0x99 -s 2 -p 1 -t 0.02 -n 4000 >can_tx_out_brake_$DATETIME.txt &
sleep 1	
./can_tx -i 0x7fd -s 8 -p 2 -t 0.02 -n 5000 >can_tx_obd2_$DATETIME.txt &
sleep 1
#./gpsrcv -u 5115 -n 8001 -v >gpsrcv_$DATETIME.txt 2>&1 & 
#sleep 1
./veh_rcv  -A $IPADDR -a $DSRCADDR -u 15000 -o 3 -t priu >veh_rcv_out_$DATETIME.txt &
sleep 1	
./veh_snd  -A $IPADDR -a $DSRCADDR -u 1516 -t priu -i 20 >veh_snd_out_$DATETIME.txt &
sleep 1	
./prius_can  >prius_can_out_$DATETIME.txt &
#./prius_can -v 
sleep 1
./argonne_cacc -e 1 -i 2 -c 2 -x -h -f cacc_setup_prius.cfg >/home/qnxuser/data/argonne_cacc_$DATETIME.txt &
tail -f /home/qnxuser/data/argonne_cacc_$DATETIME.txt
