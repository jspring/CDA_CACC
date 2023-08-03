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
./can_rx -p 1 -cv >can1_rx_out.txt &
sleep 1
./can_rx -p 3 -v >can3_rx_out.txt &
tail -f can3_rx_out.txt
