#!/bin/sh
slay -f can_tx
slay -f argonne_cacc
slay -f prius_can
slay -f can_rx
slay -f clt_vars_prius
slay -f db_slv
cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1	
./clt_vars_prius &
sleep 1	
./can_rx -p 1 -cv >can1_rx_out.txt &
tail -f can1_rx_out.txt
