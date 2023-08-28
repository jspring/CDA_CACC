#!/bin/sh
slay -f accord_can
slay -f can_tx
slay -f clt_vars_accord
slay -f db_slv
cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1	
./clt_vars_accord &
sleep 1	
./can_tx -t 0.02 -a 1 -b 2 -c &
sleep 1	
./accord_can -v -a 40 &
sleep 4
slay -f accord_can
./accord_can -v -b 100 &
sleep 1
slay -f accord_can
slay -f clt_vars_accord
slay -f can_tx
slay -f db_slv
