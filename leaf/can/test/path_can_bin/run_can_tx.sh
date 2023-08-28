#!/bin/sh
slay -f can_rx
slay -f accord_can
slay -f clt_vars_accord
slay -f db_slv
cd /home/qnxuser/path_can_bin
./db_slv -Q -S `hostname` &
sleep 1	
./clt_vars_accord &
sleep 1	
./can_tx -t 0.02 -a 1 -b 2 -cv &
