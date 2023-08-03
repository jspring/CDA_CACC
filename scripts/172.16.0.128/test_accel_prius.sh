#!/bin/sh
slay -f can_tx
cd /home/qnxuser/path_can_bin
./can_tx -i 0x99 -s 2 -p 1 -t 0.02 -v -A 5 
#./can_tx -i 0x99 -s 2 -p 1 -t 0.02 -v -A 2 &
#sleep 2
#slay -f can_tx
