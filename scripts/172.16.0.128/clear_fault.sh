#!/bin/sh
slay -f can_tx
cd /home/qnxuser/path_can_bin
./can_tx -i 0x4 -s 0 -p 1 -t 0.01 -vd 
sleep 3
slay -f can_tx
