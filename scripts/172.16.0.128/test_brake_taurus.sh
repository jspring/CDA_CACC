#!/bin/sh
slay -f can_tx
cd /home/qnxuser/path_can_bin
./can_tx -i 0x99 -s 2 -p 1 -t 0.01 -v -B -10 &
sleep 3
slay -f can_tx
./can_tx -i 0x99 -s 2 -p 1 -t 0.01 -v -B -100 
sleep 10
slay -f can_tx
