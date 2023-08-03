#!/bin/sh
slay -f can_tx
cd /home/qnxuser/path_can_bin
./can_tx -i 0x99 -s 2 -p 1 -t 0.02 -v -B -3 &
sleep .5
slay -f can_tx
