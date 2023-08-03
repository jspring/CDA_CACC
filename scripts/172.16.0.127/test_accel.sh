#!/bin/sh
slay -f can_tx
cd /home/qnxuser/path_can_bin
./can_rx -p4 -vd &
./can_tx -i 0x98 -s 1 -p 2 -t 0.02 -A 46
sleep 5
slay -f can_tx
