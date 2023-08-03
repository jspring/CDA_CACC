#!/bin/sh
slay -f can_tx
cd /home/qnxuser/path_can_bin
./can_tx -i 0x98 -s 1 -p 2 -t 0.02 -v -A 40
sleep 5
slay -f can_tx
